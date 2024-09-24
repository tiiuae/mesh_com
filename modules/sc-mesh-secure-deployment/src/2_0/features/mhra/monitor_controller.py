from traffic_monitor import TrafficMonitor
from routing_manager import *
import time
import logging
from mhra_comms import send_enable_on_demand_radio

class MonitorController:
    def __init__(self, multi_radio_manager, config, always_on_radio, on_demand_radio):
        self.multi_radio_manager = multi_radio_manager
        self.config = config
        self.always_on_radio = always_on_radio
        self.on_demand_radio = on_demand_radio
        self.on_demand_radio_status = False
        self.monitor = None
        self.on_demand_radio_transition_in_progress = False  # Flag to prevent multiple transitions
        self.dwell_time = self.config['traffic_monitor'].get('dwell_time', 60)  # Default dwell time is 60 seconds
        self.last_on_demand_enabled_time = None

    def start_always_on_monitoring(self):
        # Stop the existing monitor (if any)
        if self.monitor is not None:
            logging.info(f"Stopping the existing traffic monitor on {self.monitor.iface}")
            self.monitor.stop_monitoring()

        logging.info(f"Starting traffic monitoring on always-on radio {self.always_on_radio}!")
        self.monitor = TrafficMonitor(
            iface=self.always_on_radio,
            monitor_interval=self.config['traffic_monitor']['monitor_interval'],
            threshold_low=-1,
            threshold_high=self.config['traffic_monitor']['threshold_high'],
            traffic_threshold_cb=self.always_on_traffic_threshold_callback,
            error_threshold=self.config['traffic_monitor']['traffic_error_threshold'],
            error_cb=self.always_on_tx_error_callback
        )
        self.monitor.start_monitoring()

    def always_on_traffic_threshold_callback(self, tx_rate, rx_rate):
        # If the radio is already on or a transition is in progress, discard the request
        if self.on_demand_radio_status or self.on_demand_radio_transition_in_progress:
            logging.info("On-demand radio is already on or in transition. Discarding request.")
            return

        if tx_rate > self.config['traffic_monitor']['threshold_high'] or rx_rate > self.config['traffic_monitor']['threshold_high']:
            logging.warning(f"Traffic rate exceeded the high threshold! Tx: {tx_rate} kbps, Rx: {rx_rate} kbps")
            self.handle_high_traffic()

    def always_on_tx_error_callback(self, errors):
        # If the radio is already on or a transition is in progress, discard the request
        if self.on_demand_radio_status or self.on_demand_radio_transition_in_progress:
            logging.info("On-demand radio is already on or in transition. Discarding request.")
            return

        logging.warning(f"Errors exceeded the threshold! Current error rate: {errors}%")
        self.handle_high_traffic()

    def on_demand_radio_enable_req(self):
        """
        Handle a request to enable the on-demand radio.
        Check if the on-demand radio is already on or a transition is in progress.
        If so, discard the request.
        """
        # If the radio is already on or a transition is in progress, discard the request
        if self.on_demand_radio_status or self.on_demand_radio_transition_in_progress:
            logging.info("On-demand radio is already on or in transition. Discarding request.")
            return

        self.handle_high_traffic(send_remote_request=False)


    def handle_high_traffic(self, send_remote_request=True):
        # Set the transition flag to indicate that enabling is in progress
        self.on_demand_radio_transition_in_progress = True

        # Stop monitoring on the always-on radio
        logging.info(f"Stopping the traffic monitor on {self.monitor.iface}!")
        self.monitor.stop_monitoring()

        logging.warning(f"Starting {self.on_demand_radio} mesh to handle high traffic...")
        # Start the on-demand mesh
        self.multi_radio_manager.start_mesh(self.on_demand_radio, 1)

        # Send message to CBMA to enable security interface & attach to batman-adv for routing
        # TBD
        # For timebeing, attach mesh interface directly to bat0
        add_interface_to_batman("bat0", self.on_demand_radio)

        # Remove if any Throughput override is applied.
        restore_batman_routing(self.on_demand_radio)

        # Start monitoring on the on-demand radio
        self.start_on_demand_monitoring()

        # Set the current time as the time the on-demand radio was enabled
        self.last_on_demand_enabled_time = time.time()
        self.on_demand_radio_status = True
        self.on_demand_radio_transition_in_progress = False

        # Request other nodes also enable the on-demand mesh
        if send_remote_request:
            send_enable_on_demand_radio() 


    def start_on_demand_monitoring(self):
        # Stop the existing monitor (if any)
        if self.monitor is not None:
            logging.info(f"Stopping the existing traffic monitor on {self.monitor.iface}")
            self.monitor.stop_monitoring()

        logging.info(f"Starting traffic monitoring on the on-demand radio {self.on_demand_radio}...")
        self.monitor = TrafficMonitor(
            iface=self.on_demand_radio,
            monitor_interval=self.config['traffic_monitor']['monitor_interval'],
            threshold_low=self.config['traffic_monitor']['threshold_low'],
            threshold_high=1000000,
            traffic_threshold_cb=self.on_demand_traffic_threshold_callback
        )
        self.monitor.start_monitoring()

    def on_demand_traffic_threshold_callback(self, tx_rate, rx_rate):
        if tx_rate <= self.config['traffic_monitor']['threshold_low'] and rx_rate <= self.config['traffic_monitor']['threshold_low']:
            logging.warning(f"Traffic has dropped below the lower thresholds on {self.monitor.iface}. Tx: {tx_rate} kbps, Rx: {rx_rate} kbps")
            self.handle_low_traffic()

    def handle_low_traffic(self):
        # Check on-demand radio enable time, if its less than dwell time, do not disable.
        # Get the current time
        current_time = time.time()

        # If the on-demand radio was never enabled, just return
        if self.last_on_demand_enabled_time is None:
            logging.warning("On-demand radio has not been enabled yet.")
            return

        # Calculate the elapsed time since the on-demand radio was enabled
        elapsed_time = current_time - self.last_on_demand_enabled_time

        # Check if the elapsed time is less than the dwell time
        if elapsed_time < self.dwell_time:
            logging.info(f"Cannot disable the on-demand radio yet. Dwell time remaining: {self.dwell_time - elapsed_time:.2f} seconds")
            return

        # Stop the on-demand monitoring
        logging.info(f"Stopping the traffic monitor on {self.monitor.iface}!")
        self.monitor.stop_monitoring()

        # Apply Throughput override using Routing Manager so that switching of traffic will be smooth
        logging.info(f"Applying Throughput override for {self.on_demand_radio} iface!")
        override_batman_routing(self.on_demand_radio)

        # Wait for 10 seconds, complete override takes time due to EWMA (Exponentially weighted moving average)
        time.sleep(10) 

        # Disable the on-demand mesh
        logging.warning(f"Stopping on-demand radio {self.on_demand_radio} mesh!")
        self.multi_radio_manager.stop_mesh(self.on_demand_radio, 1)

        # Reset the last_on_demand_enabled_time
        self.last_on_demand_enabled_time = None

        # Resume monitoring on the always-on radio
        self.start_always_on_monitoring()

        # Reset flag
        self.on_demand_radio_transition_in_progress = False
        self.on_demand_radio_status = False

    def check_monitoring(self):
        if self.monitor is not None:
            self.monitor.check_monitoring()
