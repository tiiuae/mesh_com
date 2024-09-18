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
        self.monitor = None
        self.transition_in_progress = False  # Flag to prevent multiple transitions

    def start_always_on_monitoring(self):
        logging.info(f"Starting traffic monitoring on always-on radio {self.always_on_radio}!")
        self.monitor = TrafficMonitor(
            iface=self.always_on_radio,
            monitor_interval=self.config['traffic_monitor']['monitor_interval'],
            threshold_low=-1,
            threshold_high=self.config['traffic_monitor']['threshold_high'],
            traffic_threshold_cb=self.traffic_threshold_callback,
            error_threshold=self.config['traffic_monitor']['traffic_error_threshold'],
            error_cb=self.tx_error_callback
        )
        self.monitor.start_monitoring()

    def traffic_threshold_callback(self, tx_rate, rx_rate):
        if not self.transition_in_progress:
            if tx_rate > self.config['traffic_monitor']['threshold_high'] or rx_rate > self.config['traffic_monitor']['threshold_high']:
                logging.warning(f"Traffic rate exceeded the high threshold! Tx: {tx_rate} kbps, Rx: {rx_rate} kbps")
                self.handle_high_traffic()

    def tx_error_callback(self, errors):
        if not self.transition_in_progress:
            self.transition_in_progress = True
            logging.warning(f"Errors exceeded the threshold! Current error rate: {errors}%")
            self.handle_high_traffic()

    def handle_high_traffic(self):
        # Stop monitoring on the always-on radio
        logging.info(f"Stopping the traffic monitor on {self.monitor.iface}!")
        self.monitor.stop_monitoring()

        logging.warning(f"Starting {self.on_demand_radio} mesh to handle high traffic...")
        # Start the on-demand mesh
        self.multi_radio_manager.start_mesh(self.on_demand_radio, 1)

        # Request other nodes also enable the on-demand mesh
        send_enable_on_demand_radio() 

        # Send message to CBMA to enable security interface & attach to batman-adv for routing
        # TBD
        # For timebeing, attach mesh interface directly to bat0
        add_interface_to_batman("bat0", self.on_demand_radio)

        # Remove if any Throughput override is applied.
        restore_batman_routing(self.on_demand_radio)

        # Wait for 60 seconds to avoid frequent enable/disable of on-demand mesh
        time.sleep(60)

        # Start monitoring on the on-demand radio
        self.start_on_demand_monitoring()

    def start_on_demand_monitoring(self):
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
        # Stop the on-demand monitoring
        logging.info(f"Stopping the traffic monitor on {self.monitor.iface}!")
        self.monitor.stop_monitoring()

        # Apply Throughput override using Routing Manager so that switching of traffic will be smooth
        logging.info(f"Applying Throughput override for {self.on_demand_radio} iface!")
        override_batman_routing(self.on_demand_radio)

        # Wait for 30 seconds, complete override takes time due to EWMA (Exponentially weighted moving average)
        time.sleep(30) 

        # Disable the on-demand mesh
        logging.warning(f"Stopping on-demand radio {self.on_demand_radio} mesh!")
        self.multi_radio_manager.stop_mesh(self.on_demand_radio, 1)

        # Reset flag
        self.transition_in_progress = False

        # Resume monitoring on the always-on radio
        self.start_always_on_monitoring()
