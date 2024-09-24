import time
import logging

class TrafficMonitor:
    def __init__(self, iface="halow1", monitor_interval=10,
                 threshold_low=-1, threshold_high=300, 
                 traffic_threshold_cb=None, 
                 error_threshold=10, error_cb=None):
        self.iface = iface
        self.tx_rate = 0  # in kbps
        self.rx_rate = 0  # in kbps
        self.tx_errors = 0  # in %
        self.monitoring = False
        self.monitor_interval = monitor_interval  # in seconds

        self.threshold_low = threshold_low  # in kbps
        self.threshold_high = threshold_high  # in kbps
        self.traffic_threshold_cb = traffic_threshold_cb

        self.error_threshold = error_threshold  # in %
        self.error_cb = error_cb

        # Initialize previous values
        self.prev_tx_bytes = self._get_tx_bytes()
        self.prev_rx_bytes = self._get_rx_bytes()
        self.prev_tx_errors = self._get_tx_errors()

        # Check if interface statistics are accessible
        if self.prev_tx_bytes is None or self.prev_rx_bytes is None or self.prev_tx_errors is None:
            raise PermissionError(f"Cannot read from /sys/class/net/{self.iface}/statistics. Check permissions or interface name.")

    def _get_tx_bytes(self):
        """Retrieve the transmitted bytes for the interface."""
        try:
            with open(f'/sys/class/net/{self.iface}/statistics/tx_bytes', 'r') as f:
                return int(f.read().strip())
        except FileNotFoundError:
            logging.warning(f"Interface {self.iface} not found.")
            return None

    def _get_rx_bytes(self):
        """Retrieve the received bytes for the interface."""
        try:
            with open(f'/sys/class/net/{self.iface}/statistics/rx_bytes', 'r') as f:
                return int(f.read().strip())
        except FileNotFoundError:
            logging.warning(f"Interface {self.iface} not found.")
            return None

    def _get_tx_errors(self):
        """Retrieve the transmission error count for the interface."""
        try:
            with open(f'/sys/class/net/{self.iface}/statistics/tx_errors', 'r') as f:
                return int(f.read().strip())
        except FileNotFoundError:
            logging.warning(f"Interface {self.iface} not found.")
            return None

    def start_monitoring(self):
        """Start the traffic monitoring process."""
        self.monitoring = True
        logging.info("Traffic Monitoring started.")
        
        self.prev_tx_bytes = self._get_tx_bytes()
        self.prev_rx_bytes = self._get_rx_bytes()
        self.prev_tx_errors = self._get_tx_errors()
        
    def check_monitoring(self):
        """Check the traffic stats periodically."""
        self._monitor_traffic()
        self.report()
        # Trigger traffic threshold callback if thresholds are crossed
        if (self.tx_rate > self.threshold_high or self.tx_rate < self.threshold_low or
            self.rx_rate > self.threshold_high or self.rx_rate < self.threshold_low):
            if self.traffic_threshold_cb:
                self.traffic_threshold_cb(self.tx_rate, self.rx_rate)

        if self.tx_errors >= self.error_threshold:
            if self.error_cb:
                self.error_cb(self.tx_errors)

    def stop_monitoring(self):
        """Stop the traffic monitoring process."""
        self.monitoring = False
        logging.info("Traffic Monitoring stopped.")

    def _monitor_traffic(self):
        """Monitor the current traffic and update rates and errors."""
        current_tx_bytes = self._get_tx_bytes()
        current_rx_bytes = self._get_rx_bytes()
        current_tx_errors = self._get_tx_errors()

        if current_tx_bytes is not None and current_tx_errors is not None:
            tx_byte_difference = current_tx_bytes - self.prev_tx_bytes

            if tx_byte_difference > 0:
                self.tx_rate = (tx_byte_difference * 8) / (self.monitor_interval * 1000)  # kbps
                error_difference = current_tx_errors - self.prev_tx_errors
                self.tx_errors = (error_difference * 100) / tx_byte_difference if tx_byte_difference > 0 else 0

            self.prev_tx_errors = current_tx_errors
            self.prev_tx_bytes = current_tx_bytes
            
        if current_rx_bytes is not None:
            rx_byte_difference = current_rx_bytes - self.prev_rx_bytes

            if rx_byte_difference > 0:
                self.rx_rate = (rx_byte_difference * 8) / (self.monitor_interval * 1000)  # kbps

            self.prev_rx_bytes = current_rx_bytes

    def report(self):
        """Print the current transmission and reception rates and error percentage."""
        logging.debug(
            f"Tx Rate: {self.tx_rate:.2f} kbps, Rx Rate: {self.rx_rate:.2f} kbps, Error %: {self.tx_errors:.2f}"
        )