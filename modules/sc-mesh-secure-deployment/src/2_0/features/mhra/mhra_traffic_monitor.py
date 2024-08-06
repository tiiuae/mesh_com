import time

class MhraTrafficMonitor:
    """
    A class to monitor network traffic on a specific network interface.

    Attributes:
        iface (str): Network interface name.
        monitor_interval (int): Interval for monitoring in seconds.
        tx_error_threshold (int): Threshold for transmission error percentage.
        tx_error_cb (callable): Callback for transmission error threshold.
        tx_threshold_low (int): Lower threshold for transmission rate in kbps.
        tx_threshold_high (int): Upper threshold for transmission rate in kbps.
        tx_threshold_cb (callable): Callback for transmission rate threshold.
        rx_threshold_low (int): Lower threshold for reception rate in kbps.
        rx_threshold_high (int): Upper threshold for reception rate in kbps.
        rx_threshold_cb (callable): Callback for reception rate threshold.
    """
    
    def __init__(self, iface="halow1", monitor_interval=10,
                 tx_error_threshold=10, tx_error_cb=None, 
                 tx_threshold_low=-1, tx_threshold_high=300, tx_threshold_cb=None,
                 rx_threshold_low=-1, rx_threshold_high=300, rx_threshold_cb=None):
        self.iface = iface
        self.tx_rate = 0  # in kbps
        self.rx_rate = 0  # in kbps
        self.tx_errors = 0  # in %
        self.monitoring = False
        self.monitor_interval = monitor_interval  # in seconds
        
        self.tx_error_threshold = tx_error_threshold  # in %
        self.tx_error_cb = tx_error_cb
        
        self.tx_threshold_low = tx_threshold_low  # in kbps
        self.tx_threshold_high = tx_threshold_high  # in kbps
        self.tx_threshold_cb = tx_threshold_cb
        
        self.rx_threshold_low = rx_threshold_low  # in kbps
        self.rx_threshold_high = rx_threshold_high  # in kbps
        self.rx_threshold_cb = rx_threshold_cb

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
            print(f"Interface {self.iface} not found.")
            return None

    def _get_rx_bytes(self):
        """Retrieve the received bytes for the interface."""
        try:
            with open(f'/sys/class/net/{self.iface}/statistics/rx_bytes', 'r') as f:
                return int(f.read().strip())
        except FileNotFoundError:
            print(f"Interface {self.iface} not found.")
            return None

    def _get_tx_errors(self):
        """Retrieve the transmission error count for the interface."""
        try:
            with open(f'/sys/class/net/{self.iface}/statistics/tx_errors', 'r') as f:
                return int(f.read().strip())
        except FileNotFoundError:
            print(f"Interface {self.iface} not found.")
            return None

    def start_monitoring(self):
        """Start the traffic monitoring process."""
        self.monitoring = True
        print("Mhra Traffic Monitoring started.")
        
        self.prev_tx_bytes = self._get_tx_bytes()
        self.prev_rx_bytes = self._get_rx_bytes()
        self.prev_tx_errors = self._get_tx_errors()
        
        while self.monitoring:
            time.sleep(self.monitor_interval)
            self._monitor_traffic()
            
            self.report()  # For debugging purposes
            
            if self.tx_rate < self.tx_threshold_low or self.tx_rate > self.tx_threshold_high:
                if self.tx_threshold_cb:
                    self.tx_threshold_cb(self.tx_rate)

            if self.rx_rate < self.rx_threshold_low or self.rx_rate > self.rx_threshold_high:
                if self.rx_threshold_cb:
                    self.rx_threshold_cb(self.rx_rate)

            if self.tx_errors >= self.tx_error_threshold:
                if self.tx_error_cb:
                    self.tx_error_cb(self.tx_errors)

    def stop_monitoring(self):
        """Stop the traffic monitoring process."""
        self.monitoring = False
        print("Mhra Traffic Monitoring stopped.")

    def _monitor_traffic(self):
        """Monitor the current traffic and update rates and errors."""
        current_tx_bytes = self._get_tx_bytes()
        current_rx_bytes = self._get_rx_bytes()
        current_tx_errors = self._get_tx_errors()

        if current_tx_bytes is not None and current_tx_errors is not None:
            tx_byte_difference = current_tx_bytes - self.prev_tx_bytes

            if tx_byte_difference > 0:
                self.tx_rate = (tx_byte_difference * 8) / (self.monitor_interval * 1024)  # kbps
                error_difference = current_tx_errors - self.prev_tx_errors
                self.tx_errors = (error_difference * 100) / tx_byte_difference if tx_byte_difference > 0 else 0

            self.prev_tx_errors = current_tx_errors
            self.prev_tx_bytes = current_tx_bytes
            
        if current_rx_bytes is not None:
            rx_byte_difference = current_rx_bytes - self.prev_rx_bytes

            if rx_byte_difference > 0:
                self.rx_rate = (rx_byte_difference * 8) / (self.monitor_interval * 1024)  # kbps

            self.prev_rx_bytes = current_rx_bytes

    def report(self):
        """Print the current transmission and reception rates and error percentage."""
        print(f"Tx Rate: {self.tx_rate} kbps, Rx Rate: {self.rx_rate} kbps, Error %: {self.tx_errors}")

    def set_tx_thresholds(self, low, high):
        """Set new thresholds for transmission rates."""
        self.tx_threshold_low = low
        self.tx_threshold_high = high
        print(f"Tx thresholds updated to low: {low} kbps, high: {high} kbps.")

    def set_rx_thresholds(self, low, high):
        """Set new thresholds for reception rates."""
        self.rx_threshold_low = low
        self.rx_threshold_high = high
        print(f"Rx thresholds updated to low: {low} kbps, high: {high} kbps.")

    def set_monitor_interval(self, interval):
        """Set a new monitoring interval."""
        self.monitor_interval = interval
        print(f"Monitoring interval updated to {interval} seconds.")

    def set_error_threshold(self, threshold):
        """Set a new threshold for transmission errors."""
        self.tx_error_threshold = threshold
        self.tx_error_cb = callback if callback else self.tx_error_cb
        print(f"Error threshold updated to {threshold}%. Callback updated.")

# Example usage
if __name__ == "__main__":
    def error_handler(error_rate):
        print(f"Tx Errors {error_rate}% exceeding the threshold!")

    def tx_handler(tx_rate):
        print(f"Tx rate {tx_rate} kbps is exceeding the threshold!")
        
    def rx_handler(rx_rate):
        print(f"Rx rate {rx_rate} kbps is exceeding the threshold!")

    try:
        monitor = MhraTrafficMonitor(iface="halow1", monitor_interval=10, 
                                     tx_error_threshold=15, tx_error_cb=error_handler, 
                                     tx_threshold_low=-1, tx_threshold_high=300, tx_threshold_cb=tx_handler,
                                     rx_threshold_low=-1, rx_threshold_high=300, rx_threshold_cb=rx_handler)
        monitor.start_monitoring()

        # Example of changing thresholds and interval during monitoring
        time.sleep(30)
        monitor.set_tx_thresholds(5, 500)
        monitor.set_rx_thresholds(-1, 500)
        monitor.set_monitor_interval(15)
        monitor.set_error_threshold(15)

    except PermissionError as e:
        print(e)
    except KeyboardInterrupt:
        monitor.stop_monitoring()
        monitor.report()

