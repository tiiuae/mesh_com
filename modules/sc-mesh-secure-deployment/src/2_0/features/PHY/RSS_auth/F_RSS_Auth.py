import csv
import threading
import subprocess
import time
from datetime import datetime
from tabulate import tabulate
import os

class RSS_Auth:
    def __init__(self, start_script, mac_address_file, rssi_file):
        self.start_script = start_script
        self.mac_address_file = mac_address_file
        self.rssi_file = rssi_file
        self.default_macs = []
        self.current_macs = []
        self.rssi_avg = []
        self.updated_rssi_avg = []
        self.threshold = 4
        self.process = None
        self.results = {'Pass': [], 'Fail': []}

    def start(self):
        """Starts the RSSI capturing process in a separate thread."""
        # Starts the RSSI capturing script as a subprocess
        self.process = subprocess.Popen(["python3", self.start_script], stdout=subprocess.PIPE)

        # Start the run method in a separate thread
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def stop(self):
        """Stops the RSSI capturing process."""
        if self.thread and self.thread.is_alive():
            # Set a flag here to stop the run method loop if necessary
            self.thread.join()
        if self.process:
            self.process.terminate()
            self.process.wait()
            self.process = None

    def start_rssi_capture(self):
        """Starts the RSSI capturing script as a subprocess."""
        self.process = subprocess.Popen(["python3", self.start_script], stdout=subprocess.PIPE)

    def stop_rssi_capture(self):
        """Stops the RSSI capturing script."""
        if self.process:
            self.process.terminate()
            self.process = None

    def log_authentication(self, mac, result):
        log_file_path = './authentication_log.csv'
        """Log the authentication result to a CSV file."""
        try:
            with open('authentication_log.csv', mode='a', newline='') as file:
                writer = csv.writer(file)
                # Write headers if file is empty
                if file.tell() == 0:
                    writer.writerow(['Date/Time', 'MAC Address', 'Result'])
                # Write the authentication result
                writer.writerow([datetime.now().strftime('%Y-%m-%d %H:%M:%S'), mac, result])
        except Exception as e:
            print(f"An error occurred while logging: {e}")

    def load_mac_addresses(self, filename):
        macs = []
        file_path = os.path.join('./', filename)
        try:
            with open(filename, 'r') as csv_file:
                reader = csv.reader(csv_file)
                next(reader)  # Skip header
                for row in reader:
                    if row:  # Skip empty rows
                        macs.append(row)
        except FileNotFoundError:
            print(f"File {filename} not found.")
        except Exception as e:
            print(f"An error occurred: {e}")
        return macs

    def print_macs(self, macs, description):
        # Printing MAC addresses for demonstration
        print(f"\n{description}")
        print(tabulate(macs, headers=['MAC Addresses']))

    def load_rssi_values(self, filename):
        rssi_values = []
        file_path = os.path.join('./', filename)
        try:
            with open(filename, 'r') as csv_file:
                reader = csv.reader(csv_file)
                for row in reader:
                    rssi_values.append(row)
        except FileNotFoundError:
            print(f"File {filename} not found.")
        except Exception as e:
            print(f"An error occurred: {e}")
        return rssi_values

    def calculate_average_rssi(self, rssi_values):
        averages = []
        for row in rssi_values:
            values = [float(val) for val in row if self.is_float(val)]
            if values:
                averages.append(sum(values) / len(values))
        return averages
    
    @staticmethod
    def is_float(value):
        try:
            float(value)
            return True
        except ValueError:
            return False

    def authenticate_nodes(self):
        additional_macs = set(tuple(mac) for mac in self.current_macs) - set(tuple(mac) for mac in self.default_macs)
        missing_macs = set(tuple(mac) for mac in self.default_macs) - set(tuple(mac) for mac in self.current_macs)
        validated_nodes = set()
        intruder_nodes = set()

        for i, mac in enumerate(self.current_macs):
            mac_tuple = tuple(mac)  # Convert list to tuple for comparison
            if mac_tuple in additional_macs:
                print(f"New node detected with MAC {mac}, Checks needed.")
                self.log_authentication('-'.join(mac), 'Fail')
                intruder_nodes.add(mac_tuple)
            elif mac_tuple in missing_macs:
                print(f"Node with MAC {mac} has left the network.")
                self.log_authentication('-'.join(mac), 'Left')
            else:
                if abs(self.updated_rssi_avg[i] - self.rssi_avg[i]) <= self.threshold:
                    validated_nodes.add(mac_tuple)
                    self.log_authentication('-'.join(mac), 'Pass')
                else:
                    print(f"Node with MAC {mac} has different RSS: Checks required.")
                    self.log_authentication('-'.join(mac), 'Fail')
                    intruder_nodes.add(mac_tuple)

    # Update results without unpacking tuple
        self.results['Pass'] = [list(mac) for mac in validated_nodes]
        self.results['Fail'] = [list(mac) for mac in intruder_nodes]
        return validated_nodes, intruder_nodes


    def get_result(self):
        """
        Returns the latest results of the RSS authentication process.
        """
        # Return a copy of the results to avoid unintentional modifications
        return self.results.copy()


    def run(self):
        self.default_macs = self.load_mac_addresses(self.mac_address_file)
        self.print_macs(self.default_macs, "Default MAC Addresses:")
        default_rssi_values = self.load_rssi_values(self.rssi_file)
        self.rssi_avg = self.calculate_average_rssi(default_rssi_values)

        while True:  # Continuous monitoring loop
            self.current_macs = self.load_mac_addresses(self.mac_address_file)
            self.print_macs(self.current_macs, "Updated MAC Addresses:")
            updated_rssi_values = self.load_rssi_values(self.rssi_file)
            self.updated_rssi_avg = self.calculate_average_rssi(updated_rssi_values)

            validated_nodes, intruder_nodes = self.authenticate_nodes()
            print("Validated Nodes:", validated_nodes)
            print("Intruder Nodes:", intruder_nodes)

            time.sleep(5)  # The sleep time can be adjusted as needed

# The 'if __name__ == "__main__":' block is omitted as it's not needed for module use


#if __name__ == "__main__":
    # Example usage
   # rss_auth = RSS_Auth("F_RSSI_Capture_v1.py", "mac_addresses.csv", "output_processed.csv")
    #rss_auth.start()
    # Assume the script runs for a certain period or until a condition is met
    #time.sleep(60)  # Example: run for 60 seconds
    #rss_auth.stop()

