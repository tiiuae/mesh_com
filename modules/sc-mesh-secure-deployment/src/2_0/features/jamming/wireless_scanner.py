import math
import re

import pandas as pd

from options import Options
from spectral_manager import Spectral
import util


class WirelessScanner:
    def __init__(self) -> None:
        """
        Initializes the WirelessScanner object.
        """
        self.args = Options()
        self.spec = Spectral()

    def get_band_frequencies(self, current_freq, scan_band) -> str:
        """
        Get all available frequencies for the device.

        :return: A list of available frequencies on the device for the specified band.
        """
        if scan_band == "5.0GHz":
            frequencies_list = [util.map_channel_to_freq(channel) for channel in self.args.channels5]
            frequencies = ' '.join(str(freq) for freq in frequencies_list if freq != current_freq)
        else:
            print("Error: Scan band can should be 5.0GHz.") if self.args.debug else None
            frequencies = ''

        return frequencies

    def get_current_freq_sample_data(self) -> pd.DataFrame:
        """
        Reads a random CSV file from the specified folder and filters rows with a specific 'freq1' value.

        :return curr_freq_data: A DataFrame containing rows where the 'freq1' value matches the 'current_freq'
        """
        # Get current frequency and sample data
        curr_freq_data: pd.DataFrame = pd.DataFrame()
        current_freq: int = util.get_mesh_freq()
        message, scan = util.load_sample_data()
        if math.isnan(current_freq):
            return curr_freq_data

        # If jamming file selected, get the jammed frequency from file name
        if message.startswith("Jamming"):
            pattern = r"_(\d+)MHz"
            match = re.search(pattern, message)
            if match:
                # Filter for the jammed frequency rows, replace their freq1 value with current mesh frequency value
                jammed_frequency = int(match.group(1).strip())
                jammed_frequency_data = scan[scan['freq1'] == jammed_frequency]
                curr_freq_data = jammed_frequency_data.copy()
                curr_freq_data['freq1'] = current_freq
                # Convert the 'freq1' column in curr_freq_data to int64
                curr_freq_data['freq1'] = curr_freq_data['freq1'].astype('int64')
            else:
                print("Frequency not found in the file path.") if self.args.debug else None
        else:
            curr_freq_data = scan.copy()
            # Filter for current mesh frequency rows
            curr_freq_data = curr_freq_data[curr_freq_data['freq1'] == current_freq]
            # Convert the 'freq1' column in curr_freq_data to int64
            curr_freq_data['freq1'] = curr_freq_data['freq1'].astype('int64')

        return curr_freq_data

    def scan_current_frequency(self) -> pd.DataFrame:
        """
        Run a low latency spectral scan.

        :return: A pandas DataFrame containing the spectral scan data for the current frequency.
        """
        current_freq: str = str(util.get_mesh_freq())
        # Perform spectral scan on the given frequencies of the current band
        scan = self.scan(current_freq)
        return scan

    def scan_all_frequencies(self) -> pd.DataFrame:
        """
        Runs a high-latency spectral scan on specified bands.

        :return: A pandas DataFrame containing the spectral scan data.
        """
        current_freq: int = util.get_mesh_freq()
        frequencies = self.get_band_frequencies(current_freq, '5.0GHz')
        # Perform spectral scan on the given frequencies of the current band
        scan = self.scan(frequencies)
        return scan

    def scan(self, frequencies: str) -> pd.DataFrame:
        """
        Scan a list of frequencies and return a pandas DataFrame.

        :param frequencies: A string of frequencies to scan.
        :return full_scan: A pandas DataFrame containing the spectral scan performed for the passed frequencies.
        """
        # Remove any present binary dump files
        bin_file = "/tmp/data"
        cmd_clear_binary_file = f"rm {bin_file}"
        clear_binary_file_error_message = f"Failed to run {bin_file}"
        util.run_command(cmd_clear_binary_file, clear_binary_file_error_message)

        try:
            driver: str = self.spec.get_driver()
            self.spec.initialize_scan(driver)
            self.spec.execute_scan(frequencies, driver, bin_file)
            self.spec.read(bin_file)
            scan = self.spec.create_dataframe(driver)
            print("scan: ", scan) if self.args.debug else None

            # Check if the scan is valid, if it is, then return it
            if self.is_valid_scan(scan):
                if self.args.debug:
                    curr_freq = util.get_mesh_freq()
                    curr_freq_data = self.get_current_freq_sample_data()
                    filtered_scan_dataframe = scan[scan['freq1'] != curr_freq]
                    full_scan = pd.concat([curr_freq_data, filtered_scan_dataframe], ignore_index=True)
                    return full_scan
                else:
                    return scan
        except Exception as e:
            print(f"Error in scan {e}: ") if self.args.debug else None

    def is_valid_scan(self, scan: pd.DataFrame) -> bool:
        """
        Check if the given scan is valid based on whether it is empty or not.

        :param scan: A pandas DataFrame containing the spectral scan for each passed frequency.
        :return: A boolean indicating whether the scan is valid or not.
        """
        if scan.empty:
            return False
        else:
            return True
