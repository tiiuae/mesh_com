import math
import re
import pandas as pd

from options import Options
from options import VALID_CHANNELS
from spectral_manager import Spectral
import util
from log_config import logger


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
            logger.info("Error: Scan band can should be 5.0GHz.")
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
                curr_freq_data['freq1'] = curr_freq_data['freq1'].astype('int64')
            else:
                logger.info("Frequency not found in the file path.")
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
        # Scan binary dump file
        bin_file = "/tmp/data"

        # Validate that passed frequencies list contains valid frequencies, if not, filter invalid
        try:
            freq_list = [int(freq) for freq in frequencies.split(' ')]
            channel_list = [util.map_freq_to_channel(freq) for freq in freq_list]
            if not all(channel in VALID_CHANNELS for channel in channel_list):
                # Filter out invalid frequencies
                valid_channel_list = [channel for channel in channel_list if channel in VALID_CHANNELS]
                freq_list = [util.map_channel_to_freq(freq) for freq in valid_channel_list]
                frequencies = ' '.join(map(str, freq_list))
        except Exception as e:
            logger.error(f"{e}")
            return pd.DataFrame()

        try:
            # Get driver
            driver: str = self.spec.get_driver()
            # Initialize scan
            self.spec.initialize_scan(driver)
            # Execute scan
            self.spec.execute_scan(frequencies, driver, bin_file)
            # Read binary dump file
            self.spec.read(bin_file)
            # Create scan dataframe
            scan = self.spec.create_dataframe(driver)
            logger.info(f"Scan: {scan}")

            # Check if the scan is valid, if it is, return it
            valid_scan = self.validate_scan(frequencies, scan)
            if not valid_scan.empty:
                if self.args.debug:
                    curr_freq = util.get_mesh_freq()
                    curr_freq_data = self.get_current_freq_sample_data()
                    filtered_scan_dataframe = scan[scan['freq1'] != curr_freq]
                    full_scan = pd.concat([curr_freq_data, filtered_scan_dataframe], ignore_index=True)
                    return full_scan
                else:
                    return scan
        except Exception as e:
            logger.info(f"Error in scan {e}")

    def validate_scan(self, freqs: str, scan: pd.DataFrame) -> pd.DataFrame:
        """
        Return valid scan based on the minimum number of rows per frequency.

        :param freqs: A string of space-separated frequencies to validate (e.g., '5180 5200 5220').
        :param scan: A pandas DataFrame containing the spectral scan for each passed frequency.

        :return: Scan after filtering for min number of rows.
        """
        if scan.empty:
            return scan
        else:
            # For each freq, check min number of scan samples met
            freq_list = [int(freq) for freq in freqs.split()]
            for freq in freq_list:
                freq_scan = scan[scan['freq1'] == freq]
                if len(freq_scan) < self.args.min_rows:
                    # Remove all rows with freq1 equal to the specific freq
                    scan = scan[scan['freq1'] != freq]
            return scan
