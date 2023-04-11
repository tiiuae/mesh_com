"""
wireless_scanner.py
Description: 

Author: Willian T. Lunardi
Contact: wtlunar@gmail.com
License:

Repository:
"""

import os
from typing import List

import pandas as pd

from options import Options
from util import Band, map_channel_to_freq, map_channel_to_band, load_sample_data


class WirelessScanner:
    def __init__(self, args: Options):
        """
        Initializes the WirelessScanner object.

        :param args: A Namespace object containing command line arguments.
        """
        self.args = args
        if not args.debug:
            from SpectralMgr import Spectral
            self.spec = Spectral()

        # self.freq = get_current_channel()
        # self.channel = map_freq_to_channel(self.freq)
        self.channel = args.channels5[0]  # TODO: get current frequency from mesh interface upon resolving low latency scan
        self.band = map_channel_to_band(self.channel)

    def get_all_channels(self) -> List[int]:
        """
        Get all available channels for the device.

        :return: A list of all available channels for the device.
        """
        return self.args.all_channels

    def get_available_channels(self, band: Band) -> List[int]:
        """
        Get the available channels in the specified band.

        :param band: A Band enum specifying the frequency band to get available channels for.
        :return: A list of available channels in the specified band.
        """
        # Replace this function with actual code to get the available channels in the specified band
        return self.args.channels2 if band == Band.BAND_24GHZ else self.args.channels5

    def low_latency_spectral_scan(self) -> tuple[int, pd.DataFrame]:
        """
        Runs a low-latency spectral scan on the current channel.

        :return: A tuple containing the current channel as an integer and a pandas DataFrame containing the spectral
        scan data for the chosen CSV file.
        """
        freq = map_channel_to_freq(self.channel)

        if not self.args.debug:
            # Perform spectral scan on given the current frequency
            freq = str(freq)
            scan = self.scan(freq)
        else:
            # Sample locally a .csv
            message, scan = load_sample_data()
            print(f'Low latency - sampled {message}')
            # Filter the data to only include the current channel's frequency
            scan = scan[scan['freq1'] == map_channel_to_freq(self.channel)]

        return self.channel, scan

    def scan_all_channels(self) -> tuple[int, pd.DataFrame]:
        """
        Runs a high-latency spectral scan on both bands (all channels).

        :return: A tuple containing the current channel as an integer and a pandas DataFrame containing the spectral
        scan data for the chosen CSV file.
        """
        channels = self.get_all_channels()
        freqs = [map_channel_to_freq(channel) for channel in channels]

        if not self.args.debug:
            # Perform spectral scan on the given frequencies of the current band
            freqs = ' '.join(str(value) for value in freqs)
            scan = self.scan(freqs)
        else:
            # Sample locally a .csv
            message, scan = load_sample_data()
            print(f'High latency current band - sampled {message}')
            # Filter the data to only include the available channel's frequency
            scan = scan[scan['freq1'].isin(freqs)]

        return self.channel, scan

    def scan_current_band(self) -> tuple[int, pd.DataFrame]:
        """
        Runs a high-latency spectral scan on the available channels in the current band.

        :return: A tuple containing the current channel as an integer and a pandas DataFrame containing the spectral
        scan data for the chosen CSV file.
        """
        channels = self.get_available_channels(self.band)
        freqs = [map_channel_to_freq(channel) for channel in channels]

        if not self.args.debug:
            # Perform spectral scan on the given frequencies of the current band
            freqs = ' '.join(str(value) for value in freqs)
            scan = self.scan(freqs)
        else:
            # Sample locally a .csv
            message, scan = load_sample_data()
            print(f'High latency current band - sampled {message}')
            # Filter the data to only include the available channel's frequency
            scan = scan[scan['freq1'].isin(freqs)]

        return self.channel, scan

    def scan_other_band(self) -> tuple[int, pd.DataFrame]:
        """
        Runs a high-latency spectral scan on the available channels in the other band.

        :return: A tuple containing the current channel as an integer and a pandas DataFrame containing the spectral
        scan data for the chosen CSV file.
        """
        other_band = Band.BAND_50GHZ if self.band == Band.BAND_24GHZ else Band.BAND_24GHZ
        channels = self.get_available_channels(other_band)
        freqs = [map_channel_to_freq(channel) for channel in channels]

        if not self.args.debug:
            # Perform spectral scan on the given frequencies of the other band
            freqs = ' '.join(str(value) for value in freqs)
            scan = self.scan(freqs)
        else:
            # Sample locally a .csv
            message, scan = load_sample_data('floor')
            print(f'High latency other band - sampled {message}')
            # Filter the data to only include the available channel's frequency
            scan = scan[scan['freq1'].isin(freqs)]

        return self.channel, scan

    def set_channel(self, channel: int) -> None:
        """
        Sets the wireless interface to the given channel.

        :param channel: An integer specifying the channel to set the wireless interface to.
        :return: None
        """
        print('moving to', channel, map_channel_to_freq(channel))
        # Replace this function with actual code to set the wireless interface to the given channel
        self.channel = channel
        self.band = Band.BAND_24GHZ if 1 <= channel <= 14 else Band.BAND_50GHZ

    def scan(self, freqs: str) -> pd.DataFrame:
        """
        Scan a list of frequencies and return a pandas DataFrame with the time series of each frequency.

        :param freqs: A string of frequencies to scan.
        :return: A pandas DataFrame containing the time series of each frequency.
        """
        while True:
            self.spec.initialize_scan()
            self.spec.execute_scan(freqs)
            f = self.spec.file_open("/tmp/data")
            file_stats = os.stat("/tmp/data")
            scan = self.spec.read(f, file_stats.st_size, freqs)
            self.spec.file_close(f)
            # Check if the scan is valid, if it is, then return it
            if self.is_valid_scan(freqs, scan):
                return scan

    def is_valid_scan(self, freqs: str, scan: pd.DataFrame) -> bool:
        """
        Check if the given scan is valid based on the minimum number of rows per frequency.

        :param freqs: A string of space-separated frequencies to validate (e.g., '5180 5200 5220').
        :param scan: A pandas DataFrame containing the time series of each frequency.
        :return: A boolean indicating whether the scan is valid or not.
        """
        freq_list = [int(freq) for freq in freqs.split()]
        for freq in freq_list:
            freq_scan = scan[scan['freq1'] == freq]
            if len(freq_scan) < self.args.min_rows:
                return False
        return True
