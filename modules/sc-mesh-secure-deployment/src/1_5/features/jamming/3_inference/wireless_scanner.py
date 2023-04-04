"""
WirelessScanner.py
Description: 

Author: Willian T. Lunardi
Contact: wtlunar@gmail.com
License: MIT License (https://opensource.org/licenses/MIT)

Repository: https://github.com/Willtl/voice-fingerprinting.git
"""
from argparse import Namespace
from typing import List, Any

from util import Band, map_channel_to_freq, load_sample_data, map_channel_to_band


class WirelessScanner:
    def __init__(self, args: Namespace):
        self.args = args
        self.channel = args.channels5[0]  # TODO: Implement function that loads interface's channel
        self.band = map_channel_to_band(self.channel)

    def get_available_channels(self, band: Band) -> List[int]:
        """
        Get the available channels in the specified band.

        :return: A list of available channels in the specified band.
        """
        # Replace this function with actual code to get the available channels in the specified band
        return self.args.channels2 if band == Band.BAND_24GHZ else self.args.channels5

    def low_latency_spectral_scan(self) -> tuple[Any, Any]:
        """
        Runs a low-latency spectral scan on the current channel.

        :return: A pandas DataFrame containing the spectral scan data for the chosen CSV file.
        """
        # Sample a csv
        message, scan = load_sample_data()
        print(f'Low latency - sampled {message}')

        # Filter the data to only include the current channel's frequency
        scan = scan[scan['freq1'] == map_channel_to_freq(self.channel)]  # TODO: change for low latency scan

        return self.channel, scan

    def scan_current_band(self):
        channels = self.get_available_channels(self.band)
        freqs = [map_channel_to_freq(channel) for channel in channels]

        message, scan = load_sample_data()
        print(f'High latency current band - sampled {message}')

        # Filter the data to only include the available channel's frequency
        scan = scan[scan['freq1'].isin(freqs)]  # TODO: change for scan on the available channels on the current band

        return self.channel, scan

    def scan_other_band(self):
        other_band = Band.BAND_50GHZ if self.band == Band.BAND_24GHZ else Band.BAND_24GHZ
        channels = self.get_available_channels(other_band)
        freqs = [map_channel_to_freq(channel) for channel in channels]

        message, scan = load_sample_data('floor')
        print(f'High latency other band - sampled {message}')

        # Filter the data to only include the available channel's frequency
        scan = scan[scan['freq1'].isin(freqs)]  # TODO: change for scan on the available channels on the current band

        return self.channel, scan

    def set_channel(self, channel: int) -> None:
        """
        Set the wireless interface to the given channel.

        :param channel: The channel to set the wireless interface to.
        """
        print('moving to', channel, map_channel_to_freq(channel))
        # Replace this function with actual code to set the wireless interface to the given channel
        self.channel = channel
        self.band = Band.BAND_24GHZ if 1 <= channel <= 14 else Band.BAND_50GHZ
