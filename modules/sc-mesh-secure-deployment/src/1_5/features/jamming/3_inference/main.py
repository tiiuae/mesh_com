import random

from options import parse_option

normal_folders = ['communication', 'floor', 'inter_mid', 'inter_high']
max_files = 1000

from enum import Enum
from typing import List


class Band(Enum):
    BAND_2_4GHZ = "2.4GHz"
    BAND_5GHZ = "5GHz"


class WirelessScanner:
    def __init__(self, interface: str):
        self.interface = interface
        self.current_band = self.get_current_band()
        self.current_channel = self.get_current_channel()

    def get_available_channels(self, band: Band) -> List[int]:
        """
        Get the available channels in the specified band.

        :param band: The band to retrieve channels for.
        :return: A list of available channels in the specified band.
        """
        # Replace this function with actual code to get the available channels in the specified band
        return [1, 6, 11] if band == Band.BAND_2_4GHZ else [36, 40, 44, 48]

    def low_latency_spectral_scan(self, channel: int) -> None:
        """
        Run a low-latency spectral scan on the given channel.

        :param channel: The channel to perform the low-latency spectral scan on.
        """
        # Replace this function with actual code to run a low-latency spectral scan on the given channel
        pass

    def spectral_scan(self, channel: int) -> None:
        """
        Run a spectral scan on the given channel.

        :param channel: The channel to perform the spectral scan on.
        """
        # Replace this function with actual code to run a spectral scan on the given channel
        pass

    def set_channel(self, channel: int) -> None:
        """
        Set the wireless interface to the given channel.

        :param channel: The channel to set the wireless interface to.
        """
        # Replace this function with actual code to set the wireless interface to the given channel
        self.current_channel = channel
        self.current_band = Band.BAND_2_4GHZ if 1 <= channel <= 14 else Band.BAND_5GHZ

    def get_current_band(self) -> Band:
        """
        Get the current band of the wireless scanner.

        :return: The current band.
        """
        return Band.BAND_5GHZ

    def get_current_channel(self) -> int:
        """
        Get the current channel of the wireless scanner.

        :return: The current channel.
        """
        return 5180


class ChannelQualityEstimator:
    @staticmethod
    def get_current_channel_quality(channel):
        # Replace this function with actual code to get the quality of the current channel
        return random.random()

    @staticmethod
    def get_channel_quality(channel):
        # Replace this function with actual code to get the quality of a specific channel
        return random.random()


def main():
    # Parse arguments
    args = parse_option()

    scanner = WirelessScanner(interface=args.interface)
    estimator = ChannelQualityEstimator()

    # Loop through each channel
    channel_list = [5180] if 'meshfreq' in file_name else CHANNELS
    for channel in channel_list:
        # Filter measurements for a particular channel within file
        df_channel = df.loc[df['freq1'] == channel]

        # Ignore timeseries with low number of measurements
        if len(df_channel) < 64:
            continue

        # Down sample by interpolating mean
        df_serie = resize_to_length(df_channel)

        # Set serie id
        df_serie['series_id'] = series_id
        df_serie['bin_label'] = 0
        df_serie['label'] = folder_name

        # Concatenate
        all_series.append(df_serie)
        series_id += 1

    print(df)
    quit()
    preprocess_raw_files()
    store_all_data()
    # split_data()


if __name__ == '__main__':
    main()
