"""
main.py
Description: 

Author: Willian T. Lunardi
Contact: wtlunar@gmail.com
License:

Repository:
"""

import time

import numpy as np

from channel_quality_estimator import ChannelQualityEstimator
from options import Options
from preprocessor import Preprocessor
from util import print_channel_quality, map_freq_to_channel
from wireless_scanner import WirelessScanner


def main():
    args = Options()

    # Initialize scanner, preprocessor, and estimator objects
    scanner = WirelessScanner(args)
    prep = Preprocessor()
    estimator = ChannelQualityEstimator()

    while True:
        # Perform a full scan of the current frequency band
        channel, scan = scanner.scan_all_channels()
        # Preprocess the scan
        feat_array, frequencies = prep.preprocess(scan)
        # Estimate the channel quality
        channels_quality, probs = estimator.estimate(feat_array)

        # Check quality of current channel
        index = args.all_channels.index(channel)
        current_channel_quality = channels_quality[index]
        if current_channel_quality > args.threshold:
            print(f'Current channel {channel}, estimated quality: {current_channel_quality}')
        else:
            # Check if there is at least one channel under the threshold within the current band
            if any(quality.item() > args.threshold for quality in channels_quality):
                # Pick the channel with the highest quality
                best_freq = frequencies[np.argmax(channels_quality)]
                # Map the frequency to the corresponding channel
                best_channel = map_freq_to_channel(best_freq)
                # Set the device to use the best channel
                scanner.set_channel(best_channel)
            else:
                # Scan the other frequency band
                channel, scan = scanner.scan_other_band()
                # Preprocess the scan
                feat_array, frequencies_other = prep.preprocess(scan)
                # Estimate channel quality for the other band
                channels_quality_other = estimator.estimate(feat_array)
                # Print channel quality and frequencies
                print_channel_quality(args, channels_quality_other, frequencies_other)

                # Find the best channel between the current and other bands
                best_quality_both = max(max(channels_quality), max(channels_quality_other))
                best_freq_both = frequencies[np.argmax(channels_quality)] if best_quality_both in channels_quality else frequencies_other[np.argmax(channels_quality_other)]
                best_channel_both = map_freq_to_channel(best_freq_both)
                # Set the device to use the best channel from both bands
                scanner.set_channel(best_channel_both)

        # Sleep T waiting_time seconds
        time.sleep(args.waiting_time)


if __name__ == '__main__':
    main()
