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

from options import parse_option
from util import print_channel_quality, map_freq_to_channel
from channel_quality_estimator import ChannelQualityEstimator
from wireless_scanner import WirelessScanner
from preprocessor import Preprocessor


def main():
    args = parse_option()

    # Initialize scanner, preprocessor, and estimator objects
    scanner = WirelessScanner(args)
    prep = Preprocessor()
    estimator = ChannelQualityEstimator()

    while True:
        # Perform a low latency spectral scan
        channel, scan = scanner.low_latency_spectral_scan()
        if(scan.empty): continue
        # Preprocess the scan
        feat_array, frequencies = prep.preprocess(scan)
        # Estimate the channel quality
        channel_quality = estimator.estimate(feat_array)
        # Print channel quality and frequencies
        print_channel_quality(args, channel_quality, frequencies)

        # If channel quality is below the threshold
        if channel_quality < args.threshold:
            # Perform a full scan of the current frequency band
            channel, scan = scanner.scan_current_band()
            if(scan.empty): continue
            # Preprocess the scan
            feat_array, frequencies = prep.preprocess(scan)
            # Estimate the channel quality
            channels_quality = estimator.estimate(feat_array)
            # Print channel quality and frequencies
            print_channel_quality(args, channels_quality, frequencies)

            # Check if there is at least one channel under the threshold
            if any(quality > args.threshold for quality in channels_quality):
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

        time.sleep(args.waiting_time)


if __name__ == '__main__':
    main()
