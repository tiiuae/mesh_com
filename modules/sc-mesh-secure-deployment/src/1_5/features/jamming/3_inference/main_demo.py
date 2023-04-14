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
from util import print_channel_quality, map_freq_to_channel, map_channel_to_freq
from wireless_scanner import WirelessScanner

WORKING_FREQS = [5180, 5200, 5220, 5240, 5745, 5765, 5785, 5805, 5825]


def filter_working_freqs(frequencies: np.ndarray, channels_quality: np.ndarray):
    new_freqs, new_quality = [], []
    for freq, quality in zip(frequencies, channels_quality):
        if freq in WORKING_FREQS:
            new_freqs.append(freq)
            new_quality.append(quality)
    frequencies, channels_quality = np.array(new_freqs), np.array(new_quality)
    return frequencies, channels_quality


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
        channels_quality, probs = estimator.estimate(feat_array, frequencies)
        # Print channel quality and frequencies
        print_channel_quality(args, channels_quality, probs, frequencies)

        # Filter working frequencies
        frequencies, channels_quality = filter_working_freqs(frequencies, channels_quality)

        # Check quality of current channel
        # index = args.all_channels.index(channel)
        index = frequencies.tolist().index(map_channel_to_freq(channel))
        current_channel_quality = channels_quality[index]
        # if current_channel_quality > args.threshold:
        if False:
            print(f'Current channel {channel}, estimated quality: {current_channel_quality}')
        else:
            # Pick the channel with the highest quality
            best_freq = frequencies[np.argmax(channels_quality)]
            # Map the frequency to the corresponding channel
            best_channel = map_freq_to_channel(best_freq)
            # Set the device to use the best channel
            scanner.set_channel(best_channel)

        # Sleep T waiting_time seconds
        time.sleep(args.waiting_time)


if __name__ == '__main__':
    main()
