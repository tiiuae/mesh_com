"""
util.py
Description: 

Author: Willian T. Lunardi
Contact: wtlunar@gmail.com
License:
"""

import random
from enum import Enum
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

CH_TO_FREQ = {1: 2412, 2: 2417, 3: 2422, 4: 2427, 5: 2432, 6: 2437, 7: 2442, 8: 2447, 9: 2452, 10: 2457, 11: 2462,
              36: 5180, 40: 5200, 44: 5220, 48: 5240, 52: 5260, 56: 5280, 60: 5300, 64: 5320, 100: 5500, 104: 5520,
              108: 5540, 112: 5560, 116: 5580, 120: 5600, 124: 5620, 128: 5640, 132: 5660, 136: 5680, 140: 5700,
              149: 5745, 153: 5765, 157: 5785, 161: 5805, 165: 5825}
FREQ_TO_CH = {v: k for k, v in CH_TO_FREQ.items()}

FEATS_ATH10K = ['max_magnitude', 'total_gain_db', 'base_pwr_db', 'rssi', 'relpwr_db', 'avgpwr_db', 'snr', 'cnr', 'pn', 'ssi', 'pd', 'sinr', 'sir', 'mr', 'pr']


class Band(Enum):
    BAND_24GHZ = "2.4GHz"
    BAND_50GHZ = "5.0GHz"


def map_channel_to_freq(channel: int) -> int:
    """
    Maps the given channel number to its corresponding frequency int.

    :param channel: The channel number.
    :return: The frequency.
    """
    return CH_TO_FREQ[channel]


def map_freq_to_channel(freq: int) -> int:
    """
    Maps the given frequency to its corresponding channel number int.

    :param channel: The channel number.
    :return: The frequency.
    """
    return FREQ_TO_CH[freq]


def map_channel_to_band(channel: int) -> Band:
    """
    Maps the given channel number to its corresponding band enum.

    :param channel: The channel number to map to a band enum.
    :return: The band enum corresponding to the given channel number.
    """
    if channel in range(1, 14):
        # 2.4 GHz channels
        return Band.BAND_24GHZ
    elif channel in range(36, 166, 4):
        # 5 GHz channels
        return Band.BAND_50GHZ
    else:
        raise ValueError(f"Invalid channel number: {channel}.")


def load_sample_data(sample_type: Optional[str] = None):
    """
    Load sample data from a CSV file based on a randomly chosen sample type.

    :return: A tuple containing the message describing the sample data and a pandas DataFrame with the loaded data.
    """
    sample_types = ['floor', 'video', 'jamming']
    sample_type = random.choice(sample_types) if sample_type is None else sample_type

    if sample_type == 'floor':
        message = "Floor data"
        file_name = 'spectral_scan_floor_141.csv'
    elif sample_type == 'video':
        message = "Video data"
        file_name = 'spectral_scan_video_meshfreq5180_80mhz_718.csv'
    elif sample_type == 'jamming':
        message = "Jamming data"
        file_name = 'samples_chamber_5180MHz_0cm_13dBm_gaussiannoise_7.csv'
    else:
        raise ValueError("Invalid sample_type provided. Choose from 'floor', 'video', 'interference', or 'jamming'.")

    # load the data into a pandas DataFrame
    data = pd.read_csv('sample/' + file_name)

    return message, data


def plot_timeseries(X: np.ndarray, X_resized: np.ndarray) -> None:
    # Create the figure and subplots
    fig, axs = plt.subplots(nrows=2, sharex=True, figsize=(8, 6))

    # Plot the original signal on the top
    axs[0].plot(X)
    axs[0].set_title("Original Signal")

    # Plot the resized signal on the bottom
    axs[1].plot(X_resized)
    axs[1].set_title("Resized Signal")

    # Set the x-axis label
    fig.text(0.5, 0.04, 'Time', ha='center')

    # Set the y-axis labels
    axs[0].set_ylabel('Amplitude')
    axs[1].set_ylabel('Amplitude')

    # Display the plot
    plt.show()


def print_channel_quality(channels_quality: np.ndarray, frequencies: np.ndarray) -> None:
    for i, freq in enumerate(frequencies):
        if channels_quality[i] < 0.25:
            print(freq, channels_quality[i], 'BAD')
        else:
            print(freq, channels_quality[i], 'GOOD')
