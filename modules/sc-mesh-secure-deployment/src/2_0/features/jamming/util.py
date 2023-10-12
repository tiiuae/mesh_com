import pathlib
import random
import re
import subprocess
from enum import Enum
from typing import Tuple

import netifaces
import numpy as np
import pandas as pd

from options import Options
args = Options()

CH_TO_FREQ = {1: 2412, 2: 2417, 3: 2422, 4: 2427, 5: 2432, 6: 2437, 7: 2442, 8: 2447, 9: 2452, 10: 2457, 11: 2462,
              36: 5180, 40: 5200, 44: 5220, 48: 5240, 52: 5260, 56: 5280, 60: 5300, 64: 5320, 100: 5500, 104: 5520,
              108: 5540, 112: 5560, 116: 5580, 120: 5600, 124: 5620, 128: 5640, 132: 5660, 136: 5680, 140: 5700,
              149: 5745, 153: 5765, 157: 5785, 161: 5805}
FREQ_TO_CH = {v: k for k, v in CH_TO_FREQ.items()}

FEATS_ATH10K = ['max_magnitude', 'total_gain_db', 'base_pwr_db', 'rssi', 'relpwr_db', 'avgpwr_db', 'snr', 'cnr', 'pn', 'ssi', 'pd', 'sinr', 'sir', 'mr', 'pr']


class Band(Enum):
    BAND_24GHZ = "2.4GHz"
    BAND_50GHZ = "5.0GHz"


def get_ipv6_addr(osf_interface) -> str:
    """
    Get the IPv6 address of the OSF network interface.

    :param osf_interface: The name of the network interface.
    :return: The IPv6 address as a string.
    """
    # Retrieve the IPv6 addresses associated with the osf_interface
    ipv6_addresses = netifaces.ifaddresses(osf_interface).get(netifaces.AF_INET6, [])
    if ipv6_addresses:
        for addr_info in ipv6_addresses:
            if 'addr' in addr_info and addr_info['addr'].startswith('fd'):
                return addr_info['addr']
            else:
                return None
    else:
        return None


def get_mesh_freq() -> int:
    """
    Get the mesh frequency of the device.

    :return: An integer representing the mesh frequency.
    """
    mesh_freq: int = np.nan

    try:
        iw_output = subprocess.check_output(['iw', 'dev'], encoding='utf-8')
        iw_output = re.sub(r'\s+', ' ', iw_output).split(' ')

        # Extract interface sections from iw_output
        idx_list = [idx - 1 for idx, val in enumerate(iw_output) if val == "Interface"]
        if len(idx_list) > 1:
            idx_list.pop(0)

        # Calculate the start and end indices for interface sections
        start_indices = [0] + idx_list
        end_indices = idx_list + ([len(iw_output)] if idx_list[-1] != len(iw_output) else [])

        # Use zip to create pairs of start and end indices, and extract interface sections
        iw_interfaces = [iw_output[start:end] for start, end in zip(start_indices, end_indices)]

        # Check if mesh interface is up and get freq
        for interface_list in iw_interfaces:
            try:
                if "mesh" in interface_list:
                    channel_index = interface_list.index("channel") + 2
                    mesh_freq = int(re.sub("[^0-9]", "", interface_list[channel_index]).split()[0])
                    break
            except Exception as e:
                print(f"Get mesh freq exception: {e}") if args.debug else None
    except Exception as e:
        print(f"Get mesh freq exception: {e}") if args.debug else None

    return mesh_freq


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

    :param freq: The frequency value.
    :return: The frequency.
    """
    return FREQ_TO_CH[freq]


def load_sample_data(args: Options = Options()) -> Tuple[str, pd.DataFrame]:
    """
    Load sample data from a CSV file based on a randomly chosen sample type.

    :param args: An Options object containing the command-line arguments.
    :return: A tuple containing the message describing the sample data and a pandas DataFrame with the loaded data.
    """
    sample_type_to_dir = {
        'floor': 'sample/floor/',
        'jamming': 'sample/jamming/',
        'background': 'sample/background/'
    }

    # Choose a random sample_type from the dictionary
    sample_type = random.choice(list(sample_type_to_dir.keys()))
    if sample_type not in sample_type_to_dir:
        raise ValueError("Invalid sample_type provided. Choose from 'floor', 'background', or 'jamming'.")

    csv_files = pathlib.Path(sample_type_to_dir[sample_type]).glob('*.csv')
    path = random.choice(list(csv_files))
    message = f'{sample_type.capitalize()} data {path}'

    # load the data into a pandas DataFrame
    data = pd.read_csv(path)
    print(f"Sampled {sample_type} data") if args.debug else None
    return message, data


def get_frequency_quality(freq_quality: np.ndarray, probs: np.ndarray, frequencies: np.ndarray) -> Tuple[bool, dict]:
    """
    Get frequency quality information based on frequency quality and interference probabilities.

    param freq_quality: NumPy array of frequency quality values.
    param probs: NumPy array of interference probabilities.
    param frequencies: NumPy array of frequencies to assess.

    return: A Tuple containing a boolean indicating if jamming was detected and a dictionary mapping frequencies to quality values.
    """
    jamming_detected = False
    freq_quality_dict = {}

    # Check if passed params freq_quality, probs, frequencies are empty
    if freq_quality.size == 0 or frequencies.size == 0 or probs.size == 0:
        return False, {}

    for i, freq in enumerate(frequencies):
        index = np.argmax(probs[i])
        freq_quality_dict[str(freq)] = float(freq_quality[i])
        if index == 0:
            pred = 'floor like interference'
        elif index == 1:
            pred = 'wireless lab. like interference'
        elif index == 2:
            pred = 'crypto/finance like interference'
        else:
            pred = 'jamming'
            jamming_detected = True

        print(freq, freq_quality[i], np.argmax(probs[i]), pred) if args.debug else None

    return jamming_detected, freq_quality_dict


# Frequency switch related functions
def run_command(command, error_message) -> None:
    """
    Execute a shell command and check for success.

    param command: The shell command to execute.
    param error_message: Error message to display if the command fails.

    return: True if the command was successful, False otherwise.
    """
    try:
        if args.debug:
            subprocess.call(command, shell=True)
        else:
            subprocess.call(command, shell=True, stderr=subprocess.STDOUT, stdout=subprocess.DEVNULL)
    except Exception as e:
        error_message = f"Error occurred: {error_message}"
        raise Exception(error_message) from e


def read_file(filename, error_message):
    """
    Read the content of a file.

    param filename: Name of the file to read.
    param error_message: Error message to display if reading the file fails.

    return: The content of the file as a string, or None if an error occurs.
    """
    try:
        with open(filename, 'r') as f:
            return f.read()
    except Exception as e:
        print(f"Error occurred while reading {filename}: {error_message}. Exception: {str(e)}") if args.debug else None
        return None


def write_file(filename, content, error_message) -> None:
    """
    Write content to a file.

    param filename: Name of the file to write to.
    param content: The content to write to the file.
    param error_message: Error message to display if writing to the file fails.
    """
    try:
        with open(filename, 'w') as f:
            f.write(content)
    except Exception as e:
        print(f"Error occurred while writing {filename}: {error_message}. Exception: {str(e)}") if args.debug else None


def is_process_running(process_name):
    """
    Check if a process with a given name is currently running.

    param process_name: The name of the process to check.

    return: True if the process is running, False otherwise.
    """
    try:
        process_list = subprocess.check_output(['ps', 'aux']).decode('utf-8').split('\n')
        for process in process_list:
            if process_name in process:
                return True
        return False
    except Exception as e:
        print(f"Error occurred while checking process: {str(e)}") if args.debug else None
        return False
