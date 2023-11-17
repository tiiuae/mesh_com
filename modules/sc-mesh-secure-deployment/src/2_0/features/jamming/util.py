import sys
import pathlib
import random
import re
import subprocess
import asyncio
import json
import time
from enum import Enum
from typing import Tuple

import netifaces
import numpy as np
import pandas as pd
from options import Options
from log_config import logger

args = Options()

sys.path.append(args.nats_scripts_path)
import client
import config

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
                if "mesh" in interface_list and "channel" in interface_list:
                    channel_index = interface_list.index("channel") + 2
                    mesh_freq = int(re.sub("[^0-9]", "", interface_list[channel_index]).split()[0])
                    break
            except Exception as e:
                logger.error(f"Get mesh freq exception: {e}")
    except Exception as e:
        logger.error(f"Get mesh freq exception: {e}")

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


async def switch_frequency(frequency: str) -> None:
    """
    Change the mesh frequency to the specified frequency using NATS.
    """
    logger.info(f"Moving to {frequency} MHz ...")

    # Connect to NATS
    nc = await client.connect_nats()

    cmd_dict = {"frequency": frequency, "radio_index": args.radio_index}
    cmd = json.dumps(cmd_dict)

    try:
        await nc.request(f"comms.channel_change.{config.MODULE_IDENTITY}", cmd.encode(), timeout=10)
        logger.info(f"Published to comms.channel_change: {cmd}")
        await asyncio.sleep(15)
    except Exception as e:
        logger.error(f"Failed to change channel: {e}")
    finally:
        await nc.close()


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
    logger.info(f"Sampled {sample_type} data")
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
        freq_quality_dict[str(freq)] = round(float(freq_quality[i]), 5)
        if index == 0:
            pred = 'floor like interference'
        elif index == 1:
            pred = 'wireless lab. like interference'
        elif index == 2:
            pred = 'crypto/finance like interference'
        else:
            pred = 'jamming'
            jamming_detected = True

        logger.info(f"{freq} {freq_quality[i]} {np.argmax(probs[i])} {pred}")

    return jamming_detected, freq_quality_dict


def run_command(command, error_message) -> None:
    """
    Execute a shell command and check for success.

    param command: The shell command to execute.
    param error_message: Error message to display if the command fails.
    """
    try:
        # Run the command, redirecting both stdout and stderr to the log file
        with open(args.log_file, 'a') as subprocess_output:
            # Redirect the output to the log file
            return_code = subprocess.call(command, shell=False, stdout=subprocess_output, stderr=subprocess_output)

        if return_code != 0:
            logger.error(f"Command failed with return code {return_code}")
    except Exception as e:
        logger.error(f"{error_message}. Error: {e}")
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
        logger.error(f"Error occurred while reading {filename}: {error_message}. Exception: {e}")
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
        logger.error(f"Error occurred while writing {filename}: {error_message}. Exception: {e}")


def is_process_running(process_name: str) -> bool:
    """
    Check if a process with a given name is currently running.

    param process_name: The name of the process to check.

    return: True if the process is running, False otherwise.
    """
    try:
        ps_output = subprocess.check_output(['ps', 'aux']).decode('utf-8')
        if process_name in ps_output:
            return True
        else:
            return False
    except subprocess.CalledProcessError as e:
        logger.error(f"Error occurred while checking process: {e}")
        return False


def get_pid_by_process_name(process_name: str) -> int:
    """
    Get the Process ID (PID) of a process by its name.

    param: The name of the process to search for.

    return: The PID of the process if found, or 0 if the process is not found.
    """
    try:
        # List processes and filter by name
        ps_output = subprocess.check_output(['ps', 'aux'], text=True)
        for line in ps_output.split('\n'):
            if process_name in line:
                pid = int(line.split()[0])
                return pid
        return 0  # Process not found
    except subprocess.CalledProcessError:
        return 0


def kill_process_by_pid(process_name: str) -> None:
    """
    Attempt to kill a process by its name.

    param: The name of the process to be killed.
    """
    # Retrieve the Process ID (PID) of the specified process
    pid = get_pid_by_process_name(process_name)
    try:
        subprocess.check_output(['kill', str(pid)])
    except subprocess.CalledProcessError as e:
        logger.error(f"Failed to kill process: {e}")  # Failed to kill the process
