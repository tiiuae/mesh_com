import argparse
from typing import List


def parse_option():
    """
    Parse command line arguments for training.

    Returns:
        argparse.Namespace: The parsed arguments.
    """
    parser = argparse.ArgumentParser(description='Arguments for training.')
    parser.add_argument('--waiting_time', type=int, default=1, help='Time interval in seconds.')
    parser.add_argument('--channels2', type=List[int], default=[1, 6, 11], help='Available channels on the 2.4GHz.')
    parser.add_argument('--channels5', type=List[int], default=[36, 40, 44, 48, 149, 153, 157, 161, 165], help='Available channels on the 5GHz.')
    parser.add_argument('--threshold', type=float, default=0.20, help='Threshold on the probability of being jammed.')
    parser.add_argument('--interface', type=str, default='wlan0', help='Wireless interface name.')

    args = parser.parse_args()

    return args
