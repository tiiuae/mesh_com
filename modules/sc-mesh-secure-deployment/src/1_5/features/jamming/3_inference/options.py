import argparse
from typing import List


def parse_option():
    """
    Parse command line arguments for training.

    Returns:
        argparse.Namespace: The parsed arguments.
    """
    parser = argparse.ArgumentParser(description='Arguments for training.')
    parser.add_argument('--waiting_time', type=int, default=5, help='Time interval in seconds.')
    parser.add_argument('--channels2', type=List[int], default=[2410], help='Available channels on the 2.4GHz.')
    parser.add_argument('--channels5', type=List[int], default=[5180], help='Available channels on the 5GHz.')
    parser.add_argument('--threshold', type=float, default=0.5, help='Threshold on the probability of being jammed.')
    parser.add_argument('--lr', type=float, default=1e-4, help='Learning rate for optimizer.')
    parser.add_argument('--lr_scheduler', type=str, default=None, choices=[None, 'OneCycleCosine', 'CosineAnnealing'], help='Learning rate scheduler to use.')

    args = parser.parse_args()

    return args
