import argparse
import warnings

from typing import NoReturn


class SnifferOptions:
    """
    Class to handle and store options for a packet sniffer.

    Attributes:
        dataset_path (str): Path to the dataset folder containing .pcap files.
        delay (bool): Flag to consider delay between packets when in debug mode.
        session (bool): Flag to consider bidirectional flows if True, otherwise unidirectional.
        interface (str): Name of the interface to sniff on.
        timeout (int): Time in seconds to consider a connection as terminated.
        cleaning_cycle (int): Time interval in seconds to check for inactive flows.
        debug (bool): Flag to enable or disable logging.
    """

    def __init__(self):
        self.dataset_path: str = '../../datasets/TII-SSRC-23/pcap/'
        self.delay: bool = False
        self.session: bool = False
        self.interface: str = 'br-lan'
        self.timeout: int = 120
        self.cleaning_cycle: int = 60
        self.debug: bool = True

    def parse_options(self) -> NoReturn:
        """
        Parse command-line arguments and update instance attributes.

        The method utilizes `argparse` to parse command-line options and updates
        the instance attributes accordingly.
        """
        parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)

        parser.add_argument('--dataset_path', type=str, default=self.dataset_path, help='Dataset folder path containing the .pcap files.')
        parser.add_argument('--delay', type=bool, default=self.delay, help='If debug true, then consider or not delay between packets while reading .pcap.')
        parser.add_argument('--session', type=bool, default=self.session, help='If true, consider bidirectional flows, otherwise unidirectional.')
        parser.add_argument('--interface', default=self.interface, help='interface to sniff')
        parser.add_argument('--timeout', type=int, default=self.timeout, help='seconds to consider connection teardown')
        parser.add_argument('--cleaning_cycle', type=int, default=self.cleaning_cycle, help='loop through flows and check if no packet was received since')
        parser.add_argument('--debug', type=bool, default=self.debug, help='turn on logging')

        args = parser.parse_args()

        self.dataset_path = args.dataset_path
        self.delay = args.delay
        self.session = args.session
        self.interface = args.interface
        self.timeout = args.timeout
        self.cleaning_cycle = args.cleaning_cycle
        self.debug = args.debug


class LDPIOptions:
    """
    Class to handle and store options for Lightweight Deep Packet Inspection (LDPI).

    Attributes:
        n (int): Number of packets per sample. Changing this value requires retraining the model.
        l (int): Size of each packet in the samples. Changing this value requires retraining the model.
        model_name (str): Name of the model used for detection ('MLP' or 'ResCNN').
        batch_size (int): Training batch size.
        pretrain_epochs (int): Number of epochs for pretraining with contrastive learning.
        epochs (int): Number of epochs for fine-tuning the model.
        threshold_type (str): Strategy for setting the anomaly detection threshold.
    """

    def __init__(self):
        # Initialize default values
        self.n: int = 4  # Number of packets per sample (default: 4)
        self.l: int = 60  # Size of each packet in the samples (default: 60)

        # Training related
        self.model_name: str = 'ResCNN'
        self.batch_size: int = 64  # Training batch size
        self.pretrain_epochs: int = 2000  # Epochs for pretraining
        self.epochs: int = 400  # Epochs for fine tuning

        # Inference related arguments
        self.threshold_type: str = 'max'

    def parse_options(self) -> NoReturn:
        """
        Parse command-line arguments and update instance attributes.

        Uses `argparse` to parse command-line options related to the LDPI system,
        including sniffing, buffering parameters, and anomaly detection sensitivity.
        Issues a warning if the user sets different values for 'n' or 'l', as these changes
        require retraining the model.
        """
        parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)

        # Sniffing and buffering params
        parser.add_argument('--n', type=int, default=self.n, help='Number of packets per sample. Changing this requires model retraining.')
        parser.add_argument('--l', type=int, default=self.l, help='Size of each packet in the samples. Changing this requires model retraining.')

        # Anomaly detection sensitivity parameters
        parser.add_argument('--model_name', choices=['MLP', 'ResCNN'], default=self.model_name,
                            help='Model used for detection (MLP or ResCNN)')
        parser.add_argument('--threshold_type', choices=['ninety_nine', 'near_max', 'max', 'hundred_one'], default=self.threshold_type,
                            help='Threshold strategy for anomaly detection. '
                                 '`ninety_nine` is the 99th percentile threshold. '
                                 '`near_max` is the 99.99th percentile threshold. '
                                 '`max` is the maximum threshold. '
                                 '`hundred_one` is 1% over the `max`.')

        args = parser.parse_args()

        # Update class attributes with parsed arguments
        self.n = args.n
        self.l = args.l
        self.model_name = args.model_name
        self.threshold_type = args.threshold_type

        # Issue a warning if 'n' or 'l' are changed from their default values
        if self.n != 4 or self.l != 60:
            warnings.warn(
                "Changing 'n' or 'l' requires retraining/fine-tuning the model. The only available pretrained model on TII-SSRC-23 dataset is with 4 packets of 60 bytes each.")
