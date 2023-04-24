import argparse
from typing import List

class Options:
    def __init__(self):
        self.waiting_time: int = 1
        self.channels2: List[int] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
        self.channels5: List[int] = [36, 40, 44, 48, 52, 56, 60, 64, 149, 153, 157, 161, 165]
        self.all_channels: List[int] = []
        self.threshold: float = 0.0
        self.interface: str = 'wlp1s0'
        self.debug: bool = False
        self.low_latency: bool = True
        self.min_rows: int = 16
        self.parse_options()

    def parse_options(self) -> 'Options':
        parser = argparse.ArgumentParser(description='Arguments for training.')
        parser.add_argument('--waiting_time', type=int, default=self.waiting_time, help='Time interval in seconds.')
        parser.add_argument('--channels2', type=lambda x: [int(i) for i in x.split(',')], default=self.channels2,
                            help='Available channels on the 2.4GHz. Provide comma-separated values (e.g., 1,6,11).')
        parser.add_argument('--channels5', type=lambda x: [int(i) for i in x.split(',')], default=self.channels5,
                            help='Available channels on the 5GHz. Provide comma-separated values (e.g., 36,40,44,48,149,153,157,161,165).')
        parser.add_argument('--threshold', type=float, default=self.threshold, help='Threshold on the probability of being jammed.')
        parser.add_argument('--interface', type=str, default=self.interface, help='Wireless interface name.')
        parser.add_argument('--debug', type=bool, default=self.debug, help='Use local random sampling of .csv as scan instead of the actual spectral scan.')
        parser.add_argument('--low_latency', type=bool, default=self.debug,
                            help='Perform low latency on the current channel. If False, force high latency scan on current band.')
        parser.add_argument('--min_rows', type=int, default=self.min_rows, help='Minimum number of rows of a given frequency within a scan so that the scan is valid.')

        args = parser.parse_args()

        self.waiting_time = args.waiting_time
        self.channels2 = args.channels2
        self.channels5 = args.channels5
        self.all_channels = self.channels2 + self.channels5
        self.threshold = args.threshold
        self.interface = args.interface
        self.debug = args.debug
        self.min_rows = args.min_rows

        return self
