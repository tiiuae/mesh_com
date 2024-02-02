import time
import threading
from typing import NoReturn
import os
import sys

# Update imports path
ldpi_directory = os.path.abspath(os.path.dirname(__file__))
sys.path.append(ldpi_directory)
from ldpi.inference import LightDeepPacketInspection
from options import SnifferOptions
from sniffer.sniffer import Sniffer


class IDS:
    """
    This class initializes and run the packet sniffer and light deep packet inspection.
    """
    def __init__(self, decision_engine):
        self.stop_event = threading.Event()
        # Initialize command line arguments
        self.args = SnifferOptions()
        self.args.parse_options()
        # Initialize Sniffer and LightDeepPacketInspection instances
        self.snf = Sniffer(self.args)
        # Initialize LightDeepPacketInspection
        self.ldpi = LightDeepPacketInspection(decision_engine)

    def start(self):
        """
        This function initializes the `Sniffer` and `LightDeepPacketInspection` classes, starts the packet sniffer,
        and runs a decision engine loop that keeps the program running until interrupted.
        """
        # Register LDPI as a subscriber to the Sniffer
        self.snf.add_subscriber(self.ldpi)
        # Start the sniffer
        self.snf.run()
        # Decision engine loop - Keeps running until interrupted
        try:
            while True:
                time.sleep(1.0)
        except (KeyboardInterrupt, SystemExit):
            print("Shutting down...")
            self.stop()

    def stop(self):
        print("Shutting down...")
        self.stop_event.set()
        self.snf.stop()
        self.ldpi.stop()
