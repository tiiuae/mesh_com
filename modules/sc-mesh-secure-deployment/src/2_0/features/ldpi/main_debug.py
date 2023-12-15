import time
from typing import NoReturn

from ldpi import LightDeepPacketInspection
from options import SnifferOptions
from sniffer import SnifferPcap


def main() -> NoReturn:
    """
    Main debugging function for packet sniffing from a .pcap file.

    This function initializes the `SnifferPcap` class with command line arguments, sets the path to the .pcap file,
    initializes `LightDeepPacketInspection`, and starts the packet sniffing from the .pcap file. It also contains
    a loop that keeps the program running until interrupted.

    TODO:
    - Replace the while True loop with a decision engine loop.
    """
    # Initialize command line arguments
    args = SnifferOptions()
    args.parse_options()

    # Initialize SnifferPcap with the provided arguments
    snf = SnifferPcap(args)

    # Set the path to the .pcap file
    snf.set_pcap_path('datasets/TII-SSRC-23/pcap/benign/video/http.pcap')

    # Initialize LightDeepPacketInspection
    ldpi = LightDeepPacketInspection()

    # Register LDPI as a subscriber to the SnifferPcap
    snf.add_subscriber(ldpi)

    # Start the sniffer
    snf.run()

    # Decision engine loop (placeholder) - Keeps running until interrupted
    try:
        while True:
            time.sleep(1.0)
    except (KeyboardInterrupt, SystemExit):
        print("Shutting down...")
        snf.stop()
        ldpi.stop()


if __name__ == '__main__':
    main()
