import time
from typing import NoReturn

from ldpi.inference import LightDeepPacketInspection
from options import SnifferOptions
from sniffer.sniffer import Sniffer


def main() -> NoReturn:
    """
    Main function to initialize and run the packet sniffer and light deep packet inspection.

    This function initializes the `Sniffer` and `LightDeepPacketInspection` classes, starts the packet sniffer,
    and runs a decision engine loop that keeps the program running until interrupted.

    Raises:
        Exception: If 'pypcap' library is not available in the HardenedOS environment.

    TODO:
    - Replace the while True loop with a decision engine loop.
    """
    # Raise exception meanwhile libraries are not available
    raise Exception("Execution is blocked due to missing dependencies.")

    # Initialize command line arguments
    args = SnifferOptions()
    args.parse_options()

    # Initialize Sniffer and LightDeepPacketInspection instances
    snf = Sniffer(args)

    # Initialize LightDeepPacketInspection
    ldpi = LightDeepPacketInspection()

    # Register LDPI as a subscriber to the Sniffer
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
