from os import sys, path

if __name__ == "__main__" and __package__ is None:
    PARENT_DIR = path.dirname(path.dirname(path.abspath(__file__)))
    SRC_PATH = path.join(PARENT_DIR, "src")
    sys.path.append(SRC_PATH)

from comms_if_monitor import CommsInterfaceMonitor
import multiprocessing
import signal


def main():
    def callback(interfaces):
        print("Callback received:", interfaces)

    print("Test starts network interface monitoring.")
    print(
        "You are expected to get a callback with initial network interface information."
    )
    print(
        "You should also get callback prints after for example network cable plugin/plugout"
    )
    try:
        input("Press Enter to continue or Ctrl+C to stop.\n")
    except KeyboardInterrupt:
        sys.exit(0)

    # Create an instance of CommsInterfaceMonitor with the callback
    monitor = CommsInterfaceMonitor(callback)
    monitor_process = multiprocessing.Process(
        target=monitor.monitor_interfaces, name="if_mon", daemon=True
    )
    monitor_process.start()

    print("Monitoring network interfaces. Press Ctrl+C to stop.\n")

    def handle_signals(signum, frame):
        if monitor_process and monitor_process.is_alive():
            monitor.stop()
            monitor_process.terminate()

    signal.signal(signal.SIGTERM, handle_signals)
    signal.signal(signal.SIGINT, handle_signals)

    monitor_process.join()
    print("Monitoring process stopped")


if __name__ == "__main__":
    main()
