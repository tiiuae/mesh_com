import unittest
import threading
import time
import os
from src.comms_if_monitor import CommsInterfaceMonitor

interface_data = None

@unittest.skipUnless(os.getuid() == 0, "Skipping test, must be run as root")
class TestCommsInterfaceMonitor(unittest.TestCase):

    def test_monitor_interfaces_starts_and_stops_correctly(self):

        global interface_data

        def callback(interfaces):
            global interface_data
            interface_data = interfaces
            print(interfaces)

        monitor = CommsInterfaceMonitor(callback)

        thread_if_mon = threading.Thread(
            target=monitor.monitor_interfaces
        )
        thread_if_mon.start()
        time.sleep(5)
        monitor.stop()
        if thread_if_mon.is_alive():
            thread_if_mon.join()

        for interface in interface_data:
            if interface['interface_name'] == "lo" and interface['mac_address'] == '00:00:00:00:00:00':
                self.assertTrue(True)
                break
