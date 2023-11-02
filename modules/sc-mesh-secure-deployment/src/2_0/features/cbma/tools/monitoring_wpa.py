
#from socket.python3.tools.wpactrl import WpaCtrl
import os
import time
import contextlib
from tools.wpactrl import WpaCtrl
from .custom_logger import CustomLogger

class WPAMonitor:
    def __init__(self, ctrl_path):
        self.ctrl_path = ctrl_path
        self.logger = self._setup_logger()

    def _setup_logger(self):
        logger_instance = CustomLogger("wpa_monitor")
        return logger_instance.get_logger()

    def _wait_for_ctrl_interface(self):
        waiting_message_printed = False
        while not os.path.exists(self.ctrl_path):
            if not waiting_message_printed:
                self.logger.info("Waiting for the wpa_supplicant control interface file to be created...")
                waiting_message_printed = True
            time.sleep(1)

        return WpaCtrl(self.ctrl_path)

    def start_monitoring(self, queue):
        instance = self._wait_for_ctrl_interface()
        with instance as ctrl:
            ctrl.attach()
            while True:
                if ctrl.pending():
                    response = ctrl.recv()
                    decoded_response = response.decode().strip()
                    self._handle_event(decoded_response, queue)

    def _handle_event(self, decoded_response, queue):
        # Check for the MESH-PEER-CONNECTED event
        if "MESH-PEER-CONNECTED" in decoded_response:
            mac_address = decoded_response.split()[-1]
            event = f"MESH-PEER-CONNECTED {mac_address}"
            self.logger.info(event)
            queue.put(("WPA", mac_address))

        # Check for the MESH-PEER-DISCONNECTED event
        elif "MESH-PEER-DISCONNECTED" in decoded_response:
            mac_address = decoded_response.split()[-1]
            event = f"MESH-PEER-DISCONNECTED {mac_address}"
            self.logger.info(event)

        # Uncomment the next line if you want to log other events or for debugging purposes
        # self.logger.debug(f"< {decoded_response}")

# Usage
# queue = some_queue_structure
# monitor = WPAMonitor("/path/to/ctrl")
# monitor.start_monitoring(queue)


#
#
# def create_wpa_ctrl_instance(ctrl_path):
#     waiting_message_printed = False
#
#     while not os.path.exists(ctrl_path):
#         if not waiting_message_printed:
#             logger.info("Waiting for the wpa_supplicant control interface file to be created...")
#             waiting_message_printed = True
#
#         time.sleep(1)
#
#     return WpaCtrl(ctrl_path)
#
# def process_events(ctrl, queue):
#      with contextlib.suppress(KeyboardInterrupt):
#         while True:
#             if ctrl.pending():
#                 response = ctrl.recv()
#                 decoded_response = response.decode().strip()
#
#                 # Check for the MESH-PEER-CONNECTED event
#                 if "MESH-PEER-CONNECTED" in decoded_response:
#                     mac_address = decoded_response.split()[-1]
#                     event = f"MESH-PEER-CONNECTED {mac_address}"
#                     logger.info(event)
#                     queue.put(mac_address)
#
#                 # Check for the MESH-PEER-DISCONNECTED event
#                 if "MESH-PEER-DISCONNECTED" in decoded_response:
#                     mac_address = decoded_response.split()[-1]
#                     event = f"MESH-PEER-DISCONNECTED {mac_address}"
#                     logger.info(event)
#
#                 #print("<", decoded_response)