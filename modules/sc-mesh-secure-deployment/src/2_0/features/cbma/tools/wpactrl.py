import os
import socket
import select

WPA_CTRL_MAX_REPLY_LEN = 4096

class WpaCtrl:
    counter = 0

    def __init__(self, ctrl_path):
        self.ctrl_path = ctrl_path
        self.socket = None
        self.local_path = f"/tmp/wpa_ctrl_{os.getpid()}-{WpaCtrl.counter}"
        WpaCtrl.counter += 1

    def __enter__(self):
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM, 0)
        self.socket.bind(self.local_path)
        self.socket.connect(self.ctrl_path)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        os.unlink(self.local_path)
        self.socket.close()

    def request(self, cmd, msg_cb=None, reply_len=WPA_CTRL_MAX_REPLY_LEN):
        self.socket.send(cmd)

        while True:
            rlist, _, _ = select.select([self.socket], [], [], 2)

            if not rlist or self.socket not in rlist:
                raise TimeoutError("Timed out waiting for response")

            data = self.socket.recv(reply_len)

            if not data or data[0] != b'<':
                return data

            if msg_cb:
                msg_cb(data)

    def attach(self):
        return self._attach_helper(True)

    def detach(self):
        return self._attach_helper(False)

    def _attach_helper(self, attach):
        ret = self.request(b'ATTACH' if attach else b'DETACH')
        return ret == b'OK\n' if isinstance(ret, bytes) else ret

    def recv(self, reply_len=WPA_CTRL_MAX_REPLY_LEN):
        return self.socket.recv(reply_len)

    def pending(self):
        rlist, _, _ = select.select([self.socket], [], [], 0)
        return self.socket in rlist

    def get_fd(self):
        return self.socket.fileno()

# import contextlib
# from wpacrtl import WpaCtrl
#
# with WpaCtrl("/var/run/wpa_supplicant/wlp1s0") as ctrl:
#     ctrl.attach()
#
#     try:
#         while True:
#             if ctrl.pending():
#                 response = ctrl.recv()
#                 decoded_response = response.decode().strip()
#
#                 # Check for the MESH-PEER-CONNECTED event
#                 if "MESH-PEER-CONNECTED" in decoded_response:
#                     mac_address = decoded_response.split()[-1]
#                     event = f"MESH-PEER-CONNECTED {mac_address}"
#                     print(event)
#
#                 # Check for the MESH-PEER-DISCONNECTED event
#                 if "<3>MESH-PEER-DISCONNECTED" in decoded_response:
#                     mac_address = decoded_response.split()[-1]
#                     event = f"MESH-PEER-DISCONNECTED {mac_address}"
#                     print(event)
#
#                 print("<", decoded_response)
#
#     except KeyboardInterrupt:
#         pass