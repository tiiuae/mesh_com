import socket
import ctypes
import struct

class dri_broadcast():
    def __init__(self, server, port):
        super().__init__('dri_broadcast')

        self.HOST = server  # Standard loopback interface address (localhost)
        self.PORT = port  # Port to listen on (non-privileged ports are > 1023)
        self.dri_astm_msg_socket = socket.socket()  # to remove python warning
        self.setup_socket()

    def setup_socket(self):
        try:
            self.get_logger().info('dri socket init')
            self.dri_astm_msg_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.dri_astm_msg_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.dri_astm_msg_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.dri_astm_msg_socket.settimeout(1)
            self.dri_astm_msg_socket.connect((self.HOST, self.PORT))
        except socket.error as message:
            print(f"Type Error: {message}")

    def send(self, msg):
        try:
            self.dri_astm_msg_socket.sendall(msg)
        except (ConnectionRefusedError, socket.timeout, ConnectionResetError, ConnectionRefusedError):
            self.setup_socket()
