import socket
from time import sleep


class MBA:
    def __init__(self, addr):
        aux = addr.split('.')[:-1]
        aux.append('255')
        self.add = '.'.join(aux)

    def client(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind(("0.0.0.0", 5005))
        while True:
            data, addr = sock.recvfrom(1024)
            print("mba: message received")
            return data, addr

    def server(self, message):
        num_message = 200
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        for _ in range(num_message):
            sock.sendto(bytes(message, "utf-8"), (self.add, 5005))
            sleep(1)
