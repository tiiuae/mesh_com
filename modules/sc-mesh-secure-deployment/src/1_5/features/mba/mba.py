import socket
from time import sleep
from threading import Thread
import queue


class MBA:
    def __init__(self, addr):
        aux = addr.split('.')[:-1]
        aux.append('255')
        self.add = '.'.join(aux)

    def client(self, q, debug=False):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind(("0.0.0.0", 6006))
        while True:
            data, addr = sock.recvfrom(1024)
            print("mba: message received")
            if debug:
                print(data, addr)
            q.put(data)

    def server(self, message, debug=False):
        num_message = 5
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.setblocking(False)
        while num_message:
            if debug:
                print(f"this is server and it will send message {num_message} more times")
            sock.sendto(bytes(message, "utf-8"), (self.add, 6006))
            num_message -= 1
            sleep(1)

    def terminate(self):
        del self

    def test(self):
        """
        unit test should be run as
        m = MBA('127.0.0.1')
        m.test()
        """
        q = queue.Queue()
        sever_thread = Thread(target=self.server, args=("this is a test", True,), daemon=True)
        sever_thread.start()
        Thread(target=self.client, args=(q,)).start()  # client thread
        print(q.get())

