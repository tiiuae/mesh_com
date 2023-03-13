import socket
from time import sleep
from threading import Thread
import queue

from common import utils

import os
import sys


class MBA:
    def __init__(self, addr):
        aux = addr.split('.')[:-1]
        aux.append('255')
        self.add = '.'.join(aux)

    def client(self, q, debug=False, logger=None):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                sock.bind(("0.0.0.0", 6006))
                if logger:
                    logger.info("Client socket created")
        except socket.error:
            print("Client socket creation failed")
            if logger:
                logger.error("Client socket creation failed")
            return
        while True:
            data, addr = sock.recvfrom(1024)
            print("mba: message received")
            if debug:
                print(data, addr)
            q.put(data)

    def server(self, message, debug=False, logger=None):
        num_message = 5
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock: # UDP
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                sock.setblocking(False)
                if logger:
                    logger.info("Server socket created")
        except socket.error:
            print("Server socket creation failed")
            if logger:
                logger.error("Server socket creation failed")
            return
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
        m = mba.MBA('127.0.0.1')
        m.test()
        stores log to logs/mba-log.txt
        """
        common_ut = utils.Utils()
        logger = common_ut.setup_logger('mba')

        q = queue.Queue()
        sever_thread = Thread(target=self.server, args=("this is a test", True, logger), daemon=True)
        sever_thread.start()
        Thread(target=self.client, args=(q, False, logger)).start()  # client thread
        print(q.get())
        logger.info("Received message: %s", q.get())

        sever_thread.join()
        common_ut.close_logger(logger)

