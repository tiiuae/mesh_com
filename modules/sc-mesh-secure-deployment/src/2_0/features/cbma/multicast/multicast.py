import socket
import struct
import threading
import time
import argparse
import json
import sys
from queue import Queue, Empty
sys.path.insert(0, '../')
from tools.utils import get_mac_addr
from tools.custom_logger import CustomLogger

logger_instance = CustomLogger("multicast")


class MulticastHandler:
    def __init__(self, qeue, multicast_group, port, interface):
        self.queue = qeue
        self.multicast_group = multicast_group
        self.port = port
        self.interface = interface # Multicast interface. Set as wlp1s0: in case of TLS for lower macsec, bat0: in case of TLS for upper macsec
        self.logger = logger_instance.get_logger()
        self.excluded = [get_mac_addr(interface), f'{get_mac_addr(interface)}_server']

    def send_multicast_message(self, data):
        with socket.socket(socket.AF_INET6, socket.SOCK_DGRAM) as sock:
            sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_MULTICAST_HOPS, 1) #TODO: check if multi-hop nodes receive multicast over bat0
            sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_MULTICAST_IF, socket.if_nametoindex(self.interface)) # Set multicast interface

            message = {
                'mac_address': data,
                'message_type': 'mac_announcement'
            }
            self.logger.info(f'Sending data {message} to {self.multicast_group}:{self.port} from interface {self.interface}')
            sock.sendto(json.dumps(message).encode('utf-8'), (self.multicast_group, self.port))

    def receive_multicast(self):

        with socket.socket(socket.AF_INET6, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            # Bind to the wildcard address and desired port
            sock.bind(('::', self.port))

            # Set the multicast interface
            index = socket.if_nametoindex(self.interface)
            sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_MULTICAST_IF, index.to_bytes(4, byteorder='little'))

            # Construct the membership request
            mreq = socket.inet_pton(socket.AF_INET6, self.multicast_group) + index.to_bytes(4, byteorder='little')

            # Add the membership to the socket
            sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_JOIN_GROUP, mreq)
            """
        with socket.socket(socket.AF_INET6, socket.SOCK_DGRAM) as sock:
            sock.bind(('', self.port))

            group = socket.inet_pton(socket.AF_INET6, self.multicast_group)
            mreq = group + struct.pack('@I', 0)
            #mreq = group + socket.if_nametoindex(self.interface).to_bytes(4,byteorder='little')
            sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_JOIN_GROUP, mreq)
            """

            self.logger.info(f"Listening for messages on {self.multicast_group}:{self.port}...")
            while True:
                data, address = sock.recvfrom(1024)
                decoded_data = json.loads(data.decode())
                if decoded_data['mac_address'] not in self.excluded:
                    self.logger.info(f'Received data {decoded_data} from {address} at interface {self.interface}')
                    if 'mac_address' in decoded_data:
                        self.queue.put(("MULTICAST", decoded_data['mac_address']))

    def multicast_message(self):
        self.receive_multicast()


def main():
    message = get_mac_addr("wlp1s0")
    parser = argparse.ArgumentParser(description="IPv6 Multicast Sender/Receiver")
    parser.add_argument('--mode', choices=['send', 'receive', 'both'], required=True, help='Run mode: send, receive, or both')
    parser.add_argument('--address', default='ff02::1', help='Multicast IPv6 address (default: ff02::1)')
    parser.add_argument('--port', type=int, default=12345, help='Port to use (default: 12345)')
    parser.add_argument('--interface', default="wlp1s0", help='Multicast interface (default: wlp1s0)')

    args = parser.parse_args()
    queue = Queue()

    multicast_handler = MulticastHandler(queue, args.address, args.port, args.interface)

    if args.mode == 'receive':
        multicast_handler.receive_multicast()
    elif args.mode == 'send':
        multicast_handler.send_multicast_message(message)
    elif args.mode == 'both':
        receiver_thread = threading.Thread(target=multicast_handler.multicast_message)
        receiver_thread.start()

        # Wait a bit for the receiver thread to start
        time.sleep(2)

        multicast_handler.send_multicast_message(message)

        try:
            while True:
                source, data = queue.get(timeout=10)
                if source == "MULTICAST":
                    multicast_handler.logger.info(f"Main thread received MAC: {data}")

        except Empty:
            pass

        except KeyboardInterrupt:
            multicast_handler.logger.info("Shutting down...")
            sys.exit(0)


if __name__ == "__main__":
    main()
