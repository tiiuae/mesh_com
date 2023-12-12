import socket
import struct
import threading
import time
import argparse
import json
import sys
import os

path_to_decision_engine = os.path.dirname(__file__) # Path to dir containing this script
sys.path.insert(0, path_to_decision_engine)
sys.path.append(f'{path_to_decision_engine}/..')

from observable_module import ObservableModule
import cryptography_tools
from cbma.tools.custom_logger import CustomLogger
from cbma.tools.utils import get_mac_addr, get_mac_from_ipv6
logger = CustomLogger("mba").get_logger()


class MBA(ObservableModule):
    def __init__(self, decision_engine, multicast_group, port, interface, my_cert_dir, peer_cert_dir, stop_event=threading.Event()):
        super().__init__(decision_engine)
        self.multicast_group = multicast_group
        self.port = port
        self.interface = interface # Multicast interface. Set as radio name: in case of TLS for lower macsec, lower batman interface: in case of TLS for upper macsec
        self.stop_event = stop_event
        self.mymac = get_mac_addr(self.interface)
        self.my_cert_dir = my_cert_dir
        self.peer_cert_dir = peer_cert_dir

    def send_mba(self, mac, ip):
        with socket.socket(socket.AF_INET6, socket.SOCK_DGRAM) as sock:
            sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_MULTICAST_HOPS, 1)
            sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_MULTICAST_IF, socket.if_nametoindex(self.interface)) # Set multicast interface

            message = {
                'message_type': 'malicious_behaviour_announcement',
                'mal_mac': mac,
                'mal_ip': ip,
                'sender_mac': self.mymac
            }
            signed_message = self.sign_mba_message(json.dumps(message).encode('utf-8'))
            logger.info(f'Sending data {message} to {self.multicast_group}:{self.port} from interface {self.interface}')
            sock.sendto(signed_message, (self.multicast_group, self.port))

    def receive_mba(self):

        with socket.socket(socket.AF_INET6, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, 25, str(self.interface + '\0').encode('utf-8'))  # Bind socket to interface

            # Bind to the wildcard address and desired port
            sock.bind(('::', self.port))

            # Set the multicast interface
            index = socket.if_nametoindex(self.interface)
            sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_MULTICAST_IF, index.to_bytes(4, byteorder='little'))

            # Construct the membership request
            mreq = socket.inet_pton(socket.AF_INET6, self.multicast_group) + index.to_bytes(4, byteorder='little')

            # Add the membership to the socket
            sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_JOIN_GROUP, mreq)

            logger.info(f"Listening for messages on {self.multicast_group}:{self.port}...")
            while not self.stop_event.is_set():
                data, address = sock.recvfrom(1024)
                signature, message = cryptography_tools.extract_message_sign_from_signed_message(data)
                decoded_message = json.loads(message.decode())
                if decoded_message['message_type'] == 'malicious_behaviour_announcement' and decoded_message['sender_mac'] not in self.mymac:  # Ignore mba sent by self
                    logger.info(f'Received data {decoded_message} from {address} at interface {self.interface}')
                    if self.verify_mba_signature(signature, message, address[0]):
                        self.notify({"module": "MBA", "mac": decoded_message['mal_mac'], "ip": decoded_message['mal_ip']})

    def sign_mba_message(self, message):
        path_to_priv_key = f"{self.my_cert_dir}/macsec_{self.mymac.replace(':','')}.key" # TODO: update priv key access when HSM is in use
        signature = cryptography_tools.sign_message(message, path_to_priv_key, logger)
        return cryptography_tools.generate_signed_message(message, signature)

    def verify_mba_signature(self, signature, message, source_ip):
        source_mac = get_mac_from_ipv6(source_ip, self.interface)
        path_to_pub_key = f"{self.peer_cert_dir}/macsec_{source_mac.replace(':','')}.pem"
        return cryptography_tools.verify_signature(signature, message, path_to_pub_key, logger)


def main():
    parser = argparse.ArgumentParser(description="IPv6 Multicast Sender/Receiver")
    parser.add_argument('--address', default='ff02::1', help='Multicast IPv6 address (default: ff02::1)')
    parser.add_argument('--port', type=int, default=12345, help='Port to use (default: 12345)')
    parser.add_argument('--interface', default="wlp1s0", help='Multicast interface (default: wlp1s0)')

    args = parser.parse_args()
    decision_engine = "dummy"
    mba = MBA(decision_engine, args.address, args.port, args.interface)
    mac = 'test mac'
    ip = 'test ip'

    receiver_thread = threading.Thread(target=mba.receive_mba)
    receiver_thread.start()

    # Wait a bit for the receiver thread to start
    time.sleep(2)

    mba.send_mba(mac, ip)

if __name__ == "__main__":
    main()
