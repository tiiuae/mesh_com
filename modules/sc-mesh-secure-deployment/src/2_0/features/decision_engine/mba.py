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
    """
    A class representing a Malicious Behavior Announcement (MBA) system,
    responsible for sending and receiving multicast messages about malicious behavior.

    Inherits from ObservableModule to utilize its notification system.

    Attributes:
    multicast_group (str): The IPv6 address of the multicast group.
    port (int): The port number used for multicast communication.
    interface (str): The network interface used for sending and receiving multicast MBA messages.
    stop_event (threading.Event): An event to stop the receiver loop.
    mymac (str): MAC address of the interface used for MBA multicast in the current machine.
    my_cert_dir (str): Directory containing the machine's private key.
    peer_cert_dir (str): Directory containing the public keys of peers.

    Methods:
    send_mba(mac, ip): Sends a malicious behavior announcement to the multicast group.
    receive_mba(): Continuously listens for and processes incoming MBA messages.
    sign_mba_message(message): Signs an MBA message with the machine's private key.
    verify_mba_signature(signature, message, source_ip): Verifies the signature of a received MBA message.
    """
    def __init__(self, decision_engine, multicast_group, port, interface, my_cert_dir, peer_cert_dir, stop_event=threading.Event()):
        """
        Initializes the MBA instance with multicast group details, network interface, certificate directories,
        and a stop event for controlling the receiver loop.

        Parameters:
        decision_engine: An instance of a decision engine for processing received messages.
        multicast_group (str): The IPv6 address of the multicast group.
        port (int): The port number used for multicast communication.
        interface (str): The network interface used for sending and receiving multicast messages.
        my_cert_dir (str): Directory containing the machine's private key.
        peer_cert_dir (str): Directory containing the public keys of peers.
        stop_event (threading.Event, optional): An event to stop the receiver loop. Defaults to threading.Event().
        """

        super().__init__(decision_engine)
        self.multicast_group = multicast_group
        self.port = port
        self.interface = interface # Multicast interface. Set as radio name: in case of TLS for lower macsec, lower batman interface: in case of TLS for upper macsec
        self.stop_event = stop_event
        self.mymac = get_mac_addr(self.interface)
        self.my_cert_dir = my_cert_dir
        self.peer_cert_dir = peer_cert_dir

    def send_mba(self, mac, ip):
        """
        Sends a malicious behavior announcement to the multicast group.

        Parameters:
        mac (str): MAC address of the malicious node.
        ip (str): IP address of the malicious node.
        """
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
            if signed_message:
                logger.info(f'Sending data {message} to {self.multicast_group}:{self.port} from interface {self.interface}')
                sock.sendto(signed_message, (self.multicast_group, self.port))
            else:
                logger.error('Could not sign MBA message')

    def receive_mba(self):
        """
        Listens for incoming MBA messages and processes them.
        Runs continuously until the stop_event is set.
        Validates the message signature and notifies the decision engine if valid.
        """

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

            sock.settimeout(2)

            logger.info(f"Listening for messages on {self.multicast_group}:{self.port}...")
            while not self.stop_event.is_set():
                try:
                    data, address = sock.recvfrom(1024)
                    try:
                        signature, message = cryptography_tools.extract_message_sign_from_signed_message(data)
                        decoded_message = json.loads(message.decode())
                        if decoded_message['message_type'] == 'malicious_behaviour_announcement' and decoded_message['sender_mac'] not in self.mymac:  # Ignore mba sent by self
                            logger.info(f'Received data {decoded_message} from {address} at interface {self.interface}')
                            if self.verify_mba_signature(signature, message, address[0]):
                                self.notify({"module": "MBA", "mac": decoded_message['mal_mac'], "ip": decoded_message['mal_ip']})
                    except Exception as e:
                        logger.error(f"Error processing received message: {e}")
                except socket.timeout:
                    # No data received within the timeout period
                    continue  # Go back to the start of the loop
                except Exception as e:
                    logger.error(f"Error processing received message: {e}")

    def sign_mba_message(self, message):
        """
        Signs an MBA message using the private key of the machine.

        Parameters:
        message (bytes): The MBA message to be signed.

        Returns:
        bytes: The message, combined with its signature.
        """
        path_to_priv_key = f"{self.my_cert_dir}/macsec_{self.mymac.replace(':','')}.key" # TODO: update priv key access when HSM is in use
        if signature := cryptography_tools.sign_message(message, path_to_priv_key, logger):
            return cryptography_tools.generate_signed_message(message, signature)
        else:
            return None

    def verify_mba_signature(self, signature, message, source_ip):
        """
        Verifies the signature of an MBA message using the sender's public key.

        Parameters:
        signature (bytes): The signature to be verified.
        message (bytes): The MBA message that was signed.
        source_ip (str): The source IP address of the message, used to determine the sender's MAC address.

        Returns:
        bool: True if the signature is valid, False otherwise.
        """
        source_mac = get_mac_from_ipv6(source_ip, self.interface)
        path_to_pub_key = f"{self.peer_cert_dir}/macsec_{source_mac.replace(':','')}.pem"
        return cryptography_tools.verify_signature(signature, message, path_to_pub_key, logger)
