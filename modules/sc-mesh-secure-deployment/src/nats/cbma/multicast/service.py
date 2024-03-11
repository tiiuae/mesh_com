import socket
import threading
import time

from typing import Callable, Tuple

from utils.networking import LLA_PREFIX, get_interface_link_local_ipv6_address
from utils.logging import get_logger


logger = get_logger()

# TODO - Move to models
SecureConnectionCallbackType = Callable[[str], bool]


class MulticastService(object):
    # Maximum receive buffer size for network exchange
    BUFFER_SIZE: int = 1024
    MULTICAST_SEND_DELAY: float = 0.1
    SOCKET_RECV_TIMEOUT: float = 2.0
    THREAD_JOIN_TIMEOUT: float = 5.0
    MAX_RETRIES: int = 5
    GROUP_PREFIX: str = 'ff33'

    def __init__(self,
                 interface: str,
                 port: int,
                 secure_connection_callback: SecureConnectionCallbackType,
                 group_postfix: str = ':1'
                 ) -> None:
        self.interface: str = interface
        self.secure_connection_callback: SecureConnectionCallbackType = secure_connection_callback
        self.ipv6: str = get_interface_link_local_ipv6_address(self.interface)

        self.source_port: int = port
        self.destination_port: int = self.source_port + 1
        self.multicast_group: str = f'{self.GROUP_PREFIX}:{group_postfix}'

        # Stores the IPv6 of authenticated peers:
        self.authenticated_peers: list[str] = []

        self.event = threading.Event()
        self.sender_thread = threading.Thread(target=self.send_multicast_loop, daemon=True)


    def send_multicast_loop(self) -> None:
        with socket.socket(socket.AF_INET6, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
            index: int = socket.if_nametoindex(self.interface)

            sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_MULTICAST_HOPS, 1)
            sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_MULTICAST_IF, index)

            # Make sure we are using the LLA of the iface as origin address
            sock.bind((self.ipv6, 0, 0, index))

            sock.connect((self.multicast_group, self.destination_port))
            sock.setblocking(False)

            logger.debug(f'Running the multicast sender loop')
            retries = 0
            while not self.event.is_set():
                try:
                    bytes_sent: int = sock.send(b'')
                    if bytes_sent == -1:
                        logger.error("Failed to send packet")
                    else:
                        retries = 0
                except Exception as e:
                    retries += 1
                    logger.error(f"Attempt {retries}/{self.MAX_RETRIES} - Exception when sending multicast packet: {e}")
                    if retries == self.MAX_RETRIES:
                        logger.error("Maximum attempts for sending multicast packets reached")
                        self.event.set()
                        break
                time.sleep(self.MULTICAST_SEND_DELAY)

            logger.debug(f'Multicast-send loop is completed')


    def receive_multicast_loop(self) -> bool:
        success = False
        with socket.socket(socket.AF_INET6, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, str(self.interface + '\0').encode('utf-8'))

            index: int = socket.if_nametoindex(self.interface)
            index_bytes: bytes = index.to_bytes(4, byteorder='little')
            # Set the multicast interface
            sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_MULTICAST_IF, index_bytes)

            # Construct the membership request
            mreq: bytes = socket.inet_pton(socket.AF_INET6, self.multicast_group) + index_bytes

            # Add the membership to the socket
            sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_JOIN_GROUP, mreq)

            # Set the socker read/recv timeout
            sock.settimeout(self.SOCKET_RECV_TIMEOUT)

            # Bind to the wildcard address and desired port
            sock.bind(('::', self.destination_port, 0, index))
            # TODO - mcasts aren't received if not binding to the wildcard
            # sock.bind((self.ipv6, self.destination_port, 0, index))

            logger.debug(f'Running the multicast receive loop')
            retries = 0
            while not self.event.is_set():
                try:
                    _, address = sock.recvfrom(self.BUFFER_SIZE)
                    self.process_message(address)
                    retries = 0
                except socket.timeout:
                    continue
                except Exception as e:
                    retries += 1
                    logger.error(f"Attempt {retries}/{self.MAX_RETRIES} - Exception when receiving multicast packets: {e}")
                    if retries == self.MAX_RETRIES:
                        logger.error("Maximum attempts for receiving multicast packets reached")
                        self.event.set()

            logger.debug('Exiting multicast-receive loop')
            success = True
        return success


    def process_message(self, address: Tuple) -> bool:
        try:
            peer_ipv6: str = address[0]
            if peer_ipv6.startswith(LLA_PREFIX) and self.ipv6 != peer_ipv6:
                logger.debug(f'Received announcement from {peer_ipv6}')
                self.authenticate_peer(peer_ipv6)

        except Exception as exc:
            logger.error(f"Failed to process received message: {exc}")
            return False

        return True


    def authenticate_peer(self, peer_ipv6: str) -> None:
        if peer_ipv6 not in self.authenticated_peers:
            logger.debug(f"Authenticating {peer_ipv6}")
            if self.secure_connection_callback(peer_ipv6):
                logger.debug(f"{peer_ipv6} authenticated successfully")
                self.authenticated_peers.append(peer_ipv6)
            else:
                logger.error(f"Failed to authenticate {peer_ipv6}")


    def deauthenticate_peer(self, peer_ipv6: str) -> bool:
        try:
            logger.debug(f"{peer_ipv6} deauthenticated")
            self.authenticated_peers.remove(peer_ipv6)
            return True
        except ValueError:
            logger.error(f"Peer {peer_ipv6} not authenticated")
            return False


    def start(self) -> bool:
        self.event.clear()
        self.sender_thread.start()

        self.sender_thread.join(0.2)
        if not self.sender_thread.is_alive():
            return False

        logger.info(f"Starting multicast service")

        return self.receive_multicast_loop()


    def stop(self) -> bool:
        logger.info("Stopping service ...")
        self.event.set()
        self.sender_thread.join(self.THREAD_JOIN_TIMEOUT)

        self.authenticated_peers.clear()
        if self.sender_thread.is_alive():
            logger.warning("Sender didn't stop gracefully")
            # TODO - Kill the thread
            return False
        return True

