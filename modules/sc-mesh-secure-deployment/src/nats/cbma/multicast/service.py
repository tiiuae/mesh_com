import socket
import threading
import time
import random

from typing import Callable, Tuple
from struct import pack

from utils.networking import LLA_PREFIX, get_interface_link_local_ipv6_address
from utils.logging import get_logger


logger = get_logger()

# TODO - Move to models
SecureConnectionCallbackType = Callable[[str], bool]


class MulticastService(object):
    # Maximum receive buffer size for network exchange
    BUFFER_SIZE: int = 1024
    MULTICAST_SEND_DELAY: float = 1.0
    SOCKET_RECV_TIMEOUT: float = 2.0
    THREAD_START_TIMEOUT: float = 0.2
    THREAD_STOP_TIMEOUT: float = 5.0
    MAX_RETRIES: int = 5
    UNBLOCK_SECONDS: float = 10.0
    MIN_BLOCK_TIME_SECONDS: float = 0.2
    MAX_BLOCK_TIME_SECONDS: float = 1.0
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
        # Stores the IPv6 of blocked peers with the epoch of their unblocking time:
        self.blocked_peers: dict[str, float] = {}
        self.unblock_epoch: float = 0

        self.exit_event = threading.Event()
        self.sender_event = threading.Event()
        self.sender_thread = threading.Thread(target=self.send_multicast_loop, daemon=True)


    def send_multicast_loop(self) -> bool:
        success = False
        with socket.socket(socket.AF_INET6, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
            index: int = socket.if_nametoindex(self.interface)

            sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_MULTICAST_HOPS, 1)
            sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_MULTICAST_IF, index)

            # Make sure we are using the LLA of the iface as origin address
            sock.bind((self.ipv6, 0, 0, index))

            sock.connect((self.multicast_group, self.destination_port))
            sock.setblocking(False)

            logger.debug('Running the multicast-sender loop')
            retries = 0
            while not self.exit_event.is_set():
                try:
                    bytes_sent: int = sock.send(b'')
                    if bytes_sent == -1:
                        logger.error('Failed to send packet')
                    else:
                        retries = 0
                except Exception as e:
                    retries += 1
                    logger.error(f"Attempt {retries}/{self.MAX_RETRIES} - Exception when sending multicast packet: {e}")
                    if retries == self.MAX_RETRIES:
                        logger.error('Maximum attempts for sending multicast packets reached')
                        self.exit_event.set()
                        success = False
                        break
                # NOTE This is a PyOpenSSL workaround for it to work with TLS 1.3
                #      For some reason, sleeping before calling wait is breaking TLS
                #      So we wait first and sleep the remaining seconds if needed
                epoch_before = time.time()
                self.sender_event.wait()
                remaining_seconds = max(self.MULTICAST_SEND_DELAY - (time.time() - epoch_before), 0.0)
                time.sleep(remaining_seconds)

        logger.debug('Exiting multicast-sender loop')
        return success


    def receive_multicast_loop(self) -> bool:
        success = True
        with socket.socket(socket.AF_INET6, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)

            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, str(self.interface + '\0').encode('utf-8'))

            index: int = socket.if_nametoindex(self.interface)
            index_bytes: bytes = pack('=I', index)
            # Set the multicast interface
            sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_MULTICAST_IF, index_bytes)

            # Construct the membership request
            mreq: bytes = socket.inet_pton(socket.AF_INET6, self.multicast_group) + index_bytes

            # Add the membership to the socket
            sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_JOIN_GROUP, mreq)

            # Set the socker read/recv timeout
            sock.settimeout(self.SOCKET_RECV_TIMEOUT)

            # Multicasts aren't received if not binding to the wildcard
            sock.bind(('::', self.destination_port, 0, index))

            logger.debug('Running the multicast-receiver loop')
            self.unblock_epoch = self.UNBLOCK_SECONDS + time.time()
            retries = 1
            while not self.exit_event.is_set():
                self.sender_event.set()
                # Free memory of blocked peers that never connect again
                if (epoch_now := time.time()) > self.unblock_epoch:
                    self.unblock_peers()
                    self.unblock_epoch = self.UNBLOCK_SECONDS + epoch_now
                try:
                    _, address = sock.recvfrom(self.BUFFER_SIZE)
                    self.sender_event.clear()
                    if self.process_message(address):
                        retries = 1
                        continue
                except socket.timeout:
                    continue
                except Exception as e:
                    logger.error(f"Attempt {retries}/{self.MAX_RETRIES} - Exception when receiving multicast packets: {e}")
                if retries == self.MAX_RETRIES:
                    logger.error('Maximum attempts for receiving multicast packets reached')
                    self.exit_event.set()
                    self.sender_event.set()
                    success = False
                else:
                    retries += 1

        logger.debug('Exiting multicast-receiver loop')
        return success


    def process_message(self, address: Tuple) -> bool:
        try:
            peer_ipv6: str = address[0]
            if peer_ipv6.startswith(LLA_PREFIX) and \
               peer_ipv6 != self.ipv6 and \
               peer_ipv6 not in self.authenticated_peers:
                self.authenticate_peer(peer_ipv6)
        except Exception as e:
            logger.error(f"Failed to process received message: {e}")
            return False
        return True


    def authenticate_peer(self, peer_ipv6: str) -> None:
        block_epoch: float = self.blocked_peers.get(peer_ipv6, 0.0)
        if (epoch_now := time.time()) < block_epoch:
            logger.debug(f"{peer_ipv6} is blocked for {block_epoch - epoch_now:.2f} more seconds")
            return
        self.blocked_peers.pop(peer_ipv6, None)

        logger.debug(f"Authenticating {peer_ipv6}")
        if self.secure_connection_callback(peer_ipv6):
            logger.debug(f"{peer_ipv6} authenticated successfully")

            self.authenticated_peers.append(peer_ipv6)
        else:
            logger.error(f"Failed to authenticate {peer_ipv6}")

            # Increases blocking time in subsequent failures
            block_seconds = max(block_epoch - (self.unblock_epoch - self.UNBLOCK_SECONDS), 0.0)
            block_seconds += random.uniform(self.MIN_BLOCK_TIME_SECONDS, self.MAX_BLOCK_TIME_SECONDS)

            logger.warning(f"Blocking {peer_ipv6} for {block_seconds:.2f} seconds")
            self.blocked_peers[peer_ipv6] = block_seconds + time.time()


    def deauthenticate_peer(self, peer_ipv6: str) -> bool:
        self.blocked_peers.pop(peer_ipv6, None)
        try:
            self.authenticated_peers.remove(peer_ipv6)
            logger.debug(f"{peer_ipv6} deauthenticated")
            return True
        except ValueError:
            logger.error(f"{peer_ipv6} not authenticated")
            return False


    def unblock_peers(self) -> None:
        epoch_now = time.time()
        for peer_ipv6, block_epoch in list(self.blocked_peers.items()):
            if epoch_now < block_epoch:
                break
            del self.blocked_peers[peer_ipv6]


    def start(self) -> bool:
        self.exit_event.clear()
        self.sender_event.set()
        self.sender_thread.start()

        self.sender_thread.join(self.THREAD_START_TIMEOUT)
        if not self.sender_thread.is_alive():
            return False

        logger.info('Starting multicast service')
        return self.receive_multicast_loop()


    def stop(self) -> bool:
        logger.info('Stopping service ...')
        self.exit_event.set()
        self.sender_event.set()
        self.sender_thread.join(self.THREAD_STOP_TIMEOUT)

        self.authenticated_peers.clear()
        self.blocked_peers.clear()

        if self.sender_thread.is_alive():
            logger.warning("Sender didn't stop gracefully")
            # TODO - Kill the thread
            return False
        return True

