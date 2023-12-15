import socket
import threading
from abc import ABC, abstractmethod
from enum import Enum
from typing import Tuple, Union, Optional, NoReturn

import dpkt

# Define a type for the flow key
FlowKeyType = Tuple[bytes, int, bytes, int]  # (src_ip, src_port, dst_ip, dst_port)


class ModuleInterface(ABC, threading.Thread):
    """An abstract base class for feature modules in a network system.

    This class provides basic threading and event handling mechanisms for derived modules.

    Attributes:
        stop_event (threading.Event): An event to signal the stopping of the module's thread.
        thread (Optional[threading.Thread]): A thread in which the module's main functionality runs.
    """

    def __init__(self):
        super(ModuleInterface, self).__init__()
        self.stop_event = threading.Event()
        self.thread: Optional[threading.Thread] = None

    @abstractmethod
    def run(self) -> NoReturn:
        """Starts the module's functionality in a separate thread.

        This method should be overridden to define the module's main behavior.
        """
        self.thread.start()

    def terminate(self) -> NoReturn:
        """Terminates the module.

        This method handles the cleanup and resource deallocation for the module.
        """
        del self

    def stop(self) -> NoReturn:
        """Stops the module's thread.

        Signals the thread to stop and waits for it to join.
        """
        self.stop_event.set()
        self.thread.join()

    def stopped(self) -> bool:
        """Checks if the module's thread has been stopped.

        Returns:
            bool: True if the thread has been signaled to stop, False otherwise.
        """
        return self.stop_event.is_set()


class SnifferSubscriber(ModuleInterface):
    """An abstract base class for modules subscribing to a network sniffer module.

    This class defines the interface for modules that react to new network packets and teardown events.

    Methods to be implemented by subclasses:
        new_packet: Handle a new packet event.
        teardown: Handle a flow teardown event.
    """

    def __init__(self):
        super(SnifferSubscriber, self).__init__()

    @abstractmethod
    def new_packet(self, flow_key: FlowKeyType, protocol: int, timestamp: int, ip: dpkt.ip.IP) -> NoReturn:
        """Handles a new packet event from the sniffer.

        This method should be overridden to define behavior upon receiving a new packet.

        Args:
            flow_key (FlowKeyType): The key identifying the flow of the packet.
            protocol (int): The protocol number of the packet.
            timestamp (int): The timestamp when the packet was captured.
            ip (dpkt.ip.IP): The IP packet instance.
        """
        print('new packet')

    @abstractmethod
    def teardown(self, flow_key: FlowKeyType, protocol: int) -> NoReturn:
        """Handles a flow teardown event from the sniffer.

        This method should be overridden to define behavior upon a flow timeout.

        Args:
            flow_key (FlowKeyType): The key identifying the flow.
            protocol (int): The protocol number of the flow.
        """


def get_flow_key(src_ip: bytes, src_port: int, dst_ip: bytes, dst_port: int, protocol: int, session: bool) -> tuple:
    """
    Generate a flow key based on source and destination IP addresses, ports, and protocol.

    If session is True, the flow key is bidirectional, sorting the endpoints to ensure a consistent
    order regardless of which direction the flow is observed. If session is False, the flow key is unidirectional,
    using the source and destination as they are provided.

    Args:
        src_ip (bytes): The source IP address.
        src_port (int): The source port number.
        dst_ip (bytes): The destination IP address.
        dst_port (int): The destination port number.
        protocol (int): The protocol number.
        session (bool): Indicates if the flow is bidirectional (True) or unidirectional (False).

    Returns:
        tuple: A tuple representing the flow key.

    Example:
        >>> get_flow_key(b'192.168.1.1', 80, b'192.168.1.2', 443, 6, True)
        (b'192.168.1.1', 80, b'192.168.1.2', 443, 6)
    """

    # Handling bidirectional flow
    if session:
        # Sorting endpoints to ensure consistent ordering for bidirectional flow
        sorted_endpoints = sorted([(src_ip, src_port), (dst_ip, dst_port)])

        # Unpacking sorted endpoints to create a flow key
        ip1, port1 = sorted_endpoints[0]
        ip2, port2 = sorted_endpoints[1]

        flow_key = (ip1, port1, ip2, port2, protocol)
    else:
        # Handling unidirectional flow
        flow_key = (src_ip, src_port, dst_ip, dst_port, protocol)

    return flow_key


def inet_to_str(inet: Union[bytes, bytearray]) -> str:
    """
    Converts an IP address from binary format to its string representation.

    This function first attempts to convert the address as IPv4, and if that fails,
    it tries IPv6.

    Args:
        inet (Union[bytes, bytearray]): The binary representation of the IP address.

    Returns:
        str: The string representation of the IP address.

    Example:
        >>> inet_to_str(b'\x7f\x00\x00\x01')
        '127.0.0.1'
    """
    try:
        # Convert IPv4 address from binary to string
        return socket.inet_ntop(socket.AF_INET, inet)
    except ValueError:
        # If IPv4 conversion fails, try converting IPv6 address
        return socket.inet_ntop(socket.AF_INET6, inet)


def mac_addr(address: bytes) -> str:
    """
    Converts a MAC address from binary format to its string representation.

    Each byte of the address is converted to a two-digit hexadecimal string. These
    strings are then joined by colons.

    Args:
        address (bytes): The binary representation of the MAC address.

    Returns:
        str: The string representation of the MAC address in the format 'XX:XX:XX:XX:XX:XX'.

    Example:
        >>> mac_addr(b'\x00\x1A\x2B\x3C\x4D\x5E')
        '00:1a:2b:3c:4d:5e'
    """
    return ':'.join('%02x' % b for b in address)


def flow_key_to_str(flow_key: tuple) -> tuple:
    """
    Convert a flow key to a string representation.

    Args:
        flow_key (tuple): The flow key consisting of IP addresses and port numbers.

    Returns:
        tuple: The flow key with IP addresses in string format and port numbers.
    """
    # Converting IP addresses to string format and keeping other elements unchanged
    value = (inet_to_str(flow_key[0]), flow_key[1], inet_to_str(flow_key[2]), flow_key[3], flow_key[4])
    return value


def sec_to_ns(seconds: float) -> int:
    """
    Convert seconds to nanoseconds.

    Args:
        seconds (float): The time in seconds.

    Returns:
        int: The time in nanoseconds.
    """
    # Multiplying seconds by 1e+9 to convert to nanoseconds
    return int(seconds * 1e+9)


def ns_to_sec(nanoseconds: int) -> float:
    """
    Convert nanoseconds to seconds.

    Args:
        nanoseconds (int): The time in nanoseconds.

    Returns:
        float: The time in seconds.
    """
    # Dividing nanoseconds by 1e+9 to convert to seconds
    return nanoseconds / 1e+9


class Dataset(ABC):
    def __init__(self):
        self.path = ''

    @abstractmethod
    def get_files(self):
        pass


class Color(str, Enum):
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
