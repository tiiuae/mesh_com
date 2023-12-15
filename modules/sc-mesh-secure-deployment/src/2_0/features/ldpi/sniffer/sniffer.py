import logging
import threading
import time
from typing import List, Optional, Tuple, Dict, NoReturn

import dpkt
import netifaces as ni
import pcap
from tqdm import tqdm

from options import SnifferOptions
from utils import ModuleInterface, SnifferSubscriber, FlowKeyType, Color, get_flow_key, sec_to_ns

# pcap library: comes from pypcap https://github.com/pynetwork/pypcap (requires sudo apt-get install libpcap-dev)

protocol_classes = {
    dpkt.ip.IP_PROTO_TCP: dpkt.tcp.TCP,
    dpkt.ip.IP_PROTO_UDP: dpkt.udp.UDP,
    dpkt.ip.IP_PROTO_ICMP: dpkt.icmp.ICMP,
    dpkt.ip.IP_PROTO_IGMP: dpkt.igmp.IGMP,
}


class Sniffer(ModuleInterface):
    """A network sniffer module for capturing and processing network packets.

    Attributes:
        args (SnifferOptions): Configuration options for the sniffer.
        timeout_ns (int): Timeout for flows in nanoseconds.
        flows_tcp (Dict[FlowKeyType, int]): Dictionary for tracking TCP flows.
        flows_udp (Dict[FlowKeyType, int]): Dictionary for tracking UDP flows.
        local_ip (Optional[str]): Local IP address of the interface.
        subscribers (List[SnifferSubscriber]): List of subscribers for packet events.
        fragments_cache (dict): Cache for handling IP fragments.
        thread (threading.Thread): Thread for running the sniffing process.

    Args:
        args (SnifferOptions): Configuration options passed during initialization.
    """

    def __init__(self, args: SnifferOptions):

        super(Sniffer, self).__init__()
        self.args: SnifferOptions = args
        self.timeout_ns: int = sec_to_ns(self.args.timeout)
        self.flows_tcp: Dict[FlowKeyType, int] = {}
        self.flows_udp: Dict[FlowKeyType, int] = {}
        self.local_ip: Optional[str] = None
        self.subscribers: List[SnifferSubscriber] = []
        self.fragments_cache = {}

        # Set up threading to run sniff() in a separate thread
        self.thread = threading.Thread(target=self.sniff)

    def run(self, daemon: bool = True) -> NoReturn:
        """Starts the sniffer in a separate thread and starts the subscribers.

        Args:
            daemon (bool): Whether to run the thread as a daemon. Defaults to True.
        """
        self.thread.daemon = daemon
        self.thread.start()
        self.start_subscribers()

    # @abstractmethod
    def sniff(self) -> NoReturn:
        """Core sniffing method that captures packets and processes them."""
        self.local_ip = ni.ifaddresses(self.args.interface)[ni.AF_INET][0]['addr']
        sniffer = pcap.pcap(name=self.args.interface, promisc=False, immediate=True, timestamp_in_ns=True)
        sniffer.setfilter(f'not ether broadcast and src not {self.local_ip}')
        print(Color.BOLD + f'Sniffing {self.args.interface} started, ' + Color.ENDC)
        print(Color.BOLD + f'not ether broadcast and src not {self.local_ip}' + Color.ENDC)

        for ts, buf in sniffer:
            if self.stopped():
                sniffer.close()
                break
            self.process_packet(ts, buf)

    def process_packet(self, ts: int, buf: bytes, index: Optional[int] = -1) -> NoReturn:
        """Processes each captured packet.

        Args:
            ts (int): Timestamp of the packet capture.
            buf (bytes): The raw packet data.
            index (Optional[int]): Index of the packet in the capture file. Defaults to -1.
        """
        try:
            eth = dpkt.ethernet.Ethernet(buf)
            if eth.type == 17157:  # batman type
                if eth.dst == b'\xff\xff\xff\xff\xff\xff':  # ignore OGM
                    return
                len_eth = 14
                len_bat = 10
                eth = dpkt.ethernet.Ethernet(buf[len_eth + len_bat:])  # unwrap batman eth

            if eth.type in [dpkt.ethernet.ETH_TYPE_IP, dpkt.ethernet.ETH_TYPE_IP6]:
                self.unpack_ip(eth, ts)

        except dpkt.dpkt.NeedData:
            logging.warning(f'Packet {index} in PCAP file is truncated')

    def unpack_ip(self, eth: dpkt.ethernet.Ethernet, timestamp) -> NoReturn:
        """Unpacks IP packets and handles different protocols.

        Args:
            eth (dpkt.ethernet.Ethernet): Ethernet frame containing the IP packet.
            timestamp: Timestamp associated with the packet.
        """
        ip: Optional[dpkt.ip.IP, dpkt.ip6.IP6] = eth.data

        # Handle IP fragments for IPv4 packets
        if isinstance(ip, dpkt.ip.IP):
            is_fragmented = (ip.off & (dpkt.ip.IP_MF | dpkt.ip.IP_OFFMASK)) != 0
            if is_fragmented:
                # return
                ip = self.handle_ipv4_fragments(ip, timestamp)
                if ip is None:
                    return  # If the packet is fragmented and not fully reassembled, do not proceed
        elif isinstance(ip, dpkt.ip6.IP6):
            # Fragment header's value is 44 as per IANA assignments
            if dpkt.ip.IP_PROTO_FRAGMENT in ip.extension_hdrs:
                ext_frag = ip.extension_hdrs[dpkt.ip.IP_PROTO_FRAGMENT]
                # Check if it's a non-last fragment or not
                is_fragmented = (ext_frag.frag_off & 0xfff8) != 0 or ext_frag.m_flag
                if is_fragmented:
                    return  # TODO: handle IPV6 fragmentation

        protocol = ip.p
        if protocol == dpkt.ip.IP_PROTO_TCP:
            flows: Dict[FlowKeyType, int] = self.flows_tcp
        elif protocol == dpkt.ip.IP_PROTO_UDP:
            flows: Dict[FlowKeyType, int] = self.flows_udp
        else:
            flows: Dict[FlowKeyType, int] = self.flows_udp

        src_ip: bytes = ip.src
        dst_ip: bytes = ip.dst
        if protocol in [6, 17]:
            # If protocol is TCP but data does not correspond to it, then create TCP placeholder
            if protocol == 6 and not isinstance(ip.data, dpkt.tcp.TCP):
                ip.data = dpkt.tcp.TCP(sport=-1, dport=-1, seq=-1, ack=0, off_x2=0, flags=0, win=0, sum=0, urp=0)
            # If protocol is UDP but data does not correspond to it, then create UDP placeholder
            elif protocol == 17 and not isinstance(ip.data, dpkt.udp.UDP):
                ip.data = dpkt.udp.UDP(sport=-1, dport=-1, ulen=0, sum=0)

            src_port: int = ip.data.sport
            dst_port: int = ip.data.dport
        else:
            src_port: int = 0
            dst_port: int = 0
        # Compute unidirectional flow id
        flow_id: Tuple[bytes, int, bytes, int, int] = (src_ip, src_port, dst_ip, dst_port, protocol)

        # Get traffic id (same if unidirectional, sorted if bidirectional)
        traffic_id = get_flow_key(*flow_id, self.args.session)
        last_timestamp: Optional[int] = flows.get(traffic_id, False)

        # Check for flow timeout given threshold
        if last_timestamp and timestamp - last_timestamp > self.timeout_ns:
            self.report_teardown(flow_id, protocol)

        # Update latest timestamp and report packet
        flows[traffic_id] = timestamp
        self.report_packet(flow_id, protocol, timestamp, ip)

    def handle_ipv4_fragments(self, ip: dpkt.ip.IP, timestamp) -> Optional[dpkt.ip.IP]:
        """Handles reassembling IPv4 fragmented packets.

        Args:
            ip (dpkt.ip.IP): The fragmented IP packet.
            timestamp: Timestamp associated with the packet.

        Returns:
            Optional[dpkt.ip.IP]: Reassembled IP packet or None if incomplete.
        """
        # Check if the packet is fragmented
        is_fragmented = (ip.off & (dpkt.ip.IP_MF | dpkt.ip.IP_OFFMASK)) != 0

        # If the packet is not fragmented, return the original packet
        if not is_fragmented:
            return ip

        # Key for identifying the original packet of the fragment
        fragment_key = (ip.src, ip.dst, ip.id)

        # Getting the raw payload bytes
        ip_header_len = ip.hl * 4  # Calculates the length of the IP header in bytes.
        raw_payload = bytes(ip)[ip_header_len:]  # Extracts the payload from the IP packet

        # Check if the buffer needs to be reset due to time expiration
        if fragment_key in self.fragments_cache:
            last_timestamp = self.fragments_cache[fragment_key].get('last_timestamp', 0)
            # Check if the time difference is greater than 60 seconds
            if timestamp - last_timestamp > sec_to_ns(60):
                # Reset the buffer
                self.fragments_cache[fragment_key] = {'fragments': [], 'total_length': None, 'current_length': 0}

        # Store the fragment in the cache
        if fragment_key not in self.fragments_cache:
            self.fragments_cache[fragment_key] = {'fragments': [], 'total_length': None, 'last_timestamp': timestamp,
                                                  'current_length': 0}

        # Multiply the offset by 8 to get the actual byte offset
        self.fragments_cache[fragment_key]['fragments'].append(((ip.off & dpkt.ip.IP_OFFMASK) * 8, raw_payload))
        # Update the last timestamp
        self.fragments_cache[fragment_key]['last_timestamp'] = timestamp

        # Update the current length
        self.fragments_cache[fragment_key]['current_length'] += len(raw_payload)

        # If this is the last fragment, calculate the total length of the original data
        if not bool(ip.off & dpkt.ip.IP_MF):
            last_fragment_offset = (ip.off & dpkt.ip.IP_OFFMASK) * 8
            self.fragments_cache[fragment_key]['total_length'] = last_fragment_offset + len(raw_payload)

        # Check if we have all the data
        total_length = self.fragments_cache[fragment_key]['total_length']
        current_length = self.fragments_cache[fragment_key]['current_length']

        if total_length is not None and current_length == total_length:
            # Reassemble the packet
            fragments = self.fragments_cache[fragment_key]['fragments']
            fragments.sort()

            # Create a bytearray of the correct size filled with zeros
            reassembled_data = bytearray(total_length)

            # Insert fragment data at the correct offsets
            for offset, fragment_data in fragments:
                reassembled_data[offset:offset + len(fragment_data)] = fragment_data

            # Construct a new IP packet with the reassembled payload
            reassembled_packet = dpkt.ip.IP()

            # Copy the header fields from the original IP packet
            reassembled_packet.src = ip.src
            reassembled_packet.dst = ip.dst
            reassembled_packet.p = ip.p
            reassembled_packet.id = ip.id
            reassembled_packet.off = 0  # Reset the fragment
            reassembled_packet.len = len(reassembled_data)  # Set the correct total length
            reassembled_packet.data = reassembled_data

            try:
                # If the protocol is one that we know how to parse, do so
                if reassembled_packet.p in protocol_classes:
                    transport_cls = protocol_classes[reassembled_packet.p]
                    reassembled_packet.data = transport_cls(reassembled_packet.data)
            except dpkt.dpkt.UnpackError:
                # Log the error or do something else as necessary
                # print("Error: invalid header length during packet reassembly.")
                return None

            # Clear the fragments cache for this packet
            del self.fragments_cache[fragment_key]

            return reassembled_packet

        # If we don't have all the data yet, return None
        return None

    def add_subscriber(self, subscriber: SnifferSubscriber) -> bool:
        """Adds a new subscriber to the list of subscribers.

        Args:
            subscriber (SnifferSubscriber): The subscriber to be added.

        Returns:
            bool: True if the subscriber is successfully added.
        """
        assert issubclass(
            type(subscriber), SnifferSubscriber
        ), f'subscriber should be subclass of {SnifferSubscriber}'
        self.subscribers.append(subscriber)
        return True

    def start_subscribers(self) -> NoReturn:
        """Starts all registered subscribers."""
        for subscriber in self.subscribers:
            subscriber.start()

    def report_packet(self, flow_key: FlowKeyType, protocol: int, timestamp: int, ip: dpkt.ip.IP) -> NoReturn:
        """Reports a new packet to all subscribers.

        Args:
            flow_key (FlowKeyType): The flow key associated with the packet.
            protocol (int): The protocol number of the packet.
            timestamp (int): Timestamp of the packet.
            ip (dpkt.ip.IP): The IP packet.
        """
        for subscriber in self.subscribers:
            subscriber.new_packet(flow_key, protocol, timestamp, ip)

    def report_teardown(self, flow_key: FlowKeyType, protocol: int) -> NoReturn:
        """Reports the teardown of a flow to all subscribers.

        Args:
            flow_key (FlowKeyType): The flow key associated with the flow.
            protocol (int): The protocol number of the flow.
        """
        for subscriber in self.subscribers:
            subscriber.teardown(flow_key, protocol)


class SnifferPcap(Sniffer):
    """A sniffer subclass for handling pcap files.

    Attributes:
        _current_pcap_path (str): Path to the current pcap file being processed.

    Args:
        args (SnifferOptions): Configuration options passed during initialization.
    """

    def __init__(self, args: SnifferOptions):
        """Initializes the SnifferPcap with given arguments.

        Args:
            args (SnifferOptions): Configuration options for the sniffer.
        """
        super(SnifferPcap, self).__init__(args)
        self._current_pcap_path: str = ''

    def set_pcap_path(self, path: str) -> NoReturn:
        """Sets the path of the pcap file to be processed.

        Args:
            path (str): The path of the pcap file.
        """
        self._current_pcap_path = path

    def sniff(self, show_tqdm: bool = True) -> NoReturn:
        """Processes packets from a pcap file.

        Args:
            show_tqdm (bool): Whether to show progress with tqdm. Defaults to True.
        """
        self.flows_tcp: dict = {}
        self.flows_udp: dict = {}

        prev_ts = None
        logging.info(f'Preprocessing {self._current_pcap_path}')
        with open(self._current_pcap_path, 'rb') as file:
            pcap_file = list(dpkt.pcap.Reader(file))
            print(f'Processing {self._current_pcap_path}')
            time.sleep(0.1)  # Sleep to avoid IO overlap with tqdm

            # Enumerate over packets
            packet_iter = enumerate(pcap_file)
            if show_tqdm:
                packet_iter = tqdm(packet_iter, desc='Processing packets', total=len(pcap_file))

            for i, (ts, buf) in packet_iter:
                # Handle delay between packets
                if prev_ts is not None:
                    delay = ts - prev_ts
                    if delay > 0 and self.args.delay:
                        time.sleep(delay)
                prev_ts = ts

                # Process each packet
                ts_ns = sec_to_ns(ts)
                self.process_packet(ts_ns, buf)
