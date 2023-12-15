import glob
import logging
import os
from queue import Queue
from typing import Dict, Tuple, List, NoReturn

import dpkt
import numpy as np
from sklearn.metrics import accuracy_score
from tqdm import tqdm

from options import LDPIOptions, SnifferOptions
from sniffer.sniffer import Sniffer
from utils import SnifferSubscriber, sec_to_ns


class PreprocessingSnifferPcap(Sniffer):
    """
    A subclass of Sniffer that preprocesses and sniffs packets from pcap files.
    It supports handling multiple pcap files and applies preprocessing logic to each packet.

    Attributes:
        args (SnifferOptions): Configuration options for the sniffer.
        current_pcap_path (str): The current path of the pcap file being processed.
        subscribers (List[LDPIPreProcessing]): List of subscribers to notify about pcap processing.
        max_samples_per_pcap (int): Maximum number of samples to process per pcap file.
    """

    def __init__(self, args: SnifferOptions) -> NoReturn:
        """
        Initialize the PreprocessingSnifferPcap with given arguments.

        Args:
            args (SnifferOptions): Configuration options for the sniffer.
        """
        super().__init__(args)
        self.current_pcap_path: str = ''
        self.subscribers: List[LDPIPreProcessing] = []
        self.max_samples_per_pcap: int = 5000

    def sniff(self) -> NoReturn:
        """
        Start the sniffing process, which involves processing pcap files from specified directories.
        Raises ValueError if subscribers list is empty.
        """
        if not self.subscribers:
            raise ValueError("Subscribers list cannot be empty.")

        dataset_name, benign, malware = self._get_dataset_info()
        logging.basicConfig(level=logging.INFO)

        for pcap_path in benign + malware:
            self._set_current_pcap_path(pcap_path)
            self._process_pcap_file(pcap_path)

    def _get_dataset_info(self) -> Tuple[str, List[str], List[str]]:
        """
        Retrieves dataset information including the name and paths of benign and malicious pcap files.

        Returns:
            Tuple[str, List[str], List[str]]: Dataset name, list of benign pcap file paths, list of malicious pcap file paths.
        """
        # Resolve the relative path to an absolute path
        dataset_name = os.path.abspath(self.args.dataset_path)

        # Path for benign pcap files, recursively search through subdirectories
        benign_path = os.path.join(dataset_name, 'benign', '**', '*.pcap')
        benign_files = glob.glob(benign_path, recursive=True)

        # Path for malware pcap files, recursively search through subdirectories
        malicious_path = os.path.join(dataset_name, 'malicious', '**', '*.pcap')
        malicious_files = glob.glob(malicious_path, recursive=True)

        if not benign_files:
            raise FileNotFoundError(f"No benign pcap files found in directory: {benign_path}")
        if not malicious_files:
            raise FileNotFoundError(f"No malicious pcap files found in directory: {malicious_path}")

        # Extract the dataset name from the path for return value
        dataset_basename = os.path.basename(dataset_name)

        return dataset_basename, benign_files, malicious_files

    def _set_current_pcap_path(self, pcap_path: str) -> NoReturn:
        """
        Sets the current pcap path based on the provided pcap file path.

        Args:
            pcap_path (str): The path to the current pcap file.
        """
        dir_only = os.path.dirname(pcap_path)
        if 'TII-SSRC-23' in dir_only:
            index = dir_only.find('TII-SSRC-23')
            filename = os.path.basename(pcap_path).split('.pcap')[0]
            storing_path = dir_only[index:]
            self.current_pcap_path = f'{storing_path}/{filename}/'
            self.subscribers[0].set_current_path(self.current_pcap_path)

    def _process_pcap_file(self, pcap_path: str) -> NoReturn:
        """
        Processes a single pcap file.

        Args:
            pcap_path (str): The path to the pcap file to be processed.
        """
        self.flows_tcp.clear()
        self.flows_udp.clear()

        with open(pcap_path, 'rb') as file:
            logging.info(f'Preprocessing {self.current_pcap_path}')
            pcap_file = list(dpkt.pcap.Reader(file))  # convert to list so tqdm progress bar works

            for i, (ts, buf) in tqdm(enumerate(pcap_file), desc=f'Processing {pcap_path}', total=len(pcap_file)):
                ts_ns = sec_to_ns(ts)
                self.process_packet(ts_ns, buf)
                sample_counter = self._handle_subscriber_processing()

                if sample_counter > self.max_samples_per_pcap:
                    break

    def _handle_subscriber_processing(self) -> int:
        """
        Handles the processing logic for subscribers.

        Returns:
            int: The current sample counter after processing.
        """
        subscriber = self.subscribers[0]
        if not subscriber.to_process.empty():
            subscriber.preprocess_samples()

        return subscriber.sample_counter


class LDPIPreProcessing(SnifferSubscriber):
    """ LDPI preprocessing routines """

    def __init__(self) -> NoReturn:
        """
        Initialize the LDPIPreProcessing instance with default values.
        """
        super(LDPIPreProcessing, self).__init__()
        self.args = LDPIOptions()
        self.flows_tcp: Dict[Tuple[bytes, int, bytes, int], List[bytes]] = {}
        self.flows_udp: Dict[Tuple[bytes, int, bytes, int], List[bytes]] = {}
        self.c_tcp: Dict[Tuple[bytes, int, bytes, int], bool] = {}
        self.c_udp: Dict[Tuple[bytes, int, bytes, int], bool] = {}
        self.to_process: Queue = Queue()
        self.sample_counter: int = 0
        self.current_path: str = ""

    def run(self) -> NoReturn:
        """
        Placeholder for the main logic to run the LDPIPreProcessing instance.
        """
        pass

    def new_packet(self, flow_key: Tuple[bytes, int, bytes, int], protocol: int, timestamp: int, ip: dpkt.ip.IP) -> NoReturn:
        """
        Processes a new packet received from the network.

        Args:
            flow_key (Tuple[bytes, int, bytes, int]): Unique key identifying the network flow.
            protocol (int): Protocol number (e.g., 6 for TCP).
            timestamp (int): Timestamp of the packet.
            ip (dpkt.ip.IP): IP packet to be processed.
        """
        flows, checked_flows = self.get_buffers(protocol)

        # Drop packet if it's part of a flow that's already processed
        if checked_flows.get(flow_key, False):
            return

        # For TCP packets, check for FIN/RST flags
        if protocol == 6:
            if ip.data.flags & dpkt.tcp.TH_RST and ip.data.flags & dpkt.tcp.TH_ACK:
                self.teardown(flow_key, protocol)
                return

        # Extract packet bytes, anonymize if necessary
        ip_bytes = anonymize_packet(ip)

        # Manage flow
        flow = flows.setdefault(flow_key, [])
        flow.append(ip_bytes)

        # Check for FIN flag
        if protocol == 6 and (ip.data.flags & dpkt.tcp.TH_FIN):
            self.to_process.put((flow_key, flow))
            self.teardown(flow_key, protocol)
        elif len(flow) == self.args.n:
            # When a flow reaches the desired packet count (n)
            checked_flows[flow_key] = True
            self.to_process.put((flow_key, flow))
            del flows[flow_key]

    # Remove flows entries in case of teardown
    def teardown(self, flow_key: Tuple[bytes, int, bytes, int], protocol: int) -> NoReturn:
        """
        Handles the teardown of a network flow, removing it from active monitoring.

        Args:
            flow_key (Tuple[bytes, int, bytes, int]): Unique key identifying the network flow.
            protocol (int): Protocol number (e.g., 6 for TCP).
        """
        # Buffer of flows and checked flows of given protocol
        flows, checked_flows = self.get_buffers(protocol)
        removed_flows = flows.pop(flow_key, False)
        removed_checked_flows = checked_flows.pop(flow_key, False)
        # if removed_flows or removed_checked_flows:
        #     str_key = flow_key_to_str(flow_key)
        #     print(Color.UNDERLINE + f'{str_key} teardown ({self.to_process.qsize()})' + Color.ENDC)

    def preprocess_samples(self) -> np.ndarray:
        """
        Preprocesses the collected samples, preparing them for analysis or storage.

        Returns:
            np.ndarray: An array of preprocessed samples.
        """
        # Dequeue all samples for this iteration
        samples = []
        while not self.to_process.empty():
            value = self.to_process.get()
            samples.append(value[1])

        # Create 3-dimensional np array (flow->packet->byte)
        sample_size = self.args.n * self.args.l
        norm_flows = np.zeros([len(samples), sample_size], dtype=np.uint8)

        # Clean/anonymize packets, normalize and trim bytes and fill norm_flows
        for i in range(len(samples)):
            flow = samples[i]
            flow_length = len(flow)

            for j in range(self.args.n):
                if j < flow_length:
                    # Process actual packets in the flow
                    packet = flow[j]
                    np_buff = trim_or_pad_packet(packet, self.args.l)
                else:
                    # Create zero byte packets for missing packets
                    np_buff = np.zeros(self.args.l, dtype=np.uint8)

                norm_flows[i][j * self.args.l: (j + 1) * self.args.l] = np_buff
        if False:
            print(f'Storing samples {norm_flows.shape[0]} samples at {self.current_path} - Sample counter: {self.sample_counter}')

        for i in range(norm_flows.shape[0]):
            np.save(f'{self.current_path}/{self.sample_counter}.npy', norm_flows[i])
            self.sample_counter += 1
            # print(norm_flows[i].shape)

    def perf_measure(self, y_true: np.ndarray, scores: np.ndarray) -> float:
        """
        Measures the performance of a model using the true labels and the model's scores.

        Args:
            y_true (np.ndarray): Array of true labels.
            scores (np.ndarray): Array of scores from the model.

        Returns:
            float: The accuracy of the model.
        """
        y_pred = np.empty_like(y_true)
        for i in range(len(y_true)):
            if scores[i] < self.threshold:
                y_pred[i] = 0
            else:
                y_pred[i] = 1

        # tn, fp, fn, tp = confusion_matrix(y_true, y_pred).ravel()
        accuracy = accuracy_score(y_true, y_pred)
        return accuracy

    def get_buffers(self, protocol: int) -> Tuple[Dict, Dict]:
        """
        Returns the appropriate buffers for the specified protocol.

        Args:
            protocol (int): Protocol number (e.g., 6 for TCP).

        Returns:
            Tuple[Dict, Dict]: The flows and checked flows dictionaries for the specified protocol.
        """
        return (self.flows_tcp, self.c_tcp) if protocol == 6 else (self.flows_udp, self.c_udp)

    def set_current_path(self, storing_path: str) -> NoReturn:
        """
        Sets the current path for storing processed data and resets internal data structures.

        Args:
            storing_path (str): The path for storing data.
        """
        self.flows_tcp: Dict[tuple, int] = {}
        self.flows_udp: Dict[tuple, int] = {}
        self.c_tcp: Dict[tuple, int] = {}
        self.c_udp: Dict[tuple, int] = {}
        self.to_process: Queue = Queue()
        self.sample_counter = 0
        self.current_path = f'samples/{storing_path}'

        if not os.path.isdir(self.current_path):
            os.makedirs(self.current_path)


def anonymize_packet(ip: dpkt.ip.IP) -> bytes:
    """
    Anonymizes an IP packet by removing sensitive information such as IP addresses.

    Args:
        ip (dpkt.ip.IP): The IP packet to be anonymized.

    Returns:
        bytes: The anonymized IP packet.
    """
    ip_bytes = bytes(ip)

    # IP header is the first 20 bytes of the IP packet
    # Source IP is at bytes 12-15 and Destination IP is at bytes 16-19
    # We remove these bytes to anonymize the packet
    anonymized_ip_bytes = ip_bytes[:12] + ip_bytes[20:]

    return anonymized_ip_bytes


def trim_or_pad_packet(packet: bytes, length: int) -> np.ndarray:
    """
    Trims or pads a packet to a specific length.

    Args:
        packet (bytes): The packet to be modified.
        length (int): The desired length of the packet.

    Returns:
        np.ndarray: The modified packet.
    """
    # Trim to length in case packet is larger than length
    if len(packet) > length:
        packet = packet[:length]

    np_buff = np.frombuffer(bytes(packet), dtype=np.uint8)

    # If smaller than the desired length, pad with zeros
    if len(np_buff) < length:
        padded_np_buff = np.zeros(length, dtype=np.uint8)
        padded_np_buff[0:len(np_buff)] = np_buff
        return padded_np_buff

    return np_buff


def main() -> NoReturn:
    """
    Main function to initialize and run the preprocessing sniffer for pcap files.

    This function sets up logging based on debug flags, initializes the PreprocessingSnifferPcap
    with appropriate arguments, and attaches an LDPIPreProcessing instance as a subscriber.
    """
    # Process PCAP into network flow samples
    sniffer_args = SnifferOptions()

    # Configure logging based on the debug flag
    if sniffer_args.debug:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.INFO)

    # Create preprocessing sniffing process and run it with LDPI preprocessing routine
    fsnf = PreprocessingSnifferPcap(sniffer_args)
    fsnf.subscribers.append(LDPIPreProcessing())
    fsnf.run(daemon=False)


if __name__ == '__main__':
    main()
