import asyncio
import socket
import sys
import threading
import time
from collections import deque
from enum import auto, Enum

import msgpack
import numpy as np
import pandas as pd
from netstring import encode, decode

from channel_quality_est import ChannelQualityEstimator
from options import Options
from preprocessor import Preprocessor
from util import get_frequency_quality, get_mesh_freq, get_ipv6_addr, switch_frequency
from wireless_scanner import WirelessScanner
from log_config import logger

action_to_id = {
    "jamming_alert": 0,
    "estimation_request": 1,
    "estimation_report": 2,
    "target_frequency": 3,
    "switch_frequency": 4
}
id_to_action = {v: k for k, v in action_to_id.items()}


class ClientState(Enum):
    IDLE = auto()
    LOW_LATENCY_SCAN = auto()
    HIGH_LATENCY_SCAN = auto()
    REPORTING_DATA = auto()
    SWITCHING_FREQUENCY = auto()
    SENDING_JAM_ALERT = auto()
    RECOVERING_SWITCH_ERROR = auto()
    RESETTING = auto()


class ClientEvent(Enum):
    PERIODIC_LOW_LATENCY_SCAN = auto()
    HIGH_LATENCY_SCAN_PERFORMED = auto()
    NO_JAM_DETECTED = auto()
    JAM_DETECTED = auto()
    JAM_ALERT_SENT = auto()
    DATA_REPORT_SENT = auto()
    SWITCHED = auto()
    SWITCH_UNSUCCESSFUL = auto()
    PERIODIC_SWITCH = auto()
    EXT_SWITCH_EVENT = auto()
    EXT_DATA_REQUEST = auto()
    RESET_COMPLETE = auto()


class UniqueDeque:
    def __init__(self):
        self.d = deque()
        self.seen = set()
        self.lock = threading.Lock()

    def append(self, event):
        """Add event to the end if it hasn't been added before."""
        with self.lock:
            if event not in self.seen:
                self.d.append(event)
                self.seen.add(event)

    def _popleft(self):
        """Remove and return the first value. If the deque is empty, return None."""
        if self.d:
            value = self.d.popleft()
            self.seen.remove(value)
            return value
        return None

    def pop_all(self):
        """Pop all elements and return them as a list."""
        with self.lock:
            result = []
            while self.d:
                result.append(self._popleft())
            return result

    def reset(self):
        """Reset the deque by clearing its contents and resetting internal state."""
        with self.lock:
            self.d.clear()
            self.seen.clear()


class ClientFSM:
    def __init__(self, client):
        # Initial state
        self.args = Options()
        self.state = ClientState.IDLE
        self.client = client
        self.event_queue = UniqueDeque()

        # Transition table
        self.transitions = {
            (ClientState.IDLE, ClientEvent.PERIODIC_LOW_LATENCY_SCAN): (ClientState.LOW_LATENCY_SCAN, self.client.performing_scan),
            (ClientState.IDLE, ClientEvent.EXT_SWITCH_EVENT): (ClientState.SWITCHING_FREQUENCY, self.client.switch_frequency),
            (ClientState.IDLE, ClientEvent.EXT_DATA_REQUEST): (ClientState.HIGH_LATENCY_SCAN, self.client.performing_scan),
            (ClientState.LOW_LATENCY_SCAN, ClientEvent.NO_JAM_DETECTED): (ClientState.IDLE, None),
            (ClientState.LOW_LATENCY_SCAN, ClientEvent.JAM_DETECTED): (ClientState.SENDING_JAM_ALERT, self.client.sending_jam_alert),
            (ClientState.LOW_LATENCY_SCAN, ClientEvent.EXT_DATA_REQUEST): (ClientState.HIGH_LATENCY_SCAN, self.client.performing_scan),
            (ClientState.LOW_LATENCY_SCAN, ClientEvent.EXT_SWITCH_EVENT): (ClientState.SWITCHING_FREQUENCY, self.client.switch_frequency),
            (ClientState.SENDING_JAM_ALERT, ClientEvent.JAM_ALERT_SENT): (ClientState.IDLE, None),
            (ClientState.SENDING_JAM_ALERT, ClientEvent.EXT_DATA_REQUEST): (ClientState.HIGH_LATENCY_SCAN, self.client.performing_scan),
            (ClientState.SENDING_JAM_ALERT, ClientEvent.EXT_SWITCH_EVENT): (ClientState.SWITCHING_FREQUENCY, self.client.switch_frequency),
            (ClientState.HIGH_LATENCY_SCAN, ClientEvent.HIGH_LATENCY_SCAN_PERFORMED): (ClientState.REPORTING_DATA, self.client.report_spec_quality),
            (ClientState.REPORTING_DATA, ClientEvent.DATA_REPORT_SENT): (ClientState.IDLE, None),
            (ClientState.REPORTING_DATA, ClientEvent.EXT_SWITCH_EVENT): (ClientState.SWITCHING_FREQUENCY, self.client.switch_frequency),
            (ClientState.SWITCHING_FREQUENCY, ClientEvent.SWITCHED): (ClientState.RESETTING, self.client.reset),
            (ClientState.SWITCHING_FREQUENCY, ClientEvent.SWITCH_UNSUCCESSFUL): (ClientState.RECOVERING_SWITCH_ERROR, self.client.recovering_switch_error),
            (ClientState.RECOVERING_SWITCH_ERROR, ClientEvent.PERIODIC_SWITCH): (ClientState.SWITCHING_FREQUENCY, self.client.switch_frequency),
            (ClientState.RECOVERING_SWITCH_ERROR, ClientEvent.EXT_SWITCH_EVENT): (ClientState.SWITCHING_FREQUENCY, self.client.switch_frequency),
            (ClientState.RESETTING, ClientEvent.RESET_COMPLETE): (ClientState.IDLE, None)
        }

    def is_external_event(self, event: ClientEvent) -> bool:
        """
        Check if an event originated from a message received from the orchestrator.

        :param event: The event to check.
        :return: True if the event is an external event, False otherwise.
        """
        return event in [ClientEvent.EXT_DATA_REQUEST, ClientEvent.EXT_SWITCH_EVENT]

    def trigger(self, event: ClientEvent) -> None:
        """Function to handle state transitions"""
        # If it's an external event, queue it and return
        if self.is_external_event(event):
            self.event_queue.append(event)
            if self.state != ClientState.IDLE:
                return
        else:
            self._process_event(event)

        # Process any queued external events
        event_list = self.event_queue.pop_all()
        if event_list:
            for event in event_list:
                self._process_event(event)

    def _process_event(self, event: ClientEvent) -> None:
        """Internal function to process the given event"""
        key = (self.state, event)
        if key in self.transitions:
            next_state, action = self.transitions[key]
            logger.info(f'{self.state} -> {next_state}')
            self.state = next_state
            if action:
                action(event)
        else:
            logger.info(f"No transition found for event '{event}' in state '{self.state}'")


class JammingDetectionClient(threading.Thread):
    def __init__(self, node_id: str, host: str, port: int) -> None:
        super().__init__()
        self.node_id = node_id
        self.host = host
        self.port = port

        # Initialize server objects
        self.fsm = ClientFSM(self)
        self.args = Options()
        self.scanner = WirelessScanner()
        self.prep = Preprocessor()
        self.estimator = ChannelQualityEstimator()

        # Internal variables
        self.socket = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
        self.current_frequency: int = get_mesh_freq()
        self.healing_process_id: str = None
        self.target_frequency: int = np.nan
        self.best_freq: int = np.nan
        self.freq_quality: dict = {}

        # Time for periodic events' variables (in seconds)
        self.time_last_scan: float = 0
        self.time_last_switch: float = 0

        # Create listen and client run FSM threads
        self.running = False
        self.run_thread = threading.Thread(target=self.run_client_fsm)
        self.listen_thread = threading.Thread(target=self.receive_messages)

    def run(self) -> None:
        """
        Connect to the orchestrator node and start the client's operation in separate threads for running the FSM and receiving messages.
        """
        self.running = True
        self.connect_to_orchestrator()
        self.run_thread.start()
        self.listen_thread.start()

    def connect_to_orchestrator(self) -> None:
        """
        Connect to orchestrator via OSF.
        """
        max_retries: int = 3
        number_retries: int = 0
        while number_retries < max_retries:
            try:
                self.socket.connect((self.host, self.port))
                break
            except ConnectionRefusedError:
                number_retries += 1
                logger.error(f"OSF server connection failed... retry #{number_retries}")

            if number_retries == max_retries:
                sys.exit("OSF Server unreachable")

            time.sleep(2)

    def run_client_fsm(self) -> None:
        """
        Run the client Finite State Machine (FSM), managing its state transitions and periodic tasks.
        """
        while self.running:
            try:
                if self.fsm.state == ClientState.IDLE:
                    current_time = time.time()
                    # Check if it's time to perform a periodic scan
                    if current_time - self.time_last_scan >= self.args.periodic_scan:
                        self.fsm.trigger(ClientEvent.PERIODIC_LOW_LATENCY_SCAN)
                # Sleep for a short duration before checking conditions again
                time.sleep(0.1)
            except Exception as e:
                logger.error(f"Exception in run: {e}")

    def receive_messages(self) -> None:
        """
        Receive incoming messages from the orchestrator.
        """
        while self.running:
            try:
                # Receive incoming messages and decode the netstring encoded data
                try:
                    data = decode(self.socket.recv(1024))
                    if not data:
                        logger.info("No data...")
                        break
                except Exception as e:
                    # Handle netstring decoding errors
                    logger.error(f"Failed to decode netstring: {e}")
                    break

                # Deserialize the MessagePack message
                try:
                    unpacked_data = msgpack.unpackb(data, raw=False)
                    action_id: int = unpacked_data.get("a_id")
                    action_str: str = id_to_action.get(action_id)
                    logger.info(f"Received message: {unpacked_data}")

                    # Handle frequency estimation request
                    if action_str == "estimation_request":
                        self.healing_process_id = unpacked_data.get("h_id")
                        self.fsm.trigger(ClientEvent.EXT_DATA_REQUEST)

                    # Handle frequency switch request
                    elif action_str == "switch_frequency":
                        received_target_freq = unpacked_data.get("freq")
                        self.update_target_freq(received_target_freq)
                        if self.current_frequency != self.target_frequency:
                            self.fsm.trigger(ClientEvent.EXT_SWITCH_EVENT)

                    # Recalibrate to server frequency
                    elif action_str == "target_frequency":
                        received_target_freq = unpacked_data.get("freq")
                        self.update_target_freq(received_target_freq)
                        if self.current_frequency != self.target_frequency:
                            self.fsm.trigger(ClientEvent.EXT_SWITCH_EVENT)

                except msgpack.UnpackException as e:
                    logger.error(f"Failed to decode MessagePack: {e}")
                    continue

                except Exception as e:
                    logger.error(f"Error in received message: {e}")
                    continue

            except ConnectionResetError:
                logger.error("Connection forcibly closed by the remote host")
                break

    def performing_scan(self, trigger_event) -> None:
        """
        Perform spectral scan on the specified frequencies.

        :param trigger_event: ClientEvent that triggered the execution of the performing_scan function.
        :return: A dataframe of the spectral scan performed.
        """
        logger.info('Performing Scan...')
        scan: pd.DataFrame = pd.DataFrame()

        if self.fsm.state == ClientState.LOW_LATENCY_SCAN:
            scan = self.scanner.scan_current_frequency()
        elif self.fsm.state == ClientState.HIGH_LATENCY_SCAN:
            scan = self.scanner.scan_all_frequencies()

        # Preprocess the scan
        self.time_last_scan = time.time()
        feat_array, frequencies = self.prep.preprocess(scan)

        # Estimate the frequency quality and pick the frequency with the highest quality estimation
        frequencies_quality, probs, quality_normalized = self.estimator.estimate(feat_array, frequencies)
        jamming_detected, self.freq_quality = get_frequency_quality(frequencies_quality, probs, frequencies)
        # Check that frequencies_quality is not empty
        if frequencies_quality.size > 0:
            self.best_freq = int(frequencies[np.argmax(frequencies_quality)])

        # After scan report data to CH
        if self.fsm.state == ClientState.LOW_LATENCY_SCAN:
            if jamming_detected:
                logger.info('Detection JAMMING')
                self.fsm.trigger(ClientEvent.JAM_DETECTED)
            else:
                logger.info('Detection NORMAL')
                self.fsm.trigger(ClientEvent.NO_JAM_DETECTED)
        elif self.fsm.state == ClientState.HIGH_LATENCY_SCAN:
            self.fsm.trigger(ClientEvent.HIGH_LATENCY_SCAN_PERFORMED)

    def sending_jam_alert(self, trigger_event) -> None:
        """
        Send jam alert to orchestrator to report jamming.

        :param trigger_event: ClientEvent that triggered the execution of this function.
        """
        curr_freq: int = get_mesh_freq()
        action_id: int = action_to_id["jamming_alert"]
        data = {'a_id': action_id, 'n_id': self.node_id, 'freq': curr_freq}
        self.send_messages(data)
        self.fsm.trigger(ClientEvent.JAM_ALERT_SENT)

    def report_spec_quality(self, trigger_event) -> None:
        """
        Report spectrum frequency estimation to orchestrator if data is valid.

        :param trigger_event: ClientEvent that triggered the execution of this function.
        """
        # Send scan data
        logger.info("Reporting Spectrum Quality...")
        action_id: int = action_to_id["estimation_report"]
        data = {'a_id': action_id, 'n_id': self.node_id, 'freq': self.best_freq, 'qual': self.freq_quality, 'h_id': self.healing_process_id}
        self.send_messages(data)
        self.fsm.trigger(ClientEvent.DATA_REPORT_SENT)

    def send_messages(self, data) -> None:
        """
        Sends the estimated frequency quality data to a remote server.

        :param data: The message to send to orchestrator.
        """
        try:
            serialized_data = msgpack.packb(data)
            netstring_data = encode(serialized_data)
            self.socket.send(netstring_data)
            logger.info(f"Sent {data}")
        except ConnectionRefusedError:
            logger.error("Connection refused by the server. Check if the server is running and reachable.")

    def switch_frequency(self, trigger_event) -> None:
        """
        Change the mesh frequency to the target frequency.

        :param trigger_event: ClientEvent that triggered the execution of this function.
        """
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(switch_frequency(str(self.target_frequency)))

            # Validate outcome of switch frequency process
            self.current_frequency = get_mesh_freq()

            if self.current_frequency != self.target_frequency:
                logger.info("Frequency switch unsuccessful")
                self.fsm.trigger(ClientEvent.SWITCH_UNSUCCESSFUL)
            else:
                logger.info("Frequency switch successful")
                self.fsm.trigger(ClientEvent.SWITCHED)

        except Exception as e:
            logger.error(f"Switching frequency error occurred: {str(e)}")
            self.fsm.trigger(ClientEvent.SWITCH_UNSUCCESSFUL)
        finally:
            loop.close()

    def recovering_switch_error(self, trigger_event) -> None:
        """
        Handle recovering from a switch error by periodically attempting frequency switching.

        :param trigger_event: The event that triggered the recovery process.
        """
        self.time_last_scan = time.time()
        while self.running:
            current_time = time.time()
            # If periodic switch timer ended, switch frequency again
            if current_time - self.time_last_scan >= self.args.periodic_recovery_switch:
                self.fsm.trigger(ClientEvent.PERIODIC_SWITCH)
                self.time_last_scan = current_time
                break

            time.sleep(0.01)

    def reset(self, trigger_event) -> None:
        """
        Reset Client FSM related attributes.

        :param trigger_event: ClientEvent that triggered the execution of this function.
        """
        self.time_last_scan = 0
        self.time_last_switch = 0
        self.fsm.event_queue.reset()
        self.fsm.trigger(ClientEvent.RESET_COMPLETE)

    def update_target_freq(self, received_target_freq):
        """
        Update the target mesh frequency.

        :param received_target_freq: The new target frequency to be set.
        """
        self.target_frequency = received_target_freq

    def stop(self) -> None:
        """
        Stops all threads and closes the socket connection.
        """
        self.running = False
        # Close socket
        if self.socket:
            self.socket.close()
        # Join run client FSM thread
        if self.run_thread.is_alive():
            self.run_thread.join()
        # Join listen thread
        if self.listen_thread.is_alive():
            self.listen_thread.join()


def main():
    args = Options()

    host: str = args.jamming_osf_orchestrator
    port: int = args.port
    node_id: str = get_ipv6_addr(args.osf_interface)

    client = JammingDetectionClient(node_id, host, port)
    client.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Attempting to stop the clients.")
        client.stop()
        logger.info("Clients successfully stopped.")


if __name__ == '__main__':
    main()
