import msgpack
import signal
import socket
import sys
import threading
import time
import uuid
from collections import Counter
from enum import auto, Enum
from typing import List, Tuple, Dict
from netstring import encode, decode

from options import Options
from util import get_mesh_freq, map_freq_to_channel
from log_config import logger

action_to_id = {
    "jamming_alert": 0,
    "estimation_request": 1,
    "estimation_report": 2,
    "target_frequency": 3,
    "switch_frequency": 4
}
id_to_action = {v: k for k, v in action_to_id.items()}


class ServerState(Enum):
    IDLE = auto()
    CHECK_IF_TARGET_FREQ_JAMMED = auto()
    REQUEST_SPECTRUM_DATA = auto()
    GATHER_SPECTRUM_DATA = auto()
    FREQ_HOPPING_DECISION = auto()
    UPDATE_TARGET_FREQ_SEND_SWITCH_FREQ = auto()
    BROADCAST_TARGET_FREQ = auto()
    RESET_CLIENT_MESSAGES = auto()
    RESET_COMPLETE = auto()


class ServerEvent(Enum):
    JAM_ALERT = auto()
    VALID_JAM_ALERT = auto()
    INVALID_JAM_ALERT = auto()
    SENT_SPECTRUM_DATA_REQUEST = auto()
    GATHERED_ALL_SPECTRUM_DATA = auto()
    SPECTRUM_DATA_GATHERING_EXPIRED = auto()
    NEW_TARGET_FREQUENCY = auto()
    SWITCH_REQUEST_SENT = auto()
    TARGET_FREQ_EQ_CURRENT = auto()
    NO_ESTIMATES_SENT = auto()
    PERIODIC_TARGET_FREQ_BROADCAST = auto()
    BROADCAST_COMPLETE = auto()
    RESET_COMPLETE = auto()


class ServerFSM:
    def __init__(self, server):
        self.args = Options()
        self.state: ServerState = ServerState.IDLE  # Set the initial state
        self.server: JammingServer = server  # Reference to the server object

        # Transition table
        self.transitions = {
            (ServerState.IDLE, ServerEvent.JAM_ALERT): (ServerState.CHECK_IF_TARGET_FREQ_JAMMED, self.server.check_jammed_frequency),
            (ServerState.IDLE, ServerEvent.PERIODIC_TARGET_FREQ_BROADCAST): (ServerState.BROADCAST_TARGET_FREQ, self.server.broadcast_target_freq),
            (ServerState.BROADCAST_TARGET_FREQ, ServerEvent.BROADCAST_COMPLETE): (ServerState.IDLE, None),
            (ServerState.CHECK_IF_TARGET_FREQ_JAMMED, ServerEvent.VALID_JAM_ALERT): (ServerState.REQUEST_SPECTRUM_DATA, self.server.request_spectrum_data),
            (ServerState.CHECK_IF_TARGET_FREQ_JAMMED, ServerEvent.INVALID_JAM_ALERT): (ServerState.RESET_CLIENT_MESSAGES, self.server.reset_client_attributes),
            (ServerState.REQUEST_SPECTRUM_DATA, ServerEvent.SENT_SPECTRUM_DATA_REQUEST): (ServerState.GATHER_SPECTRUM_DATA, self.server.gathering_spectrum_data),
            (ServerState.GATHER_SPECTRUM_DATA, ServerEvent.GATHERED_ALL_SPECTRUM_DATA): (ServerState.FREQ_HOPPING_DECISION, self.server.frequency_hopping_decision),
            (ServerState.GATHER_SPECTRUM_DATA, ServerEvent.SPECTRUM_DATA_GATHERING_EXPIRED): (ServerState.FREQ_HOPPING_DECISION, self.server.frequency_hopping_decision),
            (ServerState.FREQ_HOPPING_DECISION, ServerEvent.NEW_TARGET_FREQUENCY): (ServerState.UPDATE_TARGET_FREQ_SEND_SWITCH_FREQ, self.server.send_switch_frequency_message),
            (ServerState.FREQ_HOPPING_DECISION, ServerEvent.TARGET_FREQ_EQ_CURRENT): (ServerState.RESET_CLIENT_MESSAGES, self.server.reset_client_attributes),
            (ServerState.FREQ_HOPPING_DECISION, ServerEvent.NO_ESTIMATES_SENT): (ServerState.REQUEST_SPECTRUM_DATA, self.server.request_spectrum_data),
            (ServerState.UPDATE_TARGET_FREQ_SEND_SWITCH_FREQ, ServerEvent.SWITCH_REQUEST_SENT): (ServerState.RESET_CLIENT_MESSAGES, self.server.reset_client_attributes),
            (ServerState.RESET_CLIENT_MESSAGES, ServerEvent.RESET_COMPLETE): (ServerState.IDLE, None)
        }

    def trigger(self, event) -> None:
        """Function to handle state transitions"""
        self._process_event(event)

    def _process_event(self, event) -> None:
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


class JammingServer:
    def __init__(self, host: str, port: int):
        """
        Initializes the JammingServer object.

        :param host: The host address to bind the orchestrator/server to.
        :param port: The port number to bind the orchestrator/server to.
        """
        # Initialize server objects and attributes
        self.running = False
        self.fsm = ServerFSM(self)
        self.host = host
        self.port = port
        self.serversocket = None
        self.clients: List[JammingClientTwin] = []
        self.args = Options()
        logger.info("Initialized server")

        # Create threads for server operations
        self.run_server_fsm_thread = threading.Thread(target=self.run_server_fsm)

        # Time for periodic events' variables (in seconds)
        self.last_requested_spectrum_data: float = time.time()
        self.last_target_freq_broadcast: float = time.time()

        # Internal Attributes
        self.target_frequency: int = get_mesh_freq()
        self.healing_process_id: str = str(uuid.uuid4())  # Generate a unique ID at the start

        signal.signal(signal.SIGINT, self.signal_handler)

    def start(self) -> None:
        """
        Starts the server and listens for incoming client connections.
        """
        try:
            self.running = True
            self.serversocket = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
            self.serversocket.bind((self.host, self.port))
            self.serversocket.listen(10)
            logger.info("Server started and listening")
            self.run_server_fsm_thread.start()
            signal.signal(signal.SIGINT, self.signal_handler)

            while self.running:
                c_socket, c_address = self.serversocket.accept()
                client = JammingClientTwin(c_socket, c_address, self.clients, self.host)
                logger.info(f'New connection {client}')
                self.clients.append(client)

        except ConnectionError as e:
            logger.info(f"Connection error: {e}")

    def signal_handler(self, sig: signal.Signals, frame) -> None:
        """
        Handles a signal interrupt (SIGINT) and stops the server gracefully.

        :param sig: The signal received by the handler.
        :param frame: The current execution frame.
        """
        logger.info("Attempting to close threads.")
        if self.clients:
            for client in self.clients:
                logger.info(f"Joining {client.address}")
                client.stop()

            logger.info("Threads successfully closed")
            sys.exit(0)

        self.stop()

    def stop(self) -> None:
        """
        Stop the server by closing the server socket and server fsm thread.
        """
        self.running = False

        # Close server socket
        if self.serversocket:
            self.serversocket.close()

        # Join Server FSM thread
        if self.run_server_fsm_thread.is_alive():
            self.run_server_fsm_thread.join()

    def run_server_fsm(self) -> None:
        """
        Run the Server Finite State Machine (FSM) continuously to manage its state transitions and periodic tasks.
        """
        while self.running:
            try:
                if self.fsm.state == ServerState.IDLE:
                    current_time = time.time()
                    self.check_jam_alert()
                    # Check if it's time to perform a periodic target frequency broadcast
                    if current_time - self.last_target_freq_broadcast >= self.args.periodic_target_freq_broadcast:
                        self.last_target_freq_broadcast = time.time()
                        self.fsm.trigger(ServerEvent.PERIODIC_TARGET_FREQ_BROADCAST)
                time.sleep(1)
            except Exception as e:
                logger.info(f"Exception in run_server_fsm: {e}")

    def check_jam_alert(self) -> None:
        """
        Check for a jamming alert message from any connected client.
        """
        for client in self.clients:
            if client.jam_alert_message:
                self.fsm.trigger(ServerEvent.JAM_ALERT)
                break

    def send_data_clients(self, data) -> None:
        """
        Sends the estimated frequency quality data to a remote server.

        :param data: The message to send to the clients.
        """
        for client in self.clients:
            try:
                serialized_data = msgpack.packb(data)
                netstring_data = encode(serialized_data)
                client.socket.sendall(netstring_data)
            except BrokenPipeError:
                logger.info(f"Broken pipe error, client disconnected: {client.address}")
                self.clients.remove(client)
            except Exception as e:
                logger.info(f"Error sending data to client {client.address}: {e}")
                self.clients.remove(client)

    def check_jammed_frequency(self, trigger_event) -> None:
        """
        Check if the client's current frequency is equal to the target frequency to ensure jam alert is valid to consider.

        :param trigger_event: ServerEvent that triggered the execution of this function.
        """
        for client in self.clients:
            jam_alert_message = client.jam_alert_message
            if jam_alert_message:
                if self.target_frequency == jam_alert_message['freq']:
                    logger.info("-- VALID JAM ALERT")
                    self.fsm.trigger(ServerEvent.VALID_JAM_ALERT)
                    return

        # If there is no valid jam alert, then we move back to IDLE
        logger.info("-- INVALID JAM ALERT")
        self.fsm.trigger(ServerEvent.INVALID_JAM_ALERT)

    def broadcast_target_freq(self, trigger_event) -> None:
        """
        Broadcast target frequency to all clients.

        :param trigger_event: ServerEvent that triggered the execution of this function.
        """
        try:
            action_id: int = action_to_id["target_frequency"]
            target_freq_data = {'a_id': action_id, 'freq': self.target_frequency}
            self.send_data_clients(target_freq_data)
            self.fsm.trigger(ServerEvent.BROADCAST_COMPLETE)
        except Exception as e:
            logger.info(f"Broadcast target frequency error: {e}")

    def request_spectrum_data(self, trigger_event) -> None:
        """
        Sends a message to all connected clients to send spectrum data.

        :param trigger_event: ServerEvent that triggered the execution of this function.
        """
        try:
            action_id: int = action_to_id["estimation_request"]
            request_spectrum_data = {'a_id': action_id, "h_id": self.healing_process_id}
            self.send_data_clients(request_spectrum_data)
            self.last_requested_spectrum_data = time.time()
            self.fsm.trigger(ServerEvent.SENT_SPECTRUM_DATA_REQUEST)
        except Exception as e:
            logger.info(f"Error executing request_spectrum_data: {e}")

    def gathering_spectrum_data(self, trigger_event) -> None:
        """
        Gather spectrum data from clients until all data is gathered or until time expiry.

        :param trigger_event: ServerEvent that triggered the execution of this function.
        """
        current_time = time.time()
        all_data_gathered = self.check_all_spectrum_data_gathered()

        while (current_time - self.last_requested_spectrum_data) < self.args.data_gathering_wait_time:
            current_time = time.time()
            all_data_gathered = self.check_all_spectrum_data_gathered()
            if all_data_gathered:
                logger.info("-- ALL CLIENTS SENT DATA")
                self.fsm.trigger(ServerEvent.GATHERED_ALL_SPECTRUM_DATA)
                break
            time.sleep(1)

        if not all_data_gathered:
            logger.info("-- DATA GATHERING TIME EXPIRED")
            self.fsm.trigger(ServerEvent.SPECTRUM_DATA_GATHERING_EXPIRED)

    def check_all_spectrum_data_gathered(self) -> bool:
        """
        Check if all clients sent the data before time expiry.

        :return: A boolean representing whether all the clients have sent their spectrum data.
        """
        all_data_gathered = True
        for client in self.clients:
            # Only consider messages for current healing process
            if client.estimation_report_message:
                message_id = client.estimation_report_message.get("h_id")
                if message_id != self.healing_process_id:
                    all_data_gathered = False
                    break
            # Check if the client did not send the data estimation
            elif not client.estimation_report_message:
                all_data_gathered = False
                break  # No need to continue checking, we already know it's False

        return all_data_gathered

    def frequency_hopping_decision(self, trigger_event) -> None:
        """
        Process the spectrum data received from the clients to take a switch frequency decision.

        :param trigger_event: ServerEvent that triggered the execution of this function.
        """
        client_estimates = self.get_client_estimates()

        if client_estimates:
            # Count the occurrences of each frequency in the client_estimates list
            freq_counts = Counter(client_estimates.values())
            most_voted_freqs = freq_counts.most_common()
            logger.info(f"most_voted_freqs: {most_voted_freqs}")
            orchestrator_best_freq = client_estimates.get(self.host)
            most_voted_freq, most_voted_freq_count = most_voted_freqs[0]

            # If most voted frequency count is equal to 1, get the orchestrator client's frequency estimation
            if most_voted_freq_count == 1:
                if orchestrator_best_freq is not None:
                    logger.info(f"-- Orchestrator estimation {orchestrator_best_freq}")
                    switch_to_frequency = orchestrator_best_freq
                # else get the estimation from other client
                else:
                    switch_to_frequency = most_voted_freq
            # If we have more than one vote, get most voted frequency
            else:
                logger.info(f"-- Most voted freq (Votes: {most_voted_freq_count})")
                switch_to_frequency = most_voted_freq

            # Check decision outcome
            if switch_to_frequency == self.target_frequency:
                self.fsm.trigger(ServerEvent.TARGET_FREQ_EQ_CURRENT)
            else:
                self.target_frequency = switch_to_frequency
                self.fsm.trigger(ServerEvent.NEW_TARGET_FREQUENCY)
        else:
            logger.info("No estimates sent")
            time.sleep(3)
            self.fsm.trigger(ServerEvent.NO_ESTIMATES_SENT)

    def get_client_estimates(self) -> Dict:
        """
        Get all clients' frequency quality estimate information.

        :return: A dictionary mapping the client IPv6 address to the frequency data it sent.
        """
        client_estimates = {}
        for client in self.clients:
            if client.estimation_report_message:
                message_healing_id = client.estimation_report_message.get("h_id")
                if message_healing_id == self.healing_process_id:
                    client_ip = client.estimation_report_message.get("n_id")
                    freq = client.estimation_report_message.get("freq")
                    if map_freq_to_channel(freq) in self.args.channels5:
                        client_estimates[client_ip] = freq
        logger.info(f"client_estimates: {client_estimates}")
        return client_estimates

    def send_switch_frequency_message(self, trigger_event) -> None:
        """
        Sends a message to all connected clients to switch to target frequency.

        :param trigger_event: ServerEvent that triggered the execution of this function.
        """
        action_id = action_to_id["switch_frequency"]
        switch_frequency_data = {'a_id': action_id, 'freq': self.target_frequency}
        self.send_data_clients(switch_frequency_data)
        self.fsm.trigger(ServerEvent.SWITCH_REQUEST_SENT)

    def reset_client_attributes(self, trigger_event):
        """
        Reset each connected clients' attributes.

        :param trigger_event: The event that triggered the reset.
        """
        self.healing_process_id = str(uuid.uuid4())  # Generate a new ID at the end of the healing process
        logger.info("-- NEW HEALING ID")
        for client in self.clients:
            client.reset()
        self.fsm.trigger(ServerEvent.RESET_COMPLETE)


class JammingClientTwin(threading.Thread):
    def __init__(self, socket: socket.socket, address: Tuple[str, int], clients: List["JammingClientTwin"], host: str) -> None:
        threading.Thread.__init__(self)
        self.socket = socket
        self.address = address
        self.clients = clients
        self.host = host
        self.running: bool = True
        self.args = Options()
        self.jam_alert_message: Dict = {}
        self.estimation_report_message: Dict = {}
        self.listen_thread = threading.Thread(target=self.receive_messages)
        self.start()

    def run(self) -> None:
        """
        Start the client's operation in a separate thread to continuously listen for incoming messages.
        """
        self.listen_thread.start()

    def receive_messages(self) -> None:
        """
        Handles incoming messages from the client.
        """
        while self.running:
            try:
                # Receive incoming messages and decode the netstring encoded data
                try:
                    data = decode(self.socket.recv(1024))
                    if not data:
                        logger.info("No data... break")
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

                    # Jam Alert received from client
                    if action_str == "jamming_alert":
                        self.jam_alert_message = unpacked_data

                    # Requested Spectrum Data received from client
                    elif action_str == "estimation_report":
                        self.estimation_report_message = unpacked_data

                except msgpack.UnpackException as e:
                    logger.error(f"Failed to decode MessagePack: {e}")
                    continue

                except Exception as e:
                    logger.error(f"Error in received message: {e}")
                    continue

            except ConnectionResetError:
                logger.error("Connection forcibly closed by the remote host")
                break

    def reset(self):
        """
        Reset message related client attributes to their default values.
        """
        self.jam_alert_message = {}
        self.estimation_report_message = {}

    def stop(self) -> None:
        """
        Stops all threads and closes the socket connection.
        """
        self.running = False
        # Close server socket
        if self.socket:
            self.socket.close()
            # Join listen thread
        if self.listen_thread.is_alive():
            self.listen_thread.join()


def main():
    args = Options()

    host: str = args.jamming_osf_orchestrator
    port: int = args.port
    server: JammingServer = JammingServer(host, port)
    server.start()


if __name__ == "__main__":
    main()
