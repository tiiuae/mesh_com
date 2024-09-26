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
from itertools import islice
import subprocess

from config import Config
from rmacs_setup import get_mesh_freq
from logging_config import logger

action_to_id = {
    "bad_channel_quality_index": 0,
    "channel_quality_report": 1,
    "operating_frequency": 2,
    "switch_frequency": 3
}
id_to_action = {v: k for k, v in action_to_id.items()}


class ServerState(Enum):
    IDLE = auto()
    ADAPTIVE_FREQUENCY_HOPPING = auto()
    SEND_CHANNEL_SWITCH_REQUEST = auto()
    UPDATE_FREQ_HOPPING_SEQUENCE = auto()
    BROADCAST_OPERATING_FREQ = auto()
    RESET_CLIENT_MESSAGES = auto()
    RESET_COMPLETE = auto()


class ServerEvent(Enum):
    BAD_CHANNEL_QUALITY_INDEX = auto()
    #CHANNEL_QUALITY_REPORT = auto()
    NEW_TARGET_FREQUENCY = auto()
    CHANNEL_SWITCH_REQUEST = auto()
    CHANNEL_SWITCH_REQUEST_SENT = auto()
    TARGET_FREQ_EQ_CURRENT = auto()
    PERIODIC_OPERATING_FREQ_BROADCAST = auto()
    BROADCAST_COMPLETE = auto()
    #CHANNEL_QUALITY_UPDATE_COMPLETE = auto()
    FREQUENCY_HOPPING_COMPLETE = auto()
    RESET_COMPLETE = auto()


class RMACSServerFSM:
    def __init__(self, server):
        self.args = Config()
        self.state: ServerState = ServerState.IDLE  # Set the initial state
        self.server: RMACSServer = server  # Reference to the server object

        # Transition table
        self.transitions = {
            (ServerState.IDLE, ServerEvent.BAD_CHANNEL_QUALITY_INDEX): (ServerState.ADAPTIVE_FREQUENCY_HOPPING, self.server.adaptive_frequency_hopping),
            #(ServerState.IDLE, ServerEvent.CHANNEL_QUALITY_REPORT): (ServerState.UPDATE_FREQ_HOPPING_SEQUENCE, self.server.broadcast_target_freq),
            (ServerState.IDLE, ServerEvent.PERIODIC_OPERATING_FREQ_BROADCAST): (ServerState.BROADCAST_OPERATING_FREQ, self.server.broadcast_target_freq),
            #(ServerState.UPDATE_FREQ_HOPPING_SEQUENCE, ServerEvent.CHANNEL_QUALITY_UPDATE_COMPLETE): (ServerState.IDLE, None),
            (ServerState.BROADCAST_OPERATING_FREQ, ServerEvent.BROADCAST_COMPLETE): (ServerState.IDLE, None),
            (ServerState.ADAPTIVE_FREQUENCY_HOPPING, ServerEvent.CHANNEL_SWITCH_REQUEST): (ServerState.SEND_CHANNEL_SWITCH_REQUEST, None),
            (ServerState.SEND_CHANNEL_SWITCH_REQUEST, ServerEvent.CHANNEL_SWITCH_REQUEST_SENT): (ServerState.ADAPTIVE_FREQUENCY_HOPPING, None),
            (ServerState.ADAPTIVE_FREQUENCY_HOPPING, ServerEvent.FREQUENCY_HOPPING_COMPLETE): (ServerState.RESET_CLIENT_MESSAGES, None),
            (ServerState.RESET_CLIENT_MESSAGES, ServerEvent.RESET_COMPLETE): (ServerState.IDLE, None),
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


class RMACSServer:
    def __init__(self, host: str, port: int):
        """
        Initializes the RMACS Server object.

        :param host: The host address to bind the orchestrator/server to.
        :param port: The port number to bind the orchestrator/server to.
        """
        # Initialize server objects and attributes
        self.running = False
        self.fsm = RMACSServerFSM(self)
        self.host = host
        self.port = port
        self.serversocket = None
        self.clients: List[RMACSClientTwin] = []
        self.args = Config()
        logger.info("Initialized server")
        self.nw_interface = self.args.nw_interface
        self.freq_quality_report = self.args.freq_quality_report
        self.seq_limit = self.args.seq_limit
        self.hop_interval = self.args.hop_interval
        self.stability_threshold = self.args.stability_threshold
        

        #frequencies: list[tuple], seq_limit: int, hop_interval: int, stability_threshold: int
        # Initialize the lock for thread-safe access
        self.lock = threading.Lock()

        # Create threads for server operations
        self.run_server_fsm_thread = threading.Thread(target=self.run_server_fsm)

        # Time for periodic events' variables (in seconds)
        #self.last_requested_spectrum_data: float = time.time()
        self.last_operating_freq_broadcast: float = time.time()

        # Internal Attributes
        self.operating_frequency: int = get_mesh_freq()
        self.switch_frequency: int = self.operating_frequency
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
                client = RMACSClientTwin(c_socket, c_address, self.clients, self.host)
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
                    if self.check_bad_channel_quality_index():
                        self.fsm.trigger(ServerEvent.BAD_CHANNEL_QUALITY_INDEX)
                    # Check if it's time to perform a periodic target frequency broadcast
                    if current_time - self.last_target_freq_broadcast >= self.args.periodic_target_freq_broadcast:
                        self.last_target_freq_broadcast = time.time()
                        self.fsm.trigger(ServerEvent.PERIODIC_OPERATING_FREQ_BROADCAST)
                
                self.check_and_update_channel_quality_report()                 
                time.sleep(1)
            except Exception as e:
                logger.info(f"Exception in run_server_fsm: {e}")

    def check_bad_channel_quality_index(self) -> bool:
        """
        Check for a BAD_CHANNEL_QUALITY_INDEX message from any connected client.
        """
        for client in self.clients:
            if client.bad_channel_message:
                self.freq = client.bad_channel_message['freq']
                self.quality_index = client.bad_channel_message['qual']
                self.freq_quality_report.update({self.freq: self.quality_index})
                return True
        return False
    
    
    def check_and_update_channel_quality_report(self) -> None:
        """
        Check for a channel quality report message from any connected client.
        """
        for client in self.clients:
            if client.channel_report_message:
                self.freq = client.channel_report_message['freq']
                self.quality_index = client.channel_report_message['qual']
                self.freq_quality_report.update({self.freq: self.quality_index})
                
                # Parse the channel_quality_message and update freq_quality_report list
                #self.fsm.trigger(ServerEvent.CHANNEL_QUALITY_REPORT)
                # write the logic to update here.
            

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


    def broadcast_target_freq(self, trigger_event) -> None:
        """
        Broadcast Operating frequency to all clients.

        :param trigger_event: ServerEvent that triggered the execution of this function.
        """
        try:
            action_id: int = action_to_id["operating_frequency"]
            operating_freq_data = {'a_id': action_id, 'freq': self.operating_frequency}
            self.send_data_clients(operating_freq_data)
            self.fsm.trigger(ServerEvent.BROADCAST_COMPLETE)
        except Exception as e:
            logger.info(f"Broadcast target frequency error: {e}")
    
        # Adaptive frequency hopping with dynamic intervals and exit condition based on top frequency stability
    def adaptive_frequency_hopping(self, trigger_event):
        # 
        top_freq_stability_counter, index = 0
        sorted_frequencies = sorted(self.freq_quality_report, key=lambda x: x[1])
        top_freq = sorted_frequencies[0]
        # Ensure the seq_limit doesn't exceed the length of sorted_frequencies
        seq_limit = min(seq_limit, len(sorted_frequencies))
        while top_freq_stability_counter < self.stability_threshold:
            # Continue hopping between the best frequencies
            self.switch_freq = sorted_frequencies[index]
            self.fsm.trigger(ServerEvent.CHANNEL_SWITCH_REQUEST)
            
            # Wait for the current hop interval before switching to the next frequency
            time.sleep(self.hop_interval)
            # Move to the next frequency in the list
            index = (index + 1) % self.seq_limit
            with self.lock:
                # Sort by the second item in the tuple (quality) in ascending order (best quality first)
                sorted_frequencies = sorted(self.hopping_frequencies, key=lambda x: x[1])
                # Select the hop frequency sequence from the top x sorted frequencies
                #hop_freq_seq = islice(sorted_frequencies,seq_limit)
                current_top_freq = sorted_frequencies[0]
            if current_top_freq == top_freq:
                top_freq_stability_counter += 1
                print(f"Top frequency {top_freq} remained the same for {top_freq_stability_counter} consecutive checks.")
            else:
                top_freq_stability_counter = 0  # Reset counter if top frequency changes
                top_freq = current_top_freq  # Update the top frequency
                print(f"Top frequency changed to {top_freq}.")
            
            # Exit if the top frequency stays the same for the stability threshold period
            if top_freq_stability_counter >= self.stability_threshold:
                print(f"Top frequency {top_freq} has been stable for {self.stability_threshold} consecutive checks. Exiting.")
                self.switch_frequency(top_freq)
                self.operating_frequency = top_freq
                self.switch_freq = top_freq
                self.fsm.trigger(ServerEvent.CHANNEL_SWITCH_REQUEST)
                break
          
        if top_freq_stability_counter >= self.stability_threshold:
            self.fsm.trigger(ServerEvent.FREQUENCY_HOPPING_COMPLETE)
    
    def switch_frequency(frequency: int, interface: str, bandwidth: int, beacons_count: int ) -> None:
        run_cmd = f"iw dev {interface} switch freq {frequency} {bandwidth}MHz beacons {beacons_count}"
        try:
            result = subprocess.run(run_cmd, 
                                shell=True, 
                                capture_output=True, 
                                text=True)
            if(result.returncode != 0):
                print("Failed to execute the switch frequency command")

        except subprocess.CalledProcessError as e:
            print(f"Error: {e}")
            return None


    def send_switch_frequency_message(self, trigger_event) -> None:
        """
        Sends a message to all connected clients to switch to target frequency.

        :param trigger_event: ServerEvent that triggered the execution of this function.
        """
        action_id = action_to_id["switch_frequency"]
        switch_frequency_data = {'a_id': action_id, 'freq': self.operating_frequency}
        self.send_data_clients(switch_frequency_data)
        self.fsm.trigger(ServerEvent.CHANNEL_SWITCH_REQUEST_SENT)

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


class RMACSClientTwin(threading.Thread):
    def __init__(self, socket: socket.socket, address: Tuple[str, int], clients: List["RMACSClientTwin"], host: str) -> None:
        threading.Thread.__init__(self)
        self.socket = socket
        self.address = address
        self.clients = clients
        self.host = host
        self.running: bool = True
        self.args = Config()
        self.bad_channel_message: Dict = {}
        self.channel_report_message: Dict = {}
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

                    # Bad channel quality index received from client
                    if action_str == "bad_channel_quality_index":
                        self.bad_channel_message = unpacked_data

                    # Channel report received from client
                    elif action_str == "channel_quality_report":
                        self.channel_report_message = unpacked_data

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
        self.bad_channel_message = {}
        self.channel_report_message = {}

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
    args = Config()
    host: str = args.rmacs_osf_orchestrator
    port: int = args.port
    server: RMACSServer = RMACSServer(host, port)
    server.start()


if __name__ == "__main__":
    main()