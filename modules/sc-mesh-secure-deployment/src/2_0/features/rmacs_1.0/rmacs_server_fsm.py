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
from rmacs_setup import get_mesh_freq, get_ipv6_addr, channel_switch_announcement
from logging_config import logger
from rmacs_comms import rmacs_comms, send_data

MULTICAST_GROUP = 'ff02::1'  # IPv6 multicast address for entire network
MULTICAST_PORT = 12345

action_to_id = {
    "bad_channel_quality_index": 0,
    "channel_quality_report": 1,
    "operating_frequency": 2,
    "switch_frequency": 3
}
id_to_action = {v: k for k, v in action_to_id.items()}


class ServerState(Enum):
    IDLE = auto()
    PREPARE_PARTIAL_FREQUENCY_HOPPING = auto()
    PARTIAL_FREQUENCY_HOPPING = auto()
    SEND_CHANNEL_SWITCH_REQUEST = auto()
    UPDATE_FREQ_HOPPING_SEQUENCE = auto()
    BROADCAST_OPERATING_FREQ = auto()
    RESET_CLIENT_MESSAGES = auto()
    RESET_COMPLETE = auto()


class ServerEvent(Enum):
    BAD_CHANNEL_QUALITY_INDEX = auto()
    CHANNEL_QUALITY_REPORT = auto()
    NEW_TARGET_FREQUENCY = auto()
    CHANNEL_SWITCH_REQUEST = auto()
    CHANNEL_SWITCH_REQUEST_SENT = auto()
    TARGET_FREQ_EQ_CURRENT = auto()
    PERIODIC_OPERATING_FREQ_BROADCAST = auto()
    BROADCAST_COMPLETE = auto()
    CHANNEL_QUALITY_UPDATE_COMPLETE = auto()
    FREQUENCY_HOPPING_COMPLETE = auto()
    RESET_COMPLETE = auto()
    PREPARATION_DONE = auto()


class RMACSServerFSM:
    def __init__(self, server):
        self.args = Config()
        self.state = ServerState.IDLE
        self.server = server
        #self.state: ServerState = ServerState.IDLE  # Set the initial state
        #self.server: RMACSServer = server  # Reference to the server object

        # Transition table
        self.transitions = {
            #(ServerState.IDLE, ServerEvent.BAD_CHANNEL_QUALITY_INDEX): (ServerState.IDLE, None),
            (ServerState.IDLE, ServerEvent.BAD_CHANNEL_QUALITY_INDEX): (ServerState.PARTIAL_FREQUENCY_HOPPING, self.server.partial_frequency_hopping),
            (ServerState.IDLE, ServerEvent.CHANNEL_QUALITY_REPORT): (ServerState.UPDATE_FREQ_HOPPING_SEQUENCE, self.server.check_and_update_channel_quality_report),
            (ServerState.IDLE, ServerEvent.PERIODIC_OPERATING_FREQ_BROADCAST): (ServerState.BROADCAST_OPERATING_FREQ, self.server.broadcast_operating_freq),
            (ServerState.UPDATE_FREQ_HOPPING_SEQUENCE, ServerEvent.CHANNEL_QUALITY_UPDATE_COMPLETE): (ServerState.IDLE, None),
            (ServerState.BROADCAST_OPERATING_FREQ, ServerEvent.BROADCAST_COMPLETE): (ServerState.IDLE, None),
            #(ServerState.PREPARE_PARTIAL_FREQUENCY_HOPPING, ServerEvent.PREPARATION_DONE): (ServerState.PARTIAL_FREQUENCY_HOPPING, self.server.partial_frequency_hopping),
            (ServerState.PARTIAL_FREQUENCY_HOPPING, ServerEvent.CHANNEL_SWITCH_REQUEST): (ServerState.SEND_CHANNEL_SWITCH_REQUEST, self.server.send_switch_frequency_message),
            (ServerState.SEND_CHANNEL_SWITCH_REQUEST, ServerEvent.CHANNEL_SWITCH_REQUEST_SENT): (ServerState.PARTIAL_FREQUENCY_HOPPING, self.server.partial_frequency_hopping),
            (ServerState.PARTIAL_FREQUENCY_HOPPING, ServerEvent.FREQUENCY_HOPPING_COMPLETE): (ServerState.IDLE, None),
            (ServerState.RESET_CLIENT_MESSAGES, ServerEvent.RESET_COMPLETE): (ServerState.IDLE, None),
        }

    def trigger(self, event) -> None:
        """Function to handle state transitions"""
        print(f" Trigger event arrived {event}")
        self._process_event(event)

    def _process_event(self, event) -> None:
        """Internal function to process the given event"""
        key = (self.state, event)
        if key in self.transitions:
            next_state, action = self.transitions[key]
            logger.info(f'{self.state} -> {next_state}')
            print(f'{self.state} -> {next_state}')
            self.state = next_state
            if action:
                action(event)
                print(f'process event : {action}')
        else:
            logger.info(f"No transition found for event '{event}' in state '{self.state}'")
            print(f"No transition found for event '{event}' in state '{self.state}'")


class RMACSServer:
    def __init__(self):
        """
        Initializes the RMACS Server object.

        :param host: The host address to bind the orchestrator/server to.
        :param port: The port number to bind the orchestrator/server to.
        """
        # Initialize server objects and attributes
        self.running = False
        self.fsm = RMACSServerFSM(self)
        #self.host = host
        #self.port = port
        #self.serversocket = None
        #self.clients: List[RMACSClientTwin] = []
        self.args = Config()
        logger.info("Initialized server")
        self.nw_interface = self.args.nw_interface
        self.freq_quality_report = self.args.freq_quality_report
        self.seq_limit = self.args.seq_limit
        self.hop_interval = self.args.hop_interval
        self.stability_threshold = self.args.stability_threshold
        
       # Receive msg 
        self.bad_channel_message: Dict = {}
        self.channel_report_message: Dict = {}
        

        # Variables for Partial Frequency Hopping
        
        self.top_freq_stability_counter = 0
        self.pfh_index = 0
        self.seq_limit = self.args.seq_limit
        self.sorted_frequencies: list = [] 
        self.top_freq = 0
        
        # Variables for Channel Switch 
        self.channel_bandwidth: int = self.args.channel_bandwidth
        self.beacons_count: int = self.args.beacons_count
        self.buffer_period: int = self.args.buffer_period

        #frequencies: list[tuple], seq_limit: int, hop_interval: int, stability_threshold: int
        # Initialize the lock for thread-safe access
        self.lock = threading.Lock()

        # Create threads for server operations
        self.run_server_fsm_thread = threading.Thread(target=self.run_server_fsm)
        self.listen_thread = threading.Thread(target=self.receive_messages)
        # Time for periodic events' variables (in seconds)
        self.last_operating_freq_broadcast: float = time.time()

        # Internal Attributes
        self.operating_frequency: int = get_mesh_freq(self.nw_interface)
        self.switch_freq: int = self.operating_frequency
        self.healing_process_id: str = str(uuid.uuid4())  # Generate a unique ID at the start

        #signal.signal(signal.SIGINT, self.signal_handler)

    def start(self) -> None:
        """
        Starts the server and listens for incoming client connections.
        """
        print("Start the server-------------")
        try:     
            self.running = True
            self.connect_to_rmacs_comms()
            print("Server started and listening")
            self.run_server_fsm_thread.start()
            self.listen_thread.start()
            #signal.signal(signal.SIGINT, self.signal_handler)

            

        except ConnectionError as e:
            logger.info(f"Connection error: {e}")
            

            
    def connect_to_rmacs_comms(self) -> None:
        """
        Connect to rmacs_comms via br-lan
        """
        max_retries: int = 3
        number_retries: int = 0
        print('connect to rmacs comms')
        while number_retries < max_retries:
            try:
                self.socket = rmacs_comms()
                print("Socket Connection is successfull")
                break
            except ConnectionRefusedError:
                number_retries += 1
                #logger.error(f"OSF server connection failed... retry #{number_retries}")

            if number_retries == max_retries:
                sys.exit("Server unreachable")

            time.sleep(2)
    '''
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
    '''

    def stop(self) -> None:
        """
        Stop the server by closing the server socket and server fsm thread.
        """
        self.running = False

        # Close server socket
        if self.socket:
            self.socket.close()

        # Join Server FSM thread
        if self.run_server_fsm_thread.is_alive():
            self.run_server_fsm_thread.join()

    def run_server_fsm(self) -> None:
        """
        Run the Server Finite State Machine (FSM) continuously to manage its state transitions and periodic tasks.
        """
        
        print("Run RMACS server-----------")
        count = 0
    
        while self.running:
            try:
                if self.fsm.state == ServerState.IDLE:
                    #print(f" rmacs server {self.fsm.state}")
                    current_time = time.time()
                    #print(f"Check for rmacs server self running  : count:{count}, time : {self.last_operating_freq_broadcast}")
                    #print(f" current state {self.fsm.state}")
                    count +=1
                    
                    if self.bad_channel_message:
                        self.update_channel_quality_report(self.bad_channel_message)
                        self.bad_channel_message = None
                        self.fsm.trigger(ServerEvent.BAD_CHANNEL_QUALITY_INDEX)
                    
                    # Check if it's time to perform a periodic operating frequency broadcast
                    if current_time - self.last_operating_freq_broadcast >= self.args.periodic_operating_freq_broadcast:
                        print("***time to broadcast")
                        self.last_operating_freq_broadcast = time.time()
                        self.fsm.trigger(ServerEvent.PERIODIC_OPERATING_FREQ_BROADCAST)
                
                # self.check_and_update_channel_quality_report()   
                              
                if self.channel_report_message:
                    print(" Received channel report and update it:")
                    self.fsm.trigger(ServerEvent.CHANNEL_QUALITY_REPORT)
                    
                time.sleep(2)
            except Exception as e:
                logger.info(f"Exception in run_server_fsm: {e}")
    
    
    def check_and_update_channel_quality_report(self,trigger_event) -> None:
        """
        Check for a channel quality report message from any connected client.
        """
        print(" Received channel qaulity report....... let's update..")
        self.msg = self.channel_report_message
        self.update_channel_quality_report(self.msg)
        self.channel_report_message = None
        print(f' Set None to channel report : {self.channel_report_message}')
        #self.freq = client.channel_report_message['freq']
        #self.quality_index = client.channel_report_message['qual']
        #self.freq_quality_report.update({self.freq: self.quality_index})
        self.fsm.trigger(ServerEvent.CHANNEL_QUALITY_UPDATE_COMPLETE)
                     
    def update_channel_quality_report(self, message) -> None:
        print("Updating channel report..... ")
        self.freq = message['freq']
        self.quality_index = message['qual']
        self.phy_error = message['phy_error']
        self.tx_rate = message['tx_rate']
        self.tx_timeout = message['tx_timeout']
        print(f'Sender data : tx_rate:{self.tx_rate} phy_error:{self.phy_error} tx_timeoout:{self.tx_timeout}')
        self.freq_quality_report.update({self.freq: self.quality_index})
        print(f" Updated FH seq ++1: {self.freq_quality_report}")
        self.update_db(('rf_signals',self.freq,0,0,0,self.quality_index))
        
    def update_db(self, args) -> None:
        if args[0] == 'rf_signals':
           cur_freq  = get_mesh_freq(self.nw_interface)
           label = 0
           if (cur_freq == args[1]):
              label = args[5]
           formatted_strings = [f"{s}" for s in args]
           # Join the formatted strings with a space separator
           result = ' '.join(formatted_strings)
           run_cmd = f"python /root/send_data2.py {formatted_strings} {label}"
        else:
            formatted_strings = [f"{s}" for s in args]
            # Join the formatted strings with a space separator
            result = ' '.join(formatted_strings)
            run_cmd = f"python /root/send_data2.py {formatted_strings}"
        try:
            result = subprocess.run(run_cmd, 
                                shell=True, 
                                capture_output=True, 
                                text=True)
            if(result.returncode != 0):
                print(f"Failed to execute the command: {run_cmd}")
                return None

        except subprocess.CalledProcessError as e:
            print(f"Error: {e}")
            return None
            

    def send_data_clients(self, data) -> None:
        """
        Sends the estimated frequency quality data to a remote server.

        :param data: The message to send to the clients.
        """
        try:
            serialized_data = msgpack.packb(data)
            netstring_data = encode(serialized_data)
            self.socket.sendto(netstring_data, (MULTICAST_GROUP, MULTICAST_PORT))  
            print(f" Send report to Mutlicast")
        except BrokenPipeError:
            logger.info(f"Broken pipe error")
        except Exception as e:
            logger.info(f"Error sending data to Multicast")
            print(f"Error sending data : {e}")

    def broadcast_operating_freq(self, trigger_event) -> None:
        """
        Broadcast Operating frequency to all clients.

        :param trigger_event: ServerEvent that triggered the execution of this function.
        """
        try:
            action_id: int = action_to_id["operating_frequency"]
            freq = get_mesh_freq(self.nw_interface)
            operating_freq_data = {'a_id': action_id, 'freq': freq}
            print(f"Brodcasting operating freq : {operating_freq_data}")
            #self.send_data_clients(operating_freq_data)
            send_data(self.socket, operating_freq_data)
            self.fsm.trigger(ServerEvent.BROADCAST_COMPLETE)
        except Exception as e:
            logger.info(f"Broadcast operating frequency error: {e}")
    
    # Adaptive frequency hopping with dynamic intervals and exit condition based on top frequency stability
    
    def partial_frequency_hopping(self, trigger_event) -> None:
        
        try:
            self.sorted_frequencies = sorted(self.freq_quality_report.items(), key=lambda x: x[1])
            self.top_freq = self.sorted_frequencies[0][0]
            self.seq_limit = min(self.seq_limit, len(self.sorted_frequencies))
            print(f" the cur state is  {self.fsm.state}")
            if (self.fsm.state == ServerState.PARTIAL_FREQUENCY_HOPPING):
                self.update_db(('pfh_monitor','1'))
                if self.top_freq_stability_counter < self.stability_threshold:
                    # Continue hopping between the best frequencies
                    self.switch_freq = self.sorted_frequencies[self.pfh_index][0]
                    print(f"******PFH :  count : {self.top_freq_stability_counter}")
                    #self.top_freq_stability_counter +=1

                    # Wait for the current hop interval before switching to the next frequency
                    time.sleep(self.hop_interval)
                    # Move to the next frequency in the list
                    self.pfh_index = (self.pfh_index + 1) % self.seq_limit
                    with self.lock:
                        # Sort by the second item in the tuple (quality) in ascending order (best quality first)
                        self.sorted_frequencies = sorted(self.freq_quality_report.items(), key=lambda x: x[1])
                        # Select the hop frequency sequence from the top x sorted frequencies
                        #hop_freq_seq = islice(sorted_frequencies,seq_limit)
                        current_top_freq = self.sorted_frequencies[0][0]
                    #print(f" the current top freq : {current_top_freq}")
                    if current_top_freq == self.top_freq:
                        self.top_freq_stability_counter += 1
                        print(f"Top frequency {self.top_freq} remained the same for {self.top_freq_stability_counter} consecutive checks.")
                    else:
                        self.top_freq_stability_counter = 0  # Reset counter if top frequency changes
                        self.top_freq = current_top_freq  # Update the top frequency
                        print(f"Top frequency changed to {self.top_freq}.")

                    # Exit if the top frequency stays the same for the stability threshold period
                    if self.top_freq_stability_counter == self.stability_threshold:
                        print(f"Top frequency {self.top_freq} has been stable for {self.stability_threshold} consecutive checks. Exiting.")
                        self.update_db(('pfh_monitor','0'))
                        self.update_db(('bcqi_monitor','0'))
                        #self.switch_frequency(self.top_freq)
                        self.operating_frequency = self.top_freq
                        self.switch_freq = self.top_freq
                        #self.limit = 0
                        #self.sorted_frequencies = []
                        #self.top_freq = 0
                    
                    print(f" Switch freq **: {self.switch_freq}")
                    result = self.switch_frequency(self.switch_freq, self.nw_interface, self.channel_bandwidth, self.beacons_count)
                    if result:
                        print("Waiting for CSA to be established")
                        time.sleep(self.beacons_count + self.buffer_period)
                    if get_mesh_freq(self.nw_interface) == self.switch_freq:
                       print(f" CSA is successfull, node switched to new operating freq : {get_mesh_freq(self.nw_interface)}")
                    else:
                       print(f" CSA is not successfull, cur operating freq : {get_mesh_freq(self.nw_interface)}")
                    self.fsm.trigger(ServerEvent.CHANNEL_SWITCH_REQUEST)

                if self.top_freq_stability_counter >= self.stability_threshold:
                    print(f"Completed partial frequency hopping: count: {self.top_freq_stability_counter}")
                    self.top_freq_stability_counter = 0
                    self.fsm.trigger(ServerEvent.FREQUENCY_HOPPING_COMPLETE)
                
        except Exception as e:
            print(f" PFH error {e}")

    def switch_frequency(self, frequency: int, interface: str, bandwidth: int, beacons_count: int ) -> bool:
        cur_freq = get_mesh_freq(interface)
        print(f" The cur freq** is {cur_freq}, freq : {frequency}")
        if cur_freq == frequency:
            print(f"Mesh node is currently operating at requested freq:{cur_freq} switch")
            return False 
        run_cmd = f"iw dev {interface} switch freq {frequency} HT{bandwidth} beacons {beacons_count}"
        print(f"run_cmd **: {run_cmd}")
        try:
            result = subprocess.run(run_cmd, 
                                shell=True, 
                                capture_output=True, 
                                text=True)
            if(result.returncode != 0):
                print("Failed to execute the switch frequency command")
                return False
            else:
                print("Channel Switch cmd run successfully!")
                self.update_db(('rf_signals',frequency,0,0,0,self.freq_quality_report.get(frequency)))
                return True

        except subprocess.CalledProcessError as e:
            print(f"Error: {e}")
            return False


    def send_switch_frequency_message(self, trigger_event) -> None:
        """
        Sends a message to all connected clients to switch to operating frequency.

        :param trigger_event: ServerEvent that triggered the execution of this function.
        """
        print('Sending channel switch request...')
        action_id = action_to_id["switch_frequency"]
        switch_id = str(uuid.uuid4())  # Generate a new ID at the end of the healing process
        switch_frequency_data = {'a_id': action_id, 'freq': self.switch_freq, 'switch_id': switch_id}
        print(f" The switch info is : {switch_frequency_data}")
        #self.send_data_clients(switch_frequency_data)
        send_data(self.socket, switch_frequency_data)
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

    def receive_messages(self) -> None:
        """
        Handles incoming messages from the client.
        """
        print(" Thread receive msg started.........")
        
        # A set to store the unique IDs of processed messages
        self.processed_ids = set()
        while self.running:
            try:
                # Receive incoming messages and decode the netstring encoded data
                try:
                    data = decode(self.socket.recv(1024))
                    if not data:
                        logger.info("No data... break")
                        print("No data... break")
                except Exception as e:
                    # Handle netstring decoding errors
                    logger.error(f"Failed to decode netstring: {e}")
                    print("Failed to decode netstring")
                    break

                # Deserialize the MessagePack message
                try:
                    unpacked_data = msgpack.unpackb(data, raw=False)
                    
                    message_id: str = unpacked_data.get("message_id")
                    if message_id in self.processed_ids:
                        print(f"Message with ID {message_id} has already been processed. Ignoring.")
                    else:
                        print(f"Processing message: {message_id}")
                        # Add the unique ID to the processed set
                        self.processed_ids.add(message_id)
                        action_id: int = unpacked_data.get("a_id")
                        action_str: str = id_to_action.get(action_id)
                        #logger.info(f"Received message: {unpacked_data}")
                        print(f"*****************Received message:{action_str}")

                        # Bad channel quality index received from client
                        if action_str == "bad_channel_quality_index":
                            self.bad_channel_message = unpacked_data

                        # Channel report received from client
                        elif action_str == "channel_quality_report":
                            self.channel_report_message = unpacked_data
                            print(f"the report is {self.channel_report_message}")
                except msgpack.UnpackException as e:
                    logger.error(f"Failed to decode MessagePack: {e}")
                    print(f"Failed to decode MessagePack: {e}")
                    continue

                except Exception as e:
                    logger.error(f"Error in received message: {e}")
                    print(f"Error in received message: {e}")
                    continue

            except ConnectionResetError:
                logger.error("Connection forcibly closed by the remote host")
                print("Connection forcibly closed by the remote host")
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
    #host: str = args.rmacs_osf_orchestrator
    #host: str = 'fdd8:84dc:fe30:7bf2:9cc3:f3ff:fe7a:8109'
    #host: str = get_ipv6_addr('br-lan')
    
    #port: int = args.port
    server: RMACSServer = RMACSServer()
    server.start()


if __name__ == "__main__":
    main()