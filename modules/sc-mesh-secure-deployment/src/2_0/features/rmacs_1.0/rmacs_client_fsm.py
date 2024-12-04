import sys
import threading
import time
import asyncio
import socket
from enum import auto, Enum
from typing import List, Tuple, Dict
from collections import deque
import subprocess
import uuid
import json
import os

import msgpack
import numpy as np
import pandas as pd
from netstring import encode, decode

from logging_config import logger
from config import create_default_config, load_config
from traffic_monitor import TrafficMonitor
from rmacs_setup import get_mesh_freq, get_ipv6_addr, get_mac_address
from spectral_scan import Spectral_Scan
from rmacs_comms import rmacs_comms, send_data

config_file_path = '/etc/meshshield/rmacs_config.yaml'
CONFIG_DIR = "/etc/meshshield"

action_to_id = {
    "bad_channel_quality_index": 0,
    "channel_quality_report": 1,
    "operating_frequency": 2,
    "switch_frequency": 3
}
id_to_action = {v: k for k, v in action_to_id.items()}


class ClientState(Enum):
    IDLE = auto()
    MONITOR_TRAFFIC = auto()
    MONITOR_ERROR = auto()
    CHANNEL_SCAN = auto()
    OPERATING_CHANNEL_SCAN = auto()
    REPORT_BCQI = auto()
    REPORT_CHANNEL_QUALITY = auto()
    CHANNEL_SWITCH = auto()
      
    
class ClientEvent(Enum):
    TRAFFIC_MONITOR = auto()
    ERROR_MONITOR = auto()
    TRAFFIC = auto()
    NO_TRAFFIC = auto()
    ERROR = auto()
    NO_ERROR = auto()
    PERFORM_CHANNEL_SCAN = auto()
    PERFORM_OPERATING_CHANNEL_SCAN = auto()
    PERFORMED_CHANNEL_SCAN = auto()
    REPORTED_CHANNEL_QUALITY = auto()
    BAD_CHANNEL_QUALITY_INDEX = auto()
    GOOD_CHANNEL_QUALITY_INDEX = auto()
    SENT_BAD_CHANNEL_QUALITY_INDEX = auto()
    SWITCH_NOT_REQUIRED = auto
    SWITCH_SUCCESSFUL = auto
    SWITCH_UNSUCCESSFUL = auto
    EXT_SWITCH_EVENT = auto()

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
        self.state = ClientState.IDLE
        self.client = client
        self.event_queue = UniqueDeque()

        # Transition table
        self.transitions = {
            (ClientState.IDLE, ClientEvent.TRAFFIC_MONITOR): (ClientState.MONITOR_TRAFFIC, self.client.traffic_monitoring),
            (ClientState.MONITOR_TRAFFIC, ClientEvent.TRAFFIC): (ClientState.MONITOR_ERROR, self.client.error_monitoring),
            (ClientState.MONITOR_TRAFFIC, ClientEvent.NO_TRAFFIC): (ClientState.CHANNEL_SCAN, self.client.channel_scan),
            (ClientState.MONITOR_ERROR, ClientEvent.ERROR): (ClientState.OPERATING_CHANNEL_SCAN, self.client.channel_scan),
            (ClientState.MONITOR_ERROR, ClientEvent.NO_ERROR): (ClientState.IDLE, None),
            (ClientState.OPERATING_CHANNEL_SCAN, ClientEvent.GOOD_CHANNEL_QUALITY_INDEX): (ClientState.MONITOR_TRAFFIC, self.client.traffic_monitoring),
            (ClientState.OPERATING_CHANNEL_SCAN, ClientEvent.BAD_CHANNEL_QUALITY_INDEX): (ClientState.REPORT_BCQI, self.client.sending_bad_channel_quality_index),
            (ClientState.REPORT_BCQI, ClientEvent.SENT_BAD_CHANNEL_QUALITY_INDEX): (ClientState.IDLE, None),
            (ClientState.CHANNEL_SCAN, ClientEvent.PERFORMED_CHANNEL_SCAN): (ClientState.REPORT_CHANNEL_QUALITY, self.client.report_channel_quality),
            (ClientState.REPORT_CHANNEL_QUALITY, ClientEvent.REPORTED_CHANNEL_QUALITY): (ClientState.IDLE, None),
            (ClientState.CHANNEL_SWITCH, ClientEvent.SWITCH_NOT_REQUIRED):(ClientState.IDLE, None),
            (ClientState.CHANNEL_SWITCH, ClientEvent.SWITCH_SUCCESSFUL):(ClientState.IDLE, None),
            (ClientState.CHANNEL_SWITCH, ClientEvent.SWITCH_UNSUCCESSFUL):(ClientState.IDLE, None)
        }

    def is_external_event(self, event: ClientEvent) -> bool:
        """
        Check if an event originated from a message received from the orchestrator.

        :param event: The event to check.
        :return: True if the event is an external event, False otherwise.
        """
        return event in [ClientEvent.EXT_SWITCH_EVENT]

    def trigger(self, event: ClientEvent) -> None:
        """Function to handle state transitions"""
        # If it's an external event, process it immediately 
        if self.is_external_event(event):
            self._process_event(event)
        else:
            self.event_queue.append(event)
            #if self.state != ClientState.IDLE:
            #    return

        # Process any internal queued events
        event_list = self.event_queue.pop_all()
        if event_list:
            for event in event_list:
                self._process_event(event)

    def _process_event(self, event: ClientEvent) -> None:
        """Internal function to process the given event"""
         # Handle EXT_SWITCH_EVENT globally, irrespective of current state
        if event == ClientEvent.EXT_SWITCH_EVENT:
            logger.info(f"EXT_SWITCH_EVENT detected in state '{self.state}', handling globally.")
            # Handle EXT_SWITCH_EVENT (e.g., switching channels)
            self.client.switch_frequency(event)
            #self.client.handle_ext_switch_event(event)
            return  # Ensure no further processing of this event happens
        key = (self.state, event)
        if key in self.transitions:
            next_state, action = self.transitions[key]
            logger.info(f'{self.state} -> {next_state}')
            self.state = next_state
            if action:
                action(event)
        else:
            logger.info(f"No transition found for event '{event}' in state '{self.state}'")
            
    
    
class InterferenceDetection(threading.Thread):
    '''
    A class to detect interference in the Radio channels
    
    Methods:
    __init__ :
    run :
    stop:
    traffic_monitoring:
    '''
    def __init__(self) -> None:
        super().__init__()
        # Initialize client objects
        self.fsm = ClientFSM(self)
        # Load the configuration
        config = load_config(config_file_path)
        self.traffic_monitor = TrafficMonitor()
        self.channel_bandwidth = config['RMACS_Config']['channel_bandwidth']
        print(f"the *********** channel bandwidth = {self.channel_bandwidth}")
        self.client_beacon_count = config['RMACS_Config']['client_beacon_count']
        self.nw_interface = config['RMACS_Config']['nw_interface']
        self.switching_frequency = config['RMACS_Config']['starting_frequency']
        self.freq_list = config['RMACS_Config']['freq_list']
        # Control channel interfaces
        self.ch_interfaces = config['RMACS_Config']['radio_interfaces']
        
        self.freq_index = -1
        self.sockets: Dict = {}
        self.listen_threads: list = []
        
        # Initialize the Scanning Object
        self.scan = Spectral_Scan()
        
        # Channel Quality index
        self.channel_quality_index_threshold = config['RMACS_Config']['channel_quality_index_threshold']
        
        # Error Monitoring threshold
        self.phy_error_limit = config['RMACS_Config']['phy_error_limit']
        self.tx_timeout_limit = config['RMACS_Config']['tx_timeout_limit']
        
        # Device MAC address
        self.mac_address = get_mac_address(self.nw_interface)
        
        # 
        self.phy_error = 0   
        self.tx_timeout = 0   

        self.num_retries = 0
        self.max_retries = 3
        self.max_error_check = config['RMACS_Config']['max_error_check']
        
        self.periodic_recovery_switch = config['RMACS_Config']['periodic_recovery_switch']

        ## Create listen and client run FSM threads
        self.running = False
        
        
        self.processed_ids = set()
        logger.info(f"The processed_id : {id(self.processed_ids)}")
        self.msg_id_lock = threading.Lock()
        
        self.run_client_fsm_thread = threading.Thread(target=self.run_client_fsm)
        
    def run(self) -> None:
        """
        Connect to the orchestrator node and start the client's operation in separate threads for running the FSM and receiving messages.
        """
        try:
            self.running = True
            for interface in self.ch_interfaces:
                try:
                    socket = rmacs_comms(interface)
                    self.sockets[interface] = socket
                    listen_thread = threading.Thread(target=self.receive_messages, args=(socket, interface))
                    self.listen_threads.append(listen_thread)
                    listen_thread.start()
                    logger.info(f"Listening on interface: {interface}")
                except ConnectionError as e:
                    logger.error(f"Connection error on {interface}: {e}")
            # Start the server FSM thread
            logger.info("Server started and listening...")
            
            self.run_client_fsm_thread.start()
        except Exception as e:
            logger.error(f"Unexpected error while starting server: {e}")
            self.stop()


    def run_client_fsm(self) -> None:
        """
        Run the client Finite State Machine (FSM), managing its state transitions and periodic tasks.
        """
        count = 0
        logger.info('RMACS client fsm is running....')
        while self.running:            
            try:
                logger.info(f"Current FSM state: {self.fsm.state}")
                if self.fsm.state == ClientState.IDLE:
                    current_time = time.time()
                    count +=1
                    self.fsm.trigger(ClientEvent.TRAFFIC_MONITOR)
                # Sleep for a short duration before checking conditions again
                time.sleep(5)
            except Exception as e:
                logger.info(f"Exception in run: {e}")
                return None
               
    def update_db(self, args) -> None:
        
        formatted_strings = [f"{s}" for s in args]
        # Join the formatted strings with a space separator
        result = ' '.join(formatted_strings)
        run_cmd = f"python /root/send_data2.py {result}"
        try:
            result = subprocess.run(run_cmd, 
                                shell=True, 
                                capture_output=True, 
                                text=True)
            if(result.returncode != 0):
                logger.info(f"Failed to execute the command: {run_cmd}")
                return None

        except subprocess.CalledProcessError as e:
            logger.info(f"Error: {e}")
            return None
                    
    def receive_messages(self, socket, interface) -> None:
        """
        Receive incoming messages from the orchestrator.
        """
        logger.info("Client receive msg thread is started.........")
        # A set to store the unique IDs of processed messages
        # self.processed_ids = set()
        # logger.info(f"The processed_id : {id(self.processed_ids)}")
        thread_id = threading.get_native_id()
        logger.info(f"The running thread id is : {thread_id}")
        while self.running:
            try:
                # Receive incoming messages and decode the netstring encoded data
                try:
                    data = decode(socket.recv(1024))
                    if not data:
                        logger.info("No data...")
                        break
                except Exception as e:
                    # Handle netstring decoding errors
                    logger.error(f"Failed to decode netstring: {e} via interface : {interface} ")
                    break

                # Deserialize the MessagePack message
                try:
                    unpacked_data = msgpack.unpackb(data, raw=False)
                    
                    message_id: str = unpacked_data.get("message_id")
                    with self.msg_id_lock:
                        if message_id in self.processed_ids:
                            logger.info(f"{thread_id}: Duplicate Msg : Message with ID {message_id} has already been processed and was received from interface : {interface}. Ignoring.")
                        else:
                            logger.info(f"{thread_id}: New Msg: Processing message: {message_id} : msg : {unpacked_data} via interface : {interface}")
                            # Add the unique ID to the processed set

                            #with self.msg_id_lock:
                            self.processed_ids.add(message_id)
                            action_id: int = unpacked_data.get("a_id")
                            action_str: str = id_to_action.get(action_id)
                            #logger.info(f"Received message: {unpacked_data} via interface : {interface}")


                            # Handle frequency switch request
                            if action_str in ["switch_frequency", "operating_frequency"]:
                                requested_switch_freq = unpacked_data.get("freq")
                                self.update_operating_freq(requested_switch_freq)
                                cur_freq = get_mesh_freq(self.nw_interface)
                                logger.info(f"The requested switch freq: {requested_switch_freq} and current operating freq: {cur_freq} via interface : {interface}")
                                if cur_freq != self.operating_frequency:
                                    self.switching_frequency = requested_switch_freq
                                    logger.info(f"Handling action_str : {action_str} via interface : {interface}")
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
    def sending_bad_channel_quality_index(self, trigger_event) -> None:
        """
        Send bad_channel_quality_index to orchestrator to report bad channel quality.

        :param trigger_event: ClientEvent that triggered the execution of this function.
        """
        curr_freq: int = get_mesh_freq(self.nw_interface)
        action_id: int = action_to_id["bad_channel_quality_index"]
        message_id: str = str(uuid.uuid4())  
        data = {'a_id': action_id,'message_id': message_id,
                'freq': curr_freq, 'qual': self.channel_quality_index, 'tx_rate': self.traffic_rate,'phy_error' : self.phy_error, 'tx_timeout' : self.tx_timeout,
                'device': self.mac_address}
        logger.info(f'Sending BCQI report to Multicast group: {data}')
        repeat = 2
        while repeat:
            # Loop through the sockets dictionary and send data
            for interface, socket in self.sockets.items():
                self.send_to_socket(socket, data, interface)
            repeat -= 1                  
        self.update_db(("bcqi_monitor", curr_freq))
        self.fsm.trigger(ClientEvent.SENT_BAD_CHANNEL_QUALITY_INDEX)
       
    def send_to_socket(self, socket, data, interface):
        try:
            send_data(socket, data, interface)
            logger.info(f"Successfully sent data to {interface}")
        except Exception as e:
            logger.error(f"Error sending data to {interface}: {e}")
    
    def report_channel_quality(self, trigger_event) -> None:
        """
        Report channel quality to orchestrator if data is valid.

        :param trigger_event: ClientEvent that triggered the execution of this function.
        """
        # Send scan data
        logger.info("Reporting channel Quality...")
        action_id: int = action_to_id["channel_quality_report"]
        message_id: str = str(uuid.uuid4()) 
        data = {'a_id': action_id,'freq': self.scan_freq, 'qual': self.channel_quality_index, 'tx_rate': self.traffic_rate,
                'phy_error' : self.phy_error, 'tx_timeout' : self.tx_timeout,'message_id': message_id, 'device': self.mac_address}
        logger.info(f'Sending Channel quality report to Multicast group: {data}')
        
        # Loop through the sockets dictionary and send data
        for interface, socket in self.sockets.items():
            self.send_to_socket(socket, data, interface)
        self.fsm.trigger(ClientEvent.REPORTED_CHANNEL_QUALITY)
    
    def switch_frequency(self, trigger_event) -> None:
          
        self.fsm.state = ClientState.CHANNEL_SWITCH
        cur_freq = get_mesh_freq(self.nw_interface)
        logger.info(f"The current operating frequency is {cur_freq} and requested switch frequency is {self.switching_frequency}")
        if cur_freq == self.switching_frequency:
            logger.info(f"Mesh node is currently operating at requested switch frequency:{cur_freq} already")
            self.fsm.trigger(ClientEvent.SWITCH_NOT_REQUIRED)
            return None 
        run_cmd = f"iw dev {self.nw_interface} switch freq {self.switching_frequency} HT{self.channel_bandwidth} beacons {self.client_beacon_count}"
        logger.info(f"run_cmd : {run_cmd}")
        try:
            result = subprocess.run(run_cmd, 
                                shell=True, 
                                capture_output=True, 
                                text=True)
            if(result.returncode != 0):
                logger.info("Failed to execute the switch frequency command")
                return None
            else:
                logger.info(f"Executed switch freq cmd successfully : ")
                time.sleep(self.client_beacon_count)
                cur_freq = get_mesh_freq(self.nw_interface)
               # logger.info(f"After successful frequency switch  {cur_freq}")
        
             # If maximum frequency switch retries not reached, try to switch again
            if cur_freq != self.switching_frequency and self.num_retries < self.max_retries:
                logger.info(f"Frequency switch is unsuccessful, retry {self.num_retries}")
                self.num_retries += 1
                self.fsm.trigger(ClientEvent.SWITCH_UNSUCCESSFUL)
                
             # Frequency switch successful
            elif cur_freq == self.switching_frequency:
                logger.info(f"Frequency switch is successful, Operating frequency : {cur_freq} and requested switch frequency : {self.switching_frequency} both are same")
                self.num_retries = 0
                self.fsm.trigger(ClientEvent.SWITCH_SUCCESSFUL)
            '''

            # If max frequency switch retries reached, and mesh frequency is NaN, mesh failed, exit rmacs module
            elif self.num_retries == self.max_retries and np.isnan(self.current_frequency):
                logger.info("Mesh failed... exiting rmacs module")
                #kill_process_by_pid("rmacs_setup.py")
            
            
            # If max frequency switch retries reached, and mesh switched to a different frequency, continue scheme on current frequency
            elif self.current_frequency != self.operating_frequency and self.num_retries == self.max_retries:
                logger.info("Switched to different channel... continue")
                self.num_retries = 0
                self.fsm.trigger(ClientEvent.SWITCH_SUCCESSFUL)
                
            '''
            
        except subprocess.CalledProcessError as e:
            logger.error(f"Switching frequency error occurred: {str(e)}")
            self.fsm.trigger(ClientEvent.SWITCH_UNSUCCESSFUL)  
             
    def update_operating_freq(self, requested_switch_freq):
        """
        Update the operating mesh frequency.

        :param requested_switch_freq: The new operating frequency to be set.
        """
        self.operating_frequency = requested_switch_freq
        
    def channel_scan(self, trigger_event) -> None:
        
        if self.fsm.state == ClientState.CHANNEL_SCAN:
            self.freq_index = (self.freq_index + 1) % len(self.freq_list)
            self.scan_freq = self.freq_list[self.freq_index]
            self.channel_report: list[dict] = self.perform_scan(self.scan_freq)
            self.channel_quality_index = self.channel_quality_estimator(self.channel_report)
            logger.info(f"Performed channel scan at freq : {self.scan_freq} and its channel quality index : {self.channel_quality_index}")
            self.fsm.trigger(ClientEvent.PERFORMED_CHANNEL_SCAN)
            
            
        elif self.fsm.state == ClientState.OPERATING_CHANNEL_SCAN:
            self.scan_freq = get_mesh_freq(self.nw_interface)
            self.channel_report = self.perform_scan(self.scan_freq)
            self.channel_quality_index = self.channel_quality_estimator(self.channel_report)
            if self.channel_quality_index > self.channel_quality_index_threshold:
                logger.info("Trigger Bad Channel Qaulity index")
                self.fsm.trigger(ClientEvent.BAD_CHANNEL_QUALITY_INDEX)
                #self.send_messages(self.channel_report)
                #self.fsm.trigger(ClientEvent.BAD_CHANNEL_QUALITY_INDEX)
                #logger.info(f"Current FSM state: {self.fsm.state}")
            else :
                logger.info("Trigger Good Channel Qaulity index")
                self.fsm.trigger(ClientEvent.GOOD_CHANNEL_QUALITY_INDEX)
            
            
    def perform_scan(self, freq: str) -> list[dict]:
        self.scan.initialize_scan()
        self.scan.execute_scan(self.scan_freq)
        self.channel_quality:list[dict] = self.scan.run_fft_eval(self.scan_freq)
        return self.channel_quality
        
        
    def channel_quality_estimator(self,channel_qaulity_report: list[dict]) -> int:
        
        self.report = json.loads(channel_qaulity_report)

        for item in self.report:
            if "index" in item:
                index_value = item["index"]
                return index_value
            elif "error" in item:
                logger.info(f"An error occurred during the channel scan process : {item['error']}")
                return None

                
    def traffic_monitoring(self, trigger_event) -> None:
        '''
        Perform traffic monitor 
        '''
        if self.fsm.state == ClientState.MONITOR_TRAFFIC:
            self.traffic_rate = self.traffic_monitor.traffic_monitor()
            if self.traffic_rate:
                logger.info(f"Traffic rate : {self.traffic_rate}")
                self.fsm.trigger(ClientEvent.TRAFFIC)
            else:
                logger.info("No traffic")
                self.fsm.trigger(ClientEvent.NO_TRAFFIC)
                
    def error_monitoring(self, trigger_event) -> None:
        
        self.error_check_count = 0
        self.monitoring = True
        while self.monitoring: 
            if self.fsm.state == ClientState.MONITOR_ERROR:
                if self.error_check_count < self.max_error_check:
                    logger.info(f'Traffic error observed for {self.error_check_count} items')
                    self.phy_error = self.traffic_monitor.get_phy_error()
                    self.tx_timeout = self.traffic_monitor.get_tx_timeout()
                    if self.phy_error > self.phy_error_limit or self.tx_timeout > self.tx_timeout_limit:
                        self.error_check_count +=1
                        logger.info(f"Observed error in on-going traffic : count = {self.error_check_count}")
                        continue
                    else:
                        logger.info("Observed no-error in on-going traffic")
                        self.monitoring = False
                        # self.fsm.state = ClientState.IDLE
                        # return
                        self.fsm.trigger(ClientEvent.NO_ERROR)
                elif self.error_check_count >= self.max_error_check:
                    logger.info(f"Report error in on-going traffic with phy_error: {self.phy_error} and tx_timeout: {self.tx_timeout}")
                    self.error_check_count = 0
                    self.monitoring = False
                    self.fsm.trigger(ClientEvent.ERROR)
                
    def recovering_switch_error(self, trigger_event) -> None:
        """
        Handle recovering from a switch error by periodically attempting frequency switching.

        :param trigger_event: The event that triggered the recovery process.
        """
        self.last_time = time.time()
        while self.running:
            current_time = time.time()
            # If periodic switch timer ended, switch frequency again
            if current_time - self.last_time >= self.periodic_recovery_switch:
                self.fsm.trigger(ClientEvent.PERIODIC_SWITCH)
                self.last_time = current_time
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
    logger.info('RMACS client thread is started....')
    if not os.path.exists(CONFIG_DIR):
        os.makedirs(CONFIG_DIR, exist_ok=True)
        logger.info(f"Created configuration directory: {CONFIG_DIR}")


    # Create the default configuration file if it doesn't exist
    create_default_config(config_file_path)

    # Load the configuration
    #config = load_config(config_file_path)
    client = InterferenceDetection()
    client.start()
    
    
if __name__ == '__main__':
    main()
     
    
    
