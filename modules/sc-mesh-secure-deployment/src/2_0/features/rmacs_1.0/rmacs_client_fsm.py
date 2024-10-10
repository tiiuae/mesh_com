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

import msgpack
import numpy as np
import pandas as pd
from netstring import encode, decode

from logging_config import logger
from config import Config
#from rmacs_setup import get_ipv6_addr
from traffic_monitor import TrafficMonitor
from rmacs_setup import get_mesh_freq, get_ipv6_addr
from spectral_scan import Spectral_Scan
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

class ClientState(Enum):
    IDLE = auto()
    MONITOR_TRAFFIC = auto()
    MONITOR_ERROR = auto()
    CHANNEL_SCAN = auto()
    OPERATING_CHANNEL_SCAN = auto()
    REPORT_BCQI = auto()
    REPORT_CHANNEL_QUALITY = auto()
    CHANNEL_SWITCH = auto()
    #OFF_BAND_SCAN = auto()
    #NO_INTERFERENCE_DETECTED = auto()
      
    
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
    #SENT_GOOD_CHANNEL_QUALITY_INDEX = auto()
    
    EXT_SWITCH_EVENT = auto()
    #EXT_DATA_REQUEST = auto()

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
        self.args = Config
        self.state = ClientState.IDLE
        self.client = client
        self.event_queue = UniqueDeque()

        # Transition table
        self.transitions = {
            (ClientState.IDLE, ClientEvent.TRAFFIC_MONITOR): (ClientState.MONITOR_TRAFFIC, self.client.traffic_monitoring),
            (ClientState.MONITOR_TRAFFIC, ClientEvent.TRAFFIC): (ClientState.MONITOR_ERROR, self.client.error_monitoring),
            (ClientState.MONITOR_TRAFFIC, ClientEvent.NO_TRAFFIC): (ClientState.CHANNEL_SCAN, self.client.channel_scan),
            (ClientState.MONITOR_ERROR, ClientEvent.ERROR): (ClientState.OPERATING_CHANNEL_SCAN, self.client.channel_scan),
            (ClientState.MONITOR_ERROR, ClientEvent.NO_ERROR): (ClientState.MONITOR_TRAFFIC, self.client.traffic_monitoring),
            (ClientState.OPERATING_CHANNEL_SCAN, ClientEvent.GOOD_CHANNEL_QUALITY_INDEX): (ClientState.MONITOR_TRAFFIC, self.client.traffic_monitoring),
            #(ClientState.OPERATING_CHANNEL_SCAN, ClientEvent.BAD_CHANNEL_QUALITY_INDEX): (ClientState.IDLE, self.client.run_client_fsm),
            (ClientState.OPERATING_CHANNEL_SCAN, ClientEvent.BAD_CHANNEL_QUALITY_INDEX): (ClientState.REPORT_BCQI, self.client.sending_bad_channel_quality_index),
            #(ClientState.REPORT_BCQI, ClientEvent.SENT_BAD_CHANNEL_QUALITY_INDEX): (ClientState.IDLE, self.client.run_client_fsm),
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
            print(f"EXT_SWITCH_EVENT detected in state '{self.state}', handling globally.")
            # Handle EXT_SWITCH_EVENT (e.g., switching channels)
            self.client.switch_frequency(event)
            #self.client.handle_ext_switch_event(event)
            return  # Ensure no further processing of this event happens
        key = (self.state, event)
        if key in self.transitions:
            next_state, action = self.transitions[key]
            print(f'{self.state} -> {next_state}')
            self.state = next_state
            if action:
                action(event)
        else:
            print(f"No transition found for event '{event}' in state '{self.state}'")
            
    
    
class InterferenceDetection(threading.Thread):
    '''
    A class to detect interference in the Radio channels
    
    Methods:
    __init__ :
    run :
    stop:
    traffic_monitoring:
    '''
    def __init__(self, node_id: str) -> None:
        super().__init__()
        self.node_id = node_id
        #self.host = host
        #self.port = port

        # Initialize client objects
        self.fsm = ClientFSM(self)
        self.args = Config()
        self.traffic_monitor = TrafficMonitor()
        self.channel_bandwidth = self.args.channel_bandwidth
        self.beacons_count = self.args.beacons_count
        self.nw_interface = self.args.nw_interface
        self.switching_frequency = self.args.starting_frequency
        self.freq_list = self.args.freq_list
        self.freq_index = -1
        
        # Initialize the Scanning Object
        self.scan = Spectral_Scan()
        
        # Channel Quality index
        self.channel_quality_index = 0
        
        # Error Monitoring threshold
        self.phy_error_limit = self.args.phy_error_limit
        self.tx_timeout_limit = self.args.tx_timeout_limit
        
        # 
        self.phy_error = 0   
        self.tx_timeout = 0   

        ## Internal variables
        #self.socket = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
        #self.current_frequency: int = get_mesh_freq(self.nw_interface)
        #self.healing_process_id: str = None
        #self.operating_frequency: int = np.nan
        #self.best_freq: int = np.nan
        #self.freq_quality: dict = {}
        self.num_retries = 0
        self.max_retries = 3
        self.max_error_check = self.args.max_error_check
#
        ## Time for periodic events' variables (in seconds)
        #self.time_last_scan: float = 0
        #self.time_last_switch: float = 0
#
        ## Create listen and client run FSM threads
        self.running = False
        self.listen_thread = threading.Thread(target=self.receive_messages)
        self.run_client_fsm_thread = threading.Thread(target=self.run_client_fsm)
        
    def run(self) -> None:
        """
        Connect to the orchestrator node and start the client's operation in separate threads for running the FSM and receiving messages.
        """
        self.running = True
        self.connect_to_rmacs_comms()
        self.listen_thread.start()
        self.run_client_fsm_thread.start()
        #asyncio.run(self.run_client_fsm()) 

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

    def run_client_fsm(self) -> None:
        """
        Run the client Finite State Machine (FSM), managing its state transitions and periodic tasks.
        """
        count = 0
        print('+++++++ Start FSM ++++++++++++++')
        while self.running:            
            try:
                print(f"Current FSM state: {self.fsm.state}")
                if self.fsm.state == ClientState.IDLE:
                    current_time = time.time()
                    count +=1
                    print(f'+++++++++ From run_client fsm : current time : {current_time} : count : {count}')
                    self.fsm.trigger(ClientEvent.TRAFFIC_MONITOR)
                # Sleep for a short duration before checking conditions again
                time.sleep(30)
            except Exception as e:
                print(f"Exception in run: {e}")
               
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
                print(f"Failed to execute the command: {run_cmd}")
                return None

        except subprocess.CalledProcessError as e:
            print(f"Error: {e}")
            return None
                    
    def receive_messages(self) -> None:
        """
        Receive incoming messages from the orchestrator.
        """
        print(" Im receviing msg from client is running ")
        while self.running:
            try:
                # Receive incoming messages and decode the netstring encoded data
                try:
                    data = decode(self.socket.recv(1024))
                    if not data:
                        logger.info("No data...")
                        print("************No data...")
                        break
                except Exception as e:
                    # Handle netstring decoding errors
                    logger.error(f"Failed to decode netstring: {e}")
                    print(f"*************Failed to decode netstring: {e}")
                    break

                # Deserialize the MessagePack message
                try:
                    unpacked_data = msgpack.unpackb(data, raw=False)
                    action_id: int = unpacked_data.get("a_id")
                    action_str: str = id_to_action.get(action_id)
                    logger.info(f"Received message: {unpacked_data}")
                    print(f"************Received message: {unpacked_data}")


                    # Handle frequency switch request
                    if action_str in ["switch_frequency", "operating_frequency"]:
                        received_operating_freq = unpacked_data.get("freq")
                        self.update_operating_freq(received_operating_freq)
                        cur_freq = get_mesh_freq(self.nw_interface)
                        print(f"received op freq : {received_operating_freq}, op freq : {self.operating_frequency} : cur freq: {cur_freq}")
                        if cur_freq != self.operating_frequency:
                            self.switching_frequency = received_operating_freq
                            print(f" Handling action_str : {action_str}")
                            self.fsm.trigger(ClientEvent.EXT_SWITCH_EVENT)

                    # Recalibrate to server frequency
                    #elif action_str == "operating_frequency":
                    #    received_operating_freq = unpacked_data.get("freq")
                    #    self.update_operating_freq(received_operating_freq)
                    #    if self.current_frequency != self.operating_frequency:
                    #        self.fsm.trigger(ClientEvent.EXT_SWITCH_EVENT)

                except msgpack.UnpackException as e:
                    logger.error(f"Failed to decode MessagePack: {e}")
                    print(f"***************Failed to decode MessagePack: {e}")
                    continue

                except Exception as e:
                    logger.error(f"Error in received message: {e}")
                    print(f"**************Error in received message: {e}")
                    continue

            except ConnectionResetError:
                logger.error("Connection forcibly closed by the remote host")
                print("***************Connection forcibly closed by the remote host")
                break
    def sending_bad_channel_quality_index(self, trigger_event) -> None:
        """
        Send bad_channel_quality_index to orchestrator to report bad channel quality.

        :param trigger_event: ClientEvent that triggered the execution of this function.
        """
        curr_freq: int = get_mesh_freq(self.nw_interface)
        action_id: int = action_to_id["bad_channel_quality_index"]
        message_id: str = str(uuid.uuid4())  
        data = {'a_id': action_id, 'n_id': self.node_id, 'message_id': message_id,
                'freq': curr_freq, 'qual': self.channel_quality_index, 'tx_rate': self.traffic_rate,'phy_error' : self.phy_error, 'tx_timeout' : self.tx_timeout }
        #self.send_messages(data)
        repeat = 2
        while repeat:
            send_data(self.socket, data)
            repeat -= 1                  
        self.update_db(("bcqi_monitor", curr_freq))
        self.fsm.trigger(ClientEvent.SENT_BAD_CHANNEL_QUALITY_INDEX)
        
    def report_channel_quality(self, trigger_event) -> None:
        """
        Report channel quality to orchestrator if data is valid.

        :param trigger_event: ClientEvent that triggered the execution of this function.
        """
        # Send scan data
        logger.info("Reporting channel Quality...")
        action_id: int = action_to_id["channel_quality_report"]
        message_id: str = str(uuid.uuid4()) 
        data = {'a_id': action_id, 'n_id': self.node_id, 'freq': self.scan_freq, 'qual': self.channel_quality_index, 'tx_rate': self.traffic_rate,'phy_error' : self.phy_error, 'tx_timeout' : self.tx_timeout,'message_id': message_id}
        #self.send_messages(data)
        send_data(self.socket, data)
        self.fsm.trigger(ClientEvent.REPORTED_CHANNEL_QUALITY)

    def send_messages(self, data) -> None:
        """
        Sends the channel quality data to a remote server.

        :param data: The message to send to orchestrator.
        """
        try:
            print("sent msg to orchestra node")
            serialized_data = msgpack.packb(data)
            netstring_data = encode(serialized_data)
            self.socket.sendto(netstring_data, (MULTICAST_GROUP, MULTICAST_PORT))  
            #self.socket.send(netstring_data)
            print(f"Sent {data}")
        except ConnectionRefusedError:
            logger.error("Connection refused by the server. Check if the server is running and reachable.")
    
    def switch_frequency(self, trigger_event) -> bool:
        
        
        self.fsm.state = ClientState.CHANNEL_SWITCH
        cur_freq = get_mesh_freq(self.nw_interface)
        print(f" The cur freq** is {cur_freq}, freq : {self.switching_frequency}")
        if cur_freq == self.switching_frequency:
            print(f"Mesh node is currently operating at requested freq:{cur_freq} switch")
            self.fsm.trigger(ClientEvent.SWITCH_NOT_REQUIRED)
            return False 
        run_cmd = f"iw dev {self.nw_interface} switch freq {self.switching_frequency} HT{self.channel_bandwidth} beacons {self.beacons_count}"
        print(f"run_cmd : {run_cmd}")
        try:
            result = subprocess.run(run_cmd, 
                                shell=True, 
                                capture_output=True, 
                                text=True)
            if(result.returncode != 0):
                #logger.error("Failed to execute the switch frequency command")
                print("Failed to execute the switch frequency command")
                #self.fsm.trigger(ClientEvent.SWITCH_UNSUCCESSFUL)
            else:
                print(f"Executed switch freq cmd successfully : ")
            time.sleep(self.beacons_count)
            cur_freq = get_mesh_freq(self.nw_interface)
            print(f"the current freq is {cur_freq}")
        
             # If maximum frequency switch retries not reached, try to switch again
            if cur_freq != self.operating_frequency and self.num_retries < self.max_retries:
                logger.info(f"Frequency switch unsuccessful, retry {self.num_retries}")
                self.num_retries += 1
                self.fsm.trigger(ClientEvent.SWITCH_UNSUCCESSFUL)
                
             # Frequency switch successful
            elif cur_freq == self.operating_frequency:
                print("Frequency switch successful")
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
             
    def update_operating_freq(self, received_operating_freq):
        """
        Update the operating mesh frequency.

        :param received_operating_freq: The new operating frequency to be set.
        """
        self.operating_frequency = received_operating_freq
        
    def channel_scan(self, trigger_event) -> None:
        
        if self.fsm.state == ClientState.CHANNEL_SCAN:
            self.freq_index = (self.freq_index + 1) % len(self.freq_list)
            self.scan_freq = self.freq_list[self.freq_index]
            self.channel_report: list[dict] = self.perform_scan(self.scan_freq)
            self.channel_quality_index = self.channel_quality_estimator(self.channel_report)
            print(f" Performed channel scan : freq : {self.scan_freq} : report : {self.channel_quality_index}")
            self.fsm.trigger(ClientEvent.PERFORMED_CHANNEL_SCAN)
            
            
        elif self.fsm.state == ClientState.OPERATING_CHANNEL_SCAN:
            self.scan_freq = get_mesh_freq(self.nw_interface)
            self.channel_report = self.perform_scan(self.scan_freq)
            #print(f"the channel report is : {self.channel_report}")
            self.channel_quality_index = self.channel_quality_estimator(self.channel_report)
            #print(f"the channel quality index is : {self.channel_quality_index}")
            if self.channel_quality_index > self.args.channel_quality_index_threshold:
                print("Trigger Bad Qaulity index")
                self.fsm.trigger(ClientEvent.BAD_CHANNEL_QUALITY_INDEX)
                #self.send_messages(self.channel_report)
                #self.fsm.trigger(ClientEvent.BAD_CHANNEL_QUALITY_INDEX)
                #print(f"Current FSM state: {self.fsm.state}")
            else :
                print("Trigger Good Qaulity index")
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
                print(f"An error occurred during the channel scan process : {item['error']}")
                return None
    
    def read_dataframe(self, bin_file_name:str):
        '''
         /usr/bin/fft_eval_json <bin path>/samples_$freq.bin $freq <data_path>
         freq: scan freq
         <data_path> : result file
        '''
        pass
                
    def traffic_monitoring(self, trigger_event) -> None:
        '''
        Perform traffic monitor 
        '''
        print("Debug from traffic monitor trigger method")
        if self.fsm.state == ClientState.MONITOR_TRAFFIC:
            self.traffic_rate = self.traffic_monitor.traffic_monitor()
            if self.traffic_rate:
                print(f"Debug: traffic rate : {self.traffic_rate}")
                self.fsm.trigger(ClientEvent.TRAFFIC)
            else:
                print("No traffic")
                self.fsm.trigger(ClientEvent.NO_TRAFFIC)
                
    def error_monitoring(self, trigger_event) -> None:
        
        self.error_check_count = 0
        self.monitoring = True
        while self.monitoring: 
            if self.fsm.state == ClientState.MONITOR_ERROR:
                print('Inside error moniotring')
                if self.error_check_count < self.max_error_check:
                    print(f'error check count : {self.error_check_count}')
                    self.phy_error = self.traffic_monitor.get_phy_error()
                    self.tx_timeout = self.traffic_monitor.get_tx_timeout()
                    if self.phy_error > self.phy_error_limit or self.tx_timeout > self.tx_timeout_limit:
                        self.error_check_count +=1
                        print(f"Observed error in on-going traffic : count = {self.error_check_count}")
                        continue
                    else:
                        logger.info("Observed no-error in on-going traffic")
                        self.monitoring = False
                        self.fsm.trigger(ClientEvent.NO_ERROR)
                elif self.error_check_count >= self.max_error_check:
                    print(f"Report error in on-going traffic with phy_error: {self.phy_error} and tx_timeout: {self.tx_timeout}")
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
            if current_time - self.last_time >= self.args.periodic_recovery_switch:
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
    args = Config()
    #host: str = get_ipv6_addr('br-lan')
    #port: int = args.port
    #node_id: str = get_ipv6_addr(args.osf_interface)
    node_id: str = 'e'
    client = InterferenceDetection(node_id)
    print('Im main')
    client.start()
    
    
if __name__ == '__main__':
    main()
     
    
    
