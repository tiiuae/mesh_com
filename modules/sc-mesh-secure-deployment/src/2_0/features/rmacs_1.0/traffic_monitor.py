import os 
import time

class TrafficMonitor:
    '''
    A class designed to monitor network traffic and transmission errors by capturing network interface statistics from sysfs.
    
    Methods:
    traffic_monitor: Monitor the Network traffic at specific Network Interface.
    error_monitor: Monitor the Transmission error at specific Network Interface.
    get_traffic_status: Calculate the Network traffic based on previous and current tx_bytes 
    read_sysfs_file: Read the network interface statistics from sysfs.

    '''
    def __init__(self, interface: str):
        self.prev_tx_bytes = None
        self.cur_tx_bytes = None
        # Set the Network interface  
        self.nw_interface =  interface
        #Network statistics file 
        self.tx_bytes_path = f"/sys/class/net/{self.nw_interface}/statistics/tx_bytes"
        self.tx_error_path = f"/sys/class/net/{self.nw_interface}/statistics/tx_errors"
        
    def traffic_monitor(self) -> int:
        '''
        Monitor the Network traffic at specific Network Interface.
        
        Returns: 
        int : Return the network traffic in bytes
        '''
        # Store the previous tx byte to prev_tx_bytes for  
        self.prev_tx_bytes = self.cur_tx_bytes
        self.cur_tx_bytes = self.read_sysfs_file(self.tx_bytes_path)
        self.traffic =  self.get_traffic_status()
        print(f"The traffic is : {self.traffic}")
        return self.traffic
               
    def error_monitor(self) -> int:
        '''
        Monitor the Transmission error at specific Network Interface.
        
        Returns: 
        int : Return the network traffic error in bytes
        '''
        self.tx_error = self.read_sysfs_file(self.tx_error_path)
        print(f"The traffic error is : {self.tx_error}")
        return self.tx_error
    
    def get_traffic_status(self) -> int:
        '''
        Calculate the Network traffic based on previous and current tx_bytes 
        
        Returns: 
        int : Return the network traffic in bytes
        '''
        if self.prev_tx_bytes is None:
            self.prev_tx_bytes = self.cur_tx_bytes
            time.sleep(2) 
            self.cur_tx_bytes = self.read_sysfs_file(self.tx_bytes_path) 
        return (self.cur_tx_bytes - self.prev_tx_bytes)
    
    def read_sysfs_file(self, syspath: str) -> int:
        if os.path.exists(syspath):
            with open(syspath, 'r') as file:
                return int(file.read().strip())
        else:
            raise FileNotFoundError(f"{syspath} does not exist.")
    
    

    


    