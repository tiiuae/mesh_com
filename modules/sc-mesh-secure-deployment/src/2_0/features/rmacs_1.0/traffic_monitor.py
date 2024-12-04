import os 
import time
import subprocess
from config import load_config
config_file_path = '/etc/meshshield/rmacs_config.yaml'

class TrafficMonitor:
    '''
    A class designed to monitor network traffic and transmission errors by capturing network interface statistics from sysfs.
    
    Methods:
    traffic_monitor: Monitor the Network traffic at specific Network Interface.
    error_monitor: Monitor the Transmission error at specific Network Interface.
    get_traffic_status: Calculate the Network traffic based on previous and current tx_bytes 
    read_sysfs_file: Read the network interface statistics from sysfs.

    '''
    def __init__(self):
        self.prev_tx_bytes = None
        self.cur_tx_bytes = None
        self.tx_rate_wait_time = 2
        self.phy_error_wait_time = 2
        self.tx_timeout_wait_time = 2
        # Set the Network interface  
        config = load_config(config_file_path)
        self.phy_interface = config['RMACS_Config']['phy_interface']
        self.nw_interface = config['RMACS_Config']['nw_interface']
        self.traffic_threshold =  config['RMACS_Config']['traffic_threshold']
        #Network statistics file 
        self.tx_bytes_path = f"/sys/class/net/{self.nw_interface}/statistics/tx_bytes"
        self.tx_error_path = f"/sys/class/net/{self.nw_interface}/statistics/tx_errors"
        self.fw_stats_path = f"/sys/kernel/debug/ieee80211/phy1/{self.nw_interface}/fw_stats"
        
        
        '''
        To get the phy_error : 
        /usr/sbin/ethtool -S wlp1s0|grep -i 'd_rx_phy_err:'|cut -f 2 -d :
        To get the tx_timeout :
        
        /usr/sbin/ethtool -S wlp1s0|grep -i 'd_tx_timeout:'|cut -f 2 -d : 
        
        '''
    def traffic_monitor(self) -> int:
        '''
        Monitor the Network traffic at specific Network Interface.
        
        Returns: 
        int : Return the network traffic in bytes
        '''  
        self.prev_tx_bytes = self.read_sysfs_file(self.tx_bytes_path)
        time.sleep(self.tx_timeout_wait_time)
        self.cur_tx_bytes = self.read_sysfs_file(self.tx_bytes_path)
        if self.prev_tx_bytes is not None and self.cur_tx_bytes is not None: 
            self.traffic = ((self.cur_tx_bytes - self.prev_tx_bytes) * 8)/ (self.tx_rate_wait_time * 1000) #kbps 
            print(f"Traffic : {self.traffic}")
            if self.traffic > self.traffic_threshold:
                print(f"Current Traffic: {self.traffic} in Kbps above threshold traffic : {self.traffic_threshold} in Kbps")
                return self.traffic
            else:
                print(f"There is no traffic, let's go for channel scan......")
                return 0
        else:
            return 0

               
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
            time.sleep(self.tx_rate_wait_time) 
            self.cur_tx_bytes = self.read_sysfs_file(self.tx_bytes_path) 
            # Convert tx_bytes value in bytes to Kilobits [Kilobits = Bytes * 8 / 1000]
        self.traffic_in_kbps = ((self.cur_tx_bytes - self.prev_tx_bytes) * 8)/ (self.tx_rate_wait_time * 1000) #kbps 
        print(f"Traffic : {self.traffic_in_kbps}")
        return self.traffic_in_kbps
    
    def get_phy_error(self) ->int:
        
        self.command = f"/usr/sbin/ethtool -S {self.nw_interface} | grep -i 'd_rx_phy_err:' | cut -f 2 -d :"
        self.prev_phy_error = self.run_command(self.command)
        time.sleep(self.phy_error_wait_time)
        self.cur_phy_error = self.run_command(self.command)
        if self.prev_phy_error is not None and self.cur_phy_error is not None:
            print(f"phy_error: {self.cur_phy_error - self.prev_phy_error}")
            return self.cur_phy_error - self.prev_phy_error
                
    def get_tx_timeout(self) ->int:
        
        self.command = f"/usr/sbin/ethtool -S {self.nw_interface} | grep -i 'd_tx_timeout:' | cut -f 2 -d :"
        self.prev_tx_timeout = self.run_command(self.command)
        time.sleep(self.tx_timeout_wait_time)
        self.cur_tx_timeout = self.run_command(self.command)
        if self.prev_tx_timeout is not None and self.cur_tx_timeout is not None:
            print(f"tx_timeout: {self.cur_tx_timeout - self.prev_tx_timeout}")
            return self.cur_tx_timeout - self.prev_tx_timeout
    
    def run_command(self, command: str) -> int:
        try:
            result = subprocess.run(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            if result.returncode == 0:
                output  = result.stdout.strip()
                return int(output)
            else:
                print(f"Command failed with return code {result.returncode}. Error:", result.stderr)
                return None
    
        except FileNotFoundError as e:
            print(f"Command not found: {e}")
            return None
        except subprocess.SubprocessError as e:
            print(f"Subprocess error: {e}")
            return None
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            return None
            
    
    def read_sysfs_file(self, syspath: str) -> int:
        '''
        Read the network interface statistics from sysfs.
        
        Returns: 
        int : Return the sysfs value 
        '''
        
        if os.path.exists(syspath):
            with open(syspath, 'r') as file:
                return int(file.read().strip())
        else:
            raise FileNotFoundError(f"{syspath} does not exist.")
        
        
def main():
    print('Main called ..........')
    obj = TrafficMonitor()
    phy_error = obj.get_phy_error()
    tx_timeout = obj.get_tx_timeout()
    print(f"phy error : {phy_error}, tx_timeout = {tx_timeout}")
    pass

if __name__ == "__main__":
    main()
        
    
    

    


    