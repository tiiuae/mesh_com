import os
import socket
import fcntl
import struct
from typing import List

VALID_CHANNELS = [36, 40, 44, 48, 149, 153, 157, 161]
# Multicast group and port 
MULTICAST_CONFIG = {
'wlp1s0': {'group': 'ff02::1', 'port': 12345},
'halow1': {'group':'ff02::1', 'port': 12346},
'osf0': {'group': 'ff13::39', 'port': 12345}
}


class Config:
    def __init__(self):
        self.rmacs_osf_orchestrator: str = 'fde3:49c:9f74:4742:0:4c:372b:1c4f'
        self.port: int = 8080
        self.radio_index: str = "0"
        self.starting_channel: int = 36
        self.channels5: List[int] = [36, 40, 44, 48, 149, 153, 157, 161]
        self.osf_interface: str = 'osf0'
        self.nw_interface: str = 'wlp1s0'
        self.phy_interface: str = 'phy1'
        self.driver = "ath10k"
        self.bridge_interface: str = 'br-lan'
        self.addr_prefix: str = 'fdd8'
        self.control_channel_interfaces = ['wlp1s0', 'halow1', 'osf0']
        
        
        # Traffic Monitor and Error Monitor threshold
        self.traffic_threshold = 2500
        self.phy_error_limit = 1500
        self.tx_timeout_limit = 1
        
        # Adaptive frequency hopping parameters 
        #self.freq_quality_report: dict = {"5180":1,"5200":1,"5220":1,"5240":1}
        self.freq_quality_report: dict = {5180:{'nodes':{}, 'Average_quality': 1},5200:{'nodes':{}, 'Average_quality': 1},5220:{'nodes':{}, 'Average_quality': 1},5240:{'nodes':{}, 'Average_quality': 1}}
        self.freq_list: dict = [5180,5200,5220,5240]
        self.seq_limit = 3
        self.hop_interval: int = 5
        self.stability_threshold: int = 2
        self.starting_frequency = "5180"
        self.report_expiry_threshold = 30 #secs
        
        # Channel details
        self.channel_bandwidth: int = 20
        self.beacon_count: int = 10
        self.client_beacon_count: int = 1
        self.buffer_period: int = 2
        
        # Channel Quality Estimation
        self.channel_quality_index_threshold = 1
        
        # Traffic Monitoring
        self.monitoring_sleep_time = 30 #sec
    
        # Error Monitoring
        self.max_error_check = 4
        self.cooldown_period = 30  #sec
        
        # BCQI
        self.bcqi_threshold_time = 30 #sec
            
        
        self.MR_MULTICAST_GROUP = 'ff02::1'  # IPv6 multicast address for entire network
        self.MR_MULTICAST_PORT = 12345
        self.OSF_MULTICAST_GROUP = 'ff13::39'  # IPv6 multicast address for entire network
        self.OSF_MULTICAST_PORT = 12345
        
        self.debug: bool = False
        self.update_flag: bool = True
        self.min_rows: int = 16
        self.periodic_scan: float = 30
        self.periodic_recovery_switch: float = 3
        self.periodic_operating_freq_broadcast: float = 15
        self.data_gathering_wait_time: float = len(self.channels5) + 5
        self.log_file: str = '/var/log/rmacs.log'
        self.bin_file: str= '/root/sample.bin'
        # Path to JSON file containing mean and std for spectral scan data normalization
        #self.col_mean_std_path: str = '/opt/mesh_com/modules/sc-mesh-secure-deployment/src/2_0/features/jamming/normalization_data/cols_mean_std.json'

   
