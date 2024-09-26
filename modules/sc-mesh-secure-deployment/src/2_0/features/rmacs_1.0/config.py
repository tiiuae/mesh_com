import os
import socket
import fcntl
import struct
from typing import List
VALID_CHANNELS = [36, 40, 44, 48, 149, 153, 157, 161]


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
        #self.module_ip: str = self.get_ip_address("eth0")
        
        
        # Traffic Monitor and Error Monitor threshold
        
        self.phy_error_limit = 100
        self.tx_timeout_limit = 5
        
        # Adaptive frequency hopping parameters 
        self.freq_quality_report: dict = {"5180":1,"5200":1,"5220":1,"5240":1}
        self.freq_list: dict = [5180,5200,5220,5240]
        self.seq_limit: int = 3
        self.hop_interval: int = 0
        self.stability_threshold: int = 0
        self.starting_frequency = "5180"
        
        # Channel details
        self.channel_bandwidth: int = 20
        self.beacons_count: int = 10
        
        # Channel Quality Estimation
        self.channel_quality_index_threshold = 3
    
        # Error Monitoring
        self.max_error_check = 1
        
        self.debug: bool = False
        self.min_rows: int = 16
        self.periodic_scan: float = 30
        self.periodic_recovery_switch: float = 3
        self.periodic_target_freq_broadcast: float = 15
        self.data_gathering_wait_time: float = len(self.channels5) + 5
        self.log_file: str = '/var/log/rmacs.log'
        self.bin_file: str= '/root/sample.bin'
        # Path to JSON file containing mean and std for spectral scan data normalization
        #self.col_mean_std_path: str = '/opt/mesh_com/modules/sc-mesh-secure-deployment/src/2_0/features/jamming/normalization_data/cols_mean_std.json'

   