import os
import socket
import fcntl
import struct
from typing import List
VALID_CHANNELS = [36, 40, 44, 48, 149, 153, 157, 161]


class Options:
    def __init__(self):
        self.jamming_osf_orchestrator: str = 'fde3:49c:9f74:4742:0:4c:372b:1c4f'
        self.port: int = 8080
        self.radio_index: str = "0"
        self.starting_channel: int = 36
        self.channels5: List[int] = [36, 40, 44, 48, 149, 153, 157, 161]
        self.osf_interface: str = 'osf0'
        self.scan_interface: str = 'wlp1s0'
        self.phy_interface: str = self.get_phy_interface()
        self.module_ip: str = self.get_ip_address("eth0")
        self.debug: bool = False
        self.min_rows: int = 16
        self.periodic_scan: float = 30
        self.periodic_recovery_switch: float = 3
        self.periodic_target_freq_broadcast: float = 15
        self.data_gathering_wait_time: float = len(self.channels5) + 5
        self.log_file: str = '/var/log/jamming_avoidance.log'
        # Path to JSON file containing mean and std for spectral scan data normalization
        self.col_mean_std_path: str = '/opt/mesh_com/modules/sc-mesh-secure-deployment/src/2_0/features/jamming/normalization_data/cols_mean_std.json'
        self.traced_model_path: str = "/opt/mesh_com/modules/sc-mesh-secure-deployment/src/2_0/features/jamming/my_traced_model.pt"

    def get_phy_interface(self) -> str:
        """
        Get phy interface value associated with the self.scan_interface
        :return: Phy interface value associated with the self.scan_interface
        """
        phy = os.popen(f'cat /sys/class/net/{self.scan_interface}/phy80211/name').read()[3]
        return phy

    def get_ip_address(self, interface) -> str:
        """
        Get eth0 IP address for nats_config.py file
        :return: eth0 IP address
        """
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            ip_address = socket.inet_ntoa(fcntl.ioctl(
                s.fileno(),
                0x8915,  # SIOCGIFADDR
                struct.pack('256s', bytes(interface, 'utf-8')[:15])
            )[20:24])
            return ip_address
        except IOError:
            return None
