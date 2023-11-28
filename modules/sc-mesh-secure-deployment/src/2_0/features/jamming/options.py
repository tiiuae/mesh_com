from typing import List

VALID_CHANNELS = [36, 40, 44, 48, 149, 153, 157, 161]

class Options:
    def __init__(self):
        self.jamming_osf_orchestrator: str = 'fd04::1'
        self.port: int = 8080
        self.radio_index: str = "0"
        self.starting_channel: int = 36
        self.channels5: List[int] = [36, 40, 44, 48, 149, 153, 157, 161]
        self.osf_interface: str = 'tun0'
        self.scan_interface: str = 'wlp1s0'
        self.debug: bool = False
        self.min_rows: int = 16
        self.periodic_scan: float = 60
        self.periodic_recovery_switch: float = 20
        self.periodic_target_freq_broadcast: float = 10
        self.data_gathering_wait_time: float = len(self.channels5) + 5
        self.log_file: str = '/var/log/jamming_avoidance.log'
        # Path to JSON file containing mean and std for spectral scan data normalization
        self.col_mean_std_path: str = '/opt/mesh_com/modules/sc-mesh-secure-deployment/src/2_0/features/jamming/normalization_data/cols_mean_std.json'
        self.traced_model_path: str = "/opt/mesh_com/modules/sc-mesh-secure-deployment/src/2_0/features/jamming/my_traced_model.pt"
