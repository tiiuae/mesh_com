import os
import re
from typing import List


VALID_CHANNELS = [36, 40, 44, 48, 149, 153, 157, 161]


class Options:
    def __init__(self):
        self.jamming_osf_orchestrator: str = 'fd01::1'
        self.port: int = 8080
        self.starting_channel: int = 36
        self.waiting_time: int = 15
        self.channels5: List[int] = [36, 40, 44, 48, 149, 153, 157, 161]
        self.threshold: float = 0.0
        self.osf_interface: str = 'tun0'
        self.scan_interface: str = 'wlp1s0'
        self.mesh_interface: str = 'wlp1s0'
        self.debug: bool = False
        self.min_rows: int = 16
        self.periodic_scan: float = 60
        self.periodic_recovery_switch: float = 20
        self.periodic_target_freq_broadcast: float = 10
        self.spectrum_data_expiry_time: float = 5
        self.data_gathering_wait_time: float = len(self.channels5) + 5
        # The path to the JSON file containing the mean and standard deviation information for the spectral scan data normalization
        self.col_mean_std_path: str = '/opt/mesh_com/modules/sc-mesh-secure-deployment/src/2_0/features/jamming/normalization_data/cols_mean_std.json'
        self.traced_model_path: str = "/opt/mesh_com/modules/sc-mesh-secure-deployment/src/2_0/features/jamming/my_traced_model.pt"

    def validate_configuration(self) -> bool:
        """
        Validate that device and options file configurations are compatible for jamming detection.

        :return: A boolean to denote whether the device and parameter configurations are valid.
        """
        valid = True

        # Check that mesh is set to 20mhz
        try:
            iw_output = os.popen('iw dev').read()
            iw_output = re.sub(r'\s+', ' ', iw_output).split(' ')

            # Extract interface sections from iw_output
            idx_list = [idx - 1 for idx, val in enumerate(iw_output) if val == "Interface"]
            if len(idx_list) > 1:
                idx_list.pop(0)

            # Calculate the start and end indices for interface sections
            start_indices = [0] + idx_list
            end_indices = idx_list + ([len(iw_output)] if idx_list[-1] != len(iw_output) else [])

            # Use zip to create pairs of start and end indices, and extract interface sections
            iw_interfaces = [iw_output[start:end] for start, end in zip(start_indices, end_indices)]

            # Get mesh interface channel width
            for interface_list in iw_interfaces:
                if "mesh" in interface_list:
                    channel_width_index = interface_list.index("width:") + 1
                    channel_width = re.sub("[^0-9]", "", interface_list[channel_width_index]).split()[0]
                    if channel_width != "20":
                        print("Mesh interface channel width must be set to 20 MHz.")
                        valid = False
                    break
                else:
                    print("No mesh interface")
        except Exception as e:
            print(e) if self.debug else None

        # Check that list of channels does not include any DFS channels
        if not all(channel in VALID_CHANNELS for channel in self.channels5):
            print("5 GHz channels must be of the following: (36,40,44,48,149,153,157,161)")
            valid = False

        # Return validity after all checks performed
        return valid


def main():
    args = Options()
    # Validate user input parameter
    if not args.validate_configuration():
        raise Exception("Please adjust the jamming detection configuration according to the above.")


if __name__ == "__main__":
    main()
