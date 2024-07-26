import os
import subprocess
import struct
from typing import BinaryIO
import pandas as pd

from rmacs_setup import Setup

# Reference : drivers/net/wireless/ath/spectral_common.h
'''
#define SPECTRAL_HT20_NUM_BINS		56
#define SPECTRAL_HT20_40_NUM_BINS		128
'''
HEADER_SIZE = 3
TYPE1_PACKET_SIZE = 17 + 56
TYPE2_PACKET_SIZE = 24 + 128
TYPE3_PACKET_SIZE = 26 + 64
SC_WIDE = 0.3125  # ieee 802.11 constants (in MHz)


class SpectralScanLite:
    '''
    A class SpectralScanLite to scan ath9k based Radio interface card (Doodle card)
    Scan report is in binary format, to be converted to text format.
    '''
    def __init(self, driver: str, interface: str):
        self.phy_interface = Setup.get_phy_interface(driver)
        self.is_interface_up = Setup.get_interface_operstate(interface)
        self.scan_interface = "TBD"
        
    
    def initialize_scan(self, driver: str) -> None:
        """
        Initialize spectral scan lite.
        
        Arguments:
        driver: str -- driver name of the radio interface 
        
        Return:
        None
        """
          
        if driver == "ath9k":
            output_file = f"/sys/kernel/debug/ieee80211/phy{self.phy_interface}/{driver}/spectral_scan_ctl"

            cmd_background = ["echo", "background"]
            with open(output_file, "w") as file:
                subprocess.call(cmd_background, stdout=file, stderr=subprocess.PIPE, shell=False)

            cmd_trigger = ["echo", "trigger"]
            with open(output_file, "w") as file:
                subprocess.call(cmd_trigger, stdout=file, stderr=subprocess.PIPE, shell=False)
        else:
            raise Exception(f"Invalid driver: {driver}")
        
    
    def execute_scan(self,driver: str, bin_file: str) -> None:
        """
        Execute spectral scan lite for ath9k chipset.
        /* enum spectral_mode:
        *
        * @SPECTRAL_DISABLED: spectral mode is disabled
        * @SPECTRAL_BACKGROUND: hardware sends samples when it is not busy with
        *	something else.
        * @SPECTRAL_MANUAL: spectral scan is enabled, triggering for samples
        *	is performed manually.
        * @SPECTRAL_CHANSCAN: Like manual, but also triggered when changing channels
        *	during a channel scan.
        */
        Reference : drivers/net/wireless/ath/ath9k/spectral.h
        """
        # Check for interface up
        if self.is_interface_up:
            # Command to execute spectral scan
            scan_cmd = ["iw", "dev", f"{self.scan_interface}", "scan"]
            try: 
                subprocess.call(scan_cmd, shell=False, stderr=subprocess.STDOUT, stdout=subprocess.DEVNULL)
            except subprocess.CalledProcessError as e:
                print(f"Error: {e}")         
        else:
            print(f"The interface :{self.scan_interface} is not up")
            return
        # Command to stop spectral scan
        cmd_disable = ["echo", "disable"]
        spectral_scan_ctl_file = f"/sys/kernel/debug/ieee80211/phy{self.phy_interface}/{driver}/spectral_scan_ctl"
        try:
            with open(spectral_scan_ctl_file, "w") as file:
                subprocess.call(cmd_disable, stdout=file, stderr=subprocess.PIPE, shell=False)
        except subprocess.CalledProcessError as e:
            print(f"Error: {e}")

        # Command to dump scan output from spectral_scan0 to binary file
        cmd_dump = ["cat", f"/sys/kernel/debug/ieee80211/phy{self.phy_interface}/{driver}/spectral_scan0"]
        try:
            with open(bin_file, "wb") as output_file:
                subprocess.call(cmd_dump, stdout=output_file, stderr=subprocess.PIPE, shell=False)
        except subprocess.CalledProcessError as e:
            print(f"Error: {e}")
            
    def read(self, bin_file: str) -> pd.DataFrame:
        """
        Read spectral scan binary file.
        
        Parameter:
        bin_file: str -- spectral scan binary file

        Return: 
        A dataframe of the spectral scan.
        """
        try:
            binary_scan_file = self.file_open(bin_file)
            file_stats = os.stat(bin_file)
            data = binary_scan_file.read(file_stats.st_size)
            self.file_close(binary_scan_file)
            pos = 0
            count = 0
            self.VALUES = dict()
            while pos < len(data):
                (stype, slen) = struct.unpack_from(">BH", data, pos)
                if not ((stype == 1 and slen == TYPE1_PACKET_SIZE) or
                        (stype == 2 and slen == TYPE2_PACKET_SIZE)):
                    print("skip malformed packet")
                    break
                # 20 MHz
                if stype == 1:
                    if pos >= len(data) - HEADER_SIZE - TYPE1_PACKET_SIZE + 1:
                        break
                    pos += HEADER_SIZE
                    (max_exp, freq, rssi, noise, max_mag, max_index, hweight, tsf) = \
                        struct.unpack_from(">BHbbHBBQ", data, pos)
                    pos += 17

                    sdata = struct.unpack_from("56B", data, pos)
                    pos += 56
                    self.VALUES[count] = (max_exp, freq, rssi, noise, max_mag, max_index, hweight, tsf)
                    count = count + 1

        except FileNotFoundError:
            print("File not found. Make sure the file exists.")
        except PermissionError:
            print("Permission error. Check if you have the necessary permissions to access the file.")
        except Exception as e:
            print(f"{e}")
        vals_list = []
        spectral_capture_df = pd.DataFrame()

        for key, value in self.VALUES.items():
            vals_list.append(list(value))
            
        ath9k_scan_features = ["max_exp","freq", "rssi", "noise", "max_mag", "max_index", "hweight", "tsf"]
        spectral_capture_df = pd.DataFrame(vals_list, columns=ath9k_scan_features)
        return spectral_capture_df 

    @staticmethod
    def file_close(file_pointer: BinaryIO) -> None:
        """
        Close the spectral binary dump file.

        param: Typed version of the return of open() in binary mode
        """
        file_pointer.close()

    @staticmethod
    def file_open(file_path: str) -> BinaryIO:
        """
        Open spectral binary dump file.

        param: The name of the binary dump file.
        return: Typed version of the return of open() in binary mode
        """
        if not os.path.exists(file_path):
            raise FileNotFoundError("File not found.")
        else:
            return open(file_path, 'rb')

    
    
        
        