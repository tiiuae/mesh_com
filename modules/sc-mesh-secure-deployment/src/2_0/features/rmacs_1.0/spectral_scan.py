#!/usr/bin/python
import math
import os
import struct
import subprocess
import sys
import time
from typing import BinaryIO
import json
import re

import pandas as pd

from config import Config
from rmacs_setup import get_mesh_freq, get_channel_bw, get_interface_operstate, get_phy_interface
from logging_config import logger

class Spectral_Scan:
    def __init__(self):
        self.VALUES = dict()
        self.args = Config()
        self.phy_interface = self.args.phy_interface
        self.nw_interface = self.args.nw_interface
        self.is_interface_up = get_interface_operstate(self.nw_interface)
        self.driver = self.args.driver
        self.bin_file = self.args.bin_file
        print(" Spectral scan init method called............")
        #self.scan_interface = "TBD"

    def initialize_scan(self) -> None:
        """
        Initialize spectral scan.
        """
        print(" initialize method called............")
        if self.driver == "ath10k":
            output_file = f"/sys/kernel/debug/ieee80211/{self.phy_interface}/{self.driver}/spectral_scan_ctl"
            print(f"output file : {output_file}")

            cmd_background = ["echo", "background"]
            with open(output_file, "w") as file:
                subprocess.call(cmd_background, stdout=file, stderr=subprocess.PIPE, shell=False)

            cmd_trigger = ["echo", "trigger"]
            with open(output_file, "w") as file:
                subprocess.call(cmd_trigger, stdout=file, stderr=subprocess.PIPE, shell=False)
        else:
            raise Exception(f"Invalid driver: {self.driver}")

    def execute_scan(self, freq: str) -> None:
        """
        Execute spectral scan.

        param interface: A string of the interface to use to perform the spectral scan.
        param frequencies: A string of the frequencies to scan.
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
        """
        
        print(" execute scan method called............")
         # Check for interface up
        if self.is_interface_up:
            # Command to execute spectral scan
            scan_cmd = ["iw", "dev", f"{self.nw_interface}", "scan", "freq", f"{freq}", "flush"]
            print(f"scan cmd : {scan_cmd}")
            try: 
                subprocess.call(scan_cmd, shell=False, stderr=subprocess.STDOUT, stdout=subprocess.DEVNULL)
            except subprocess.CalledProcessError as e:
                print(f"Error: {e}")         
        else:
            print(f"The interface :{self.driver} is not up")
            return
        # Command to stop spectral scan
        cmd_disable = ["echo", "disable"]
        spectral_scan_ctl_file = f"/sys/kernel/debug/ieee80211/{self.phy_interface}/{self.driver}/spectral_scan_ctl"
        try:
            with open(spectral_scan_ctl_file, "w") as file:
                subprocess.call(cmd_disable, stdout=file, stderr=subprocess.PIPE, shell=False)
        except subprocess.CalledProcessError as e:
            print(f"Error: {e}")

        # Command to dump scan output from spectral_scan0 to binary file
        cmd_dump = ["cat", f"/sys/kernel/debug/ieee80211/{self.phy_interface}/{self.driver}/spectral_scan0"]
        try:
            with open(self.bin_file, "wb") as output_file:
                subprocess.call(cmd_dump, stdout=output_file, stderr=subprocess.PIPE, shell=False)
        except subprocess.CalledProcessError as e:
            print(f"Error: {e}")
            
    def run_fft_eval(self, freq:str) -> list[dict]:

        try:
            # Run the subprocess command
            result = subprocess.run(['/root/fft_eval_json', self.bin_file, f"{freq}"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            
            # Check return code and handle output
            if result.returncode == 0:
                print("Channel Quality Report :", result.stdout)
                output = result.stdout
                output = re.sub(r'([{,])\s*(\w+)\s*:', r'\1"\2":', output)
                return output
            else:
                print(f"Command failed with return code {result.returncode}. Error:", result.stderr)
                return [{"error": result.stderr}]

        except FileNotFoundError as e:
            return [{"error": f"Command not found: {e}"}]
        except subprocess.SubprocessError as e:
            return [{"error": f"Subprocess failed: {e}"}]
        except json.JSONDecodeError as e:
            return [{"error": f"Failed to parse JSON: {e}"}]
       
       
    
'''
 
def main():
    print('Im main method...........')
    scan = Spectral_Scan()
    scan.initialize_scan()
    scan.execute_scan("5220")
    scan.run_fft_eval("5220")


if __name__ == "__main__":
    main()
    
'''