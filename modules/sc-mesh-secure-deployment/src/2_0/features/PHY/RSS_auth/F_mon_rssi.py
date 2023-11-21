import argparse
import subprocess
import sys
import csv
import datetime
import time
from time import sleep
from threading import Thread
from getmac import get_mac_address
from netaddr import *
import yaml
import os

rssi_values = []

def get_mac_oui():
    mac = EUI(get_mac_address(interface))
    oui = mac.oui
    print(oui.registration().address)
    return oui

def get_rssi():
    global interface
    cmd = "iw dev " + interface + " station dump | grep 'signal:' | awk '{print $2}'"
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
    rssi = proc.communicate()[0].decode('utf-8')
    return rssi

def log_rssi():
    global rssi_mon_interval
    global debug
    flag_file_path = '/tmp/run_flag.txt'  # Path to the flag file

    fn_suffix = datetime.datetime.now().strftime('%m_%d_%Y_%H_%M_%S')
    log_file_path = './'
    log_file_name = 'rssi' + fn_suffix + '.csv'
    header_written = False

    while os.path.exists(flag_file_path):  # Check if the flag file exists
        rssi_sta = get_rssi()
        if debug:
            print(rssi_sta)

        with open(log_file_path + log_file_name, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            if not header_written:
                writer.writerow(['Timestamp', 'RSSI'])  # Write the header
                header_written = True
            writer.writerow([time.time(), rssi_sta])

        sleep(rssi_mon_interval)

    print("RSSI capturing stopped.")

if __name__ == '__main__':
    # Construct the argument parser
    phy_cfg = argparse.ArgumentParser()

    # Add the arguments to the parser
    phy_cfg.add_argument("-r", "--rssi_period", required=True, help="RSSI monitoring period Ex: 5 (equals to 5 sec)")
    phy_cfg.add_argument("-i", "--interface", required=True)
    args = phy_cfg.parse_args()

    # Get the physical parameter monitoring configuration
    print('> Loading yaml conf... ')
    conf = yaml.safe_load(open("phy_param.conf", 'r'))
    debug = conf['debug']
    rssi_mon_interval = conf['rssi_poll_interval']
    interface = conf['interface']
    capture_rssi = conf['rssi']

    # Populate args
    rssi_mon_interval = int(args.rssi_period)
    interface = args.interface

    # Capture RSSI if enabled in config file
    if capture_rssi:
        Thread(target=log_rssi).start()

