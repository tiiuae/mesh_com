import argparse
import getopt
import sys
import datetime
import subprocess
from time import sleep
from threading import Thread

#default rssi monitoring interval: 5sec
rssi_mon_interval = 5
#default interface
interface = "wlan0"

def get_rssi():
    global interface
    cmd = "iw dev " + interface + " station dump | grep 'signal:' | awk '{print $2}'"
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
    rssi = proc.communicate()[0].decode('utf-8')
    return rssi

def log_rssi():
    global rssi_mon_interval
    fn_suffix=str(datetime.datetime.now())
    log_file_path = '/var/log/'
    log_file_name =  'rssi'+fn_suffix+'.txt'
    while True:
        f = open(log_file_path+log_file_name, 'a')
        rssi_sta = get_rssi()
        print(rssi_sta)
        f.write(rssi_sta)
        f.close()
        sleep(rssi_mon_interval)

if __name__=='__main__':

    # Construct the argument parser
    phy_cfg = argparse.ArgumentParser()

    # Add the arguments to the parser
    phy_cfg.add_argument("-r", "--rssi_period", required=True, help="RSSI monitoring period Ex: 5 (equals to 5 sec)")
    phy_cfg.add_argument("-i", "--interface", required=True)
    args = phy_cfg.parse_args()

    #populate args
    rssi_mon_interval = int(args.rssi_period)
    interface = args.interface


    Thread(target=log_rssi).start()

