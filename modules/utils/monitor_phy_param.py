import argparse
import getopt
import sys
import datetime
import time
import subprocess
from time import sleep
from threading import Thread
from getmac import get_mac_address
from netaddr import *
import yaml
import serial
import re

def is_csi_supported():
    global csi_type
    global debug
    global serial_port

    print(csi_type)
    if (csi_type == 'nexmon'):
        soc_version_cmd = "cat /proc/cpuinfo | grep 'Revision' | awk '{print $3}'"
        proc = subprocess.Popen(soc_version_cmd, stdout=subprocess.PIPE, shell=True)
        soc_version = proc.communicate()[0].decode('utf-8').strip()
        if debug:
           print("SOC VERSION:"+soc_version)
        if ((soc_version == 'a020d3') or (soc_version == 'b03114')):
            print("RPI nexmon CSI is supported, SOC VERSION:"+soc_version)
            return 1
        else:
            return 0
    elif (csi_type == 'esp'):
        ser=serial.Serial('/dev/' + serial_port, 115200, timeout=1)
        ser.flush()
        while True:
            try:
                data = ser.readline().decode('utf-8')
                if re.search('CSI', data):
                    with open("esp_csi_data.txt", "a") as f:
                        f.write(data)
            except:
                break
        return 0
    else:
        return 0;


def capture_raw_csi():
    global interface
    global mac_addr_filter
    global channel
    global bandwidth
    global debug

    #Get CSI extractor filter
    csi_filter_cmd = "makecsiparams -c " + str(channel) + '\/' + str(bandwidth) + " -C 1 -N 1 -m " + mac_addr_filter + " -b 0x88"
    if debug:
        print(csi_filter_cmd)
    proc = subprocess.Popen(csi_filter_cmd, stdout=subprocess.PIPE, shell=True)
    filter_conf_resp = proc.communicate()[0].decode('utf-8').strip()
    if debug:
        print(filter_conf_resp)

   #Configure CSI extractor
    csi_ext_cmd = "nexutil -I" + interface + " -s500 -b -l34 -v"+filter_conf_resp
    proc = subprocess.Popen(csi_ext_cmd, stdout=subprocess.PIPE, shell=True)
    if debug:
        print(csi_ext_cmd)

    en_monitor_mode_cmd = "iw phy `iw dev " + interface + " info | gawk '/wiphy/ {printf \"phy\" $2}'` interface add mon0 type monitor && ifconfig mon0 up"
    proc = subprocess.Popen(en_monitor_mode_cmd, stdout=subprocess.PIPE, shell=True)

    if debug:
        print(en_monitor_mode_cmd)

    #Make sure injector is generating unicast traffic to mac_addr_filter
    #destination, Start tcpdump to capture  the CSI
    dump_cmd = "tcpdump -G 60 -i " + interface + " dst port 5500 -w csi-%m_%d_%Y_%H_%M_%S.pcap"
    proc = subprocess.Popen(dump_cmd, stdout=subprocess.PIPE, shell=True)

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

    fn_suffix=str(datetime.datetime.now().strftime('%m_%d_%Y_%H_%M_%S'))
    log_file_path = '/var/log/'
    log_file_name =  'rssi'+fn_suffix+'.txt'
    while True:
        f = open(log_file_path+log_file_name, 'a')
        rssi_sta = get_rssi()
        if debug:
            print(rssi_sta)
        f.write(str(time.time())+' '+rssi_sta)
        f.close()
        sleep(rssi_mon_interval)

if __name__=='__main__':

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
    csi_type  = conf['csi_format']
    interface = conf['interface']
    capture_rssi = conf['rssi']
    capture_csi = conf['csi']
    mac_addr_filter = conf['mac_addr_filter']
    channel = conf['channel']
    bandwidth = conf['bandwidth']
    serial_port = conf['serial_port']
    #populate args
    rssi_mon_interval = int(args.rssi_period)
    interface = args.interface

    # Capture CSI with type defined in config file
    if capture_csi:
        val = is_csi_supported()
        if (val == 1):
            Thread(target=capture_raw_csi).start()

    # Capture RSSI if enabled in config file
    if capture_rssi:
        Thread(target=log_rssi).start()

