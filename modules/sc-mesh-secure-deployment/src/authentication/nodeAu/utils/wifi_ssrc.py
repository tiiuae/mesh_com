from wifi import Cell
import subprocess
import socket
import fcntl
import struct
import time


def scan_wifi():
    '''
    Scan wifi with the AuthAP pattern.
    If more than one, select the best quality one.
    If none, return false.
    '''
    killall()
    print('Scanning APs')
    possible = False
    aps = Cell.all('wlan0')
    max = 0
    for ap in list(aps):
        if 'AuthAP' in ap.ssid:
            qua = float(ap.quality.split('/')[0]) / float(ap.quality.split('/')[1])
            if qua > max:
                possible = ap.ssid
                max = qua
    return possible


def connect_wifi(candidate):
    '''
    Connect to the best Ap selected in scan_wifi()
    we are using apmanager.sh for this
    '''
    command = '/bin/bash utils/apmanager.sh  -ap_connect ' + candidate
    subprocess.call(command, shell=True)


def killall():
    subprocess.call('pkill wpa_supplicant', shell=True)
    subprocess.call('ifconfig wlan0 down', shell=True)
    subprocess.call('ifconfig wlan0 up', shell=True)


def create_ap(ID):
    '''
    If none AuthAP is available, then create a new one.
    Using apmanager.sh
    '''
    killall()
    time.sleep(2)
    command = '/bin/bash utils/apmanager.sh -ap_create ' + ID
    subprocess.call(command, shell=True)


def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15].encode())
    )[20:24])
