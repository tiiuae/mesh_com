import subprocess
import time
from wifi import Cell
import pathlib
script_path = pathlib.Path(__file__).parent.resolve()


def scan_wifi(interface):
    '''
    Scan wifi with the AuthAP pattern.
    If more than one, select the best quality one.
    If none, return false.
    '''
    killall(interface)
    print('Scanning APs')
    possible = False
    aps = Cell.all(interface)
    max = 0
    for ap in list(aps):
        if ap.ssid != None and 'AuthAP' in ap.ssid:
            qua = float(ap.quality.split('/')[0]) / float(ap.quality.split('/')[1])
            if qua > max:
                possible = ap.ssid
                max = qua
    return possible


def connect_wifi(candidate, interface):
    '''
    Connect to the best Ap selected in scan_wifi()
    we are using apmanager.sh for this
    '''
    killall(interface)
    command = [str(script_path)+'/apmanager.sh', '-ap_connect', candidate, interface]
    subprocess.call(command, shell=False)


def killall(interface):
    subprocess.call(['killall', 'wpa_supplicant'], shell=False)
    subprocess.call(['killall', 'hostapd'], shell=False)
    subprocess.call(['ifconfig', interface, 'down'], shell=False)
    subprocess.call(['ifconfig', interface, 'up'], shell=False)


def create_ap(ID, interface):
    '''
    If none AuthAP is available, then create a new one.
    Using apmanager.sh
    '''
    killall(interface)
    time.sleep(2)
    command = [str(script_path)+'/apmanager.sh', '-ap_create', ID, interface]
    subprocess.call(command, shell=False)


def clean_ap(ID, interface):
    '''
    If none AuthAP is available, then create a new one.
    Using apmanager.sh
    '''
    killall(interface)
    time.sleep(2)
    command = [str(script_path)+'/apmanager.sh', '-ap_remove', ID, interface]
    subprocess.call(command, shell=False)
