import subprocess
from time import sleep
from threading import Thread
import yaml

#ROS node to convert to hex format
#<element id><len><oui><vendor ie>
#standards-oui.ieee.org/oui/oui.txt
# QC OUI: 88-12-4E or 00-A0-C6 or 64-9c-81 ...
# BRCM OUI: BC-97-E1 or 00-1B-E9 or 00-05-B5 ...
#
def prepare_vendor_ie(drone_id):
   length=len(drone_id);
   #Add custom OUI
   oui="bbddcc"
   # 3 byte for OUI
   length=length/2 + 3
   len_tag=format(int(length), '#04x')[2:]
   vendor_ie="dd" + len_tag + oui + drone_id
   return vendor_ie

def update_vendor_ie(drone_id):
    cmd="hostapd_cli SET vendor_elements " + str(drone_id)
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,shell=True)
    cmd="hostapd_cli DISABLE"
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,shell=True)
    cmd="hostapd_cli ENABLE"
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,shell=True)


def prepare_ble_dri_uuid(drone_id):
    return ' '.join([drone_id[i:i+2] for i in range(0, len(drone_id), 2)])

def ble_dri_tx(drone_id):
    global bt_if
    # allow rfkill to bring up bluetooth hci interface
    cmd="rfkill unblock bluetooth"
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,shell=True)
    # bring up bluetooth interface.
    # To do: use bluez python lib
    cmd="hciconfig " + str(bt_if) + " up"
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,shell=True)

    # enable ble advertising, To do: dynamically detect connection vs connectionless adv
    cmd="hciconfig " + str(bt_if) + " leadv 3"
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,shell=True)

    sd = prepare_ble_dri_uuid(drone_id)
    # To do: generate dynamic UUID and remove hardcoded tx power(get from conf)
    cmd="hcitool -i " + str(bt_if) + " cmd 0x08 0x0008 1E 02 01 1A 1A FF 4C 00 02 15 " + str(sd) + " 00 00 00 00 " + "C5 00"
    print(cmd)
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,shell=True)

def get_wifi_beacon_list():
    cmd="iw wlan0 scan -u | grep 'SSID\|Vendor specific'"
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,shell=True)
    dri_oui_list = proc.communicate()[0].decode('utf-8').strip()
    if debug:
        print(dri_oui_list)

def dri_thread():
    global dri_data_file
    global dri_update_int
    global debug
    global tx_mode

    while True:
        f = open(dri_data_file, 'r')
        dri_data = f.read()
        if debug:
            print(dri_data)
        f.close()
        if (tx_mode == 'wifi'):
            beacon_vendor_ie=prepare_vendor_ie(dri_data)
            if debug:
                print(beacon_vendor_ie)
            update_vendor_ie(beacon_vendor_ie)
        elif (tx_mode == 'bt'):
            ble_dri_tx(dri_data)
        sleep(dri_update_int)

def observer_thread():
    global dri_update_int
    global debug
    global tx_mode

    while True:
        if (tx_mode == 'wifi'):
            get_wifi_beacon_list()
        sleep(dri_update_int)

if __name__=='__main__':
    print('> Loading yaml conf... ')
    conf = yaml.safe_load(open("dri.conf", 'r'))
    debug = conf['debug']
    dri_update_int = conf['dri_ie_update_interval']
    dri_data_file = conf['dri_file_name']
    tx_mode = conf['mode']
    dri_role = conf['dri_role']

    if (dri_role == "uav"):
        if (tx_mode == 'wifi'):
            dri_if = conf['dri_if']
        elif (tx_mode == 'bt'):
            bt_if = conf['bt_if_name']
        Thread(target=dri_thread).start()
    elif (dri_role == "observer"):
        obs_if = conf['obs_if']
        Thread(target=observer_thread).start()
