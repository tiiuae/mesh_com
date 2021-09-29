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
   vendor_ie="dd" + oui + len_tag + drone_id
   return vendor_ie


def update_vendor_ie(drone_id):
    cmd="hostapd_cli SET vendor_elements " + str(drone_id)
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,shell=True)
    cmd="hostapd_cli DISABLE"
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,shell=True)
    cmd="hostapd_cli ENABLE"
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,shell=True)

def dri_thread():
    global dri_data_file
    global dri_update_int
    global debug

    while True:
        f = open(dri_data_file, 'r')
        dri_data = f.read()
        if debug:
            print(dri_data)
        f.close()
        beacon_vendor_ie=prepare_vendor_ie(dri_data)
        if debug:
            print(beacon_vendor_ie)
        update_vendor_ie(beacon_vendor_ie)
        sleep(dri_update_int)

if __name__=='__main__':
    print('> Loading yaml conf... ')
    conf = yaml.safe_load(open("dri.conf", 'r'))
    debug = conf['debug']
    dri_update_int = conf['dri_ie_update_interval']
    dri_data_file = conf['dri_file_name']
    Thread(target=dri_thread).start()

