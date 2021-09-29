import subprocess

#ROS node to convert to hex format
#<element id><len><oui><vendor ie>
#standards-oui.ieee.org/oui/oui.txt
# QC OUI: 88-12-4E or 00-A0-C6 or 64-9c-81 ...
# BRCM OUI: BC-97-E1 or 00-1B-E9 or 00-05-B5 ...
#
def prepare_vendor_ie(drone_id):
   length=len(drone_id);
   print(length)
   #Add custom OUI
   oui="bbddcc"
   # 3 byte for OUI
   length=length/2 + 3
   print(length)
   len_tag=format(int(length), '#04x')[2:]
   print(len_tag)
   vendor_ie="dd" + oui + len_tag + drone_id
   return vendor_ie


def update_vendor_ie(drone_id):
    cmd="hostapd_cli SET vendor_elements " + drone_id
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,shell=True)
    cmd="hostapd_cli DISABLE"
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,shell=True)
    cmd="hostapd_cli ENABLE"
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,shell=True)

if __name__=='__main__':
    beacon_vendor_ie=prepare_vendor_ie("AB234566")
    print(beacon_vendor_ie)
