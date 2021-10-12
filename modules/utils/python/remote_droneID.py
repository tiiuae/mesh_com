import subprocess
from time import sleep
from threading import Thread
import yaml

class DRI:
    # ROS node to convert to hex format
    # <element id><len><oui><vendor ie>
    # standards-oui.ieee.org/oui/oui.txt
    # QC OUI: 88-12-4E or 00-A0-C6 or 64-9c-81 ...
    # BRCM OUI: BC-97-E1 or 00-1B-E9 or 00-05-B5 ...
    #
    def __init__(self):
        self.bt_if = ""
        self.dri_if = ""
        self.obs_if = ""
        print('> Loading yaml conf... ')
        self._conf = yaml.safe_load(open("dri.conf", 'r'))
        self.debug = self._conf['debug']
        self.dri_update_int = self._conf['dri_ie_update_interval']
        self.dri_data_file = self._conf['dri_file_name']
        self.tx_mode = self._conf['mode']
        self.dri_role = self._conf['dri_role']

    @staticmethod
    def prepare_vendor_ie(drone_id):
        length = len(drone_id)
        # Add custom OUI
        oui = "bbddcc"
        # 3 byte for OUI
        length = length / 2 + 3
        len_tag = format(int(length), '#04x')[2:]
        vendor_ie = "dd" + len_tag + oui + drone_id
        return vendor_ie

    @staticmethod
    def update_vendor_ie(drone_id):
        cmds = ["hostapd_cli SET vendor_elements " + str(drone_id),
                "hostapd_cli DISABLE",
                "hostapd_cli ENABLE"]
        for cmd in cmds:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)

    @staticmethod
    def prepare_ble_dri_uuid(drone_id):
        return ' '.join([drone_id[i:i + 2] for i in range(0, len(drone_id), 2)])

    @staticmethod
    def ble_dri_tx(self, drone_id):
        # allow rfkill to bring up bluetooth hci interface
        cmd = "rfkill unblock bluetooth"
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
        # bring up bluetooth interface.
        # To do: use bluez python lib
        cmd = "hciconfig " + str(self.bt_if) + " up"
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
        # enable ble advertising, To do: dynamically detect connection vs connectionless adv
        cmd = "hciconfig " + str(self.bt_if) + " leadv 3"
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
        sd = self.prepare_ble_dri_uuid(drone_id)
        # To do: generate dynamic UUID and remove hardcoded tx power(get from conf)
        cmd = "hcitool -i " + str(self.bt_if) + " cmd 0x08 0x0008 1E 02 01 1A 1A FF 4C 00 02 15 " +\
              str(sd) + " 00 00 00 00 " + "C5 00"
        print(cmd)
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)

    @staticmethod
    def get_wifi_beacon_list(self):
        cmd = "iw wlan0 scan -u | grep 'SSID\|Vendor specific'"
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
        dri_oui_list = proc.communicate()[0].decode('utf-8').strip()
        if self.debug:
            print(f"{dri_oui_list=}")

    def dri_thread(self):
        while True:
            f = open(self.dri_data_file, 'r')
            dri_data = f.read()
            if self.debug:
                print(f"{dri_data=}")
            f.close()
            if self.tx_mode == 'wifi':
                beacon_vendor_ie = self.prepare_vendor_ie(dri_data)
                if self.debug:
                    print(f"{beacon_vendor_ie=}")
                self.update_vendor_ie(beacon_vendor_ie)
            elif self.tx_mode == 'bt':
                self.ble_dri_tx(dri_data)
            sleep(self.dri_update_int)

    def observer_thread(self):
        while True:
            if self.tx_mode == 'wifi':
                self.get_wifi_beacon_list(self)
            sleep(self.dri_update_int)

    def run(self):
        if self.dri_role == "uav":
            if self.tx_mode == 'wifi':
                self.dri_if = self._conf['dri_if']
            elif self.tx_mode == 'bt':
                self.bt_if = self._conf['bt_if_name']
            Thread(target=self.dri_thread).start()
        elif self.dri_role == "observer":
            self.obs_if = self._conf['obs_if']
            Thread(target=self.observer_thread).start()

if __name__ == '__main__':
    drone_device_id = DRI()
    drone_device_id.run()
