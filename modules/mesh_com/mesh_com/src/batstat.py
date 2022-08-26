"""
Batman stats helper
"""
import subprocess
import re
import json
from time import sleep
import netifaces


class STATUS:  # pylint: disable=too-few-public-methods
    """
    Status values
    """

    def __init__(self):
        self.mesh = "MESH"  # Mesh configuration in use
        self.off = "OFF"  # all radios off
        self.no_config = "NO_CONFIG"  # No configuration set
        self.error = "ERROR"  # general ERROR
        self.ongoing = "ONGOING"  # Ongoing
        self.accesspoint = "AP"  # accesspoint configuration in use
        self.not_avail = "NA"  # Not Available


class Batman:
    """
    Batman mesh class
    """

    def __init__(self):
        self.topology = {}
        self.device_template = {}
        self.device_rssi_list = []
        self.device_noise_dict = {}
        self.mesh_status = STATUS()
        self._status = self.mesh_status.no_config  # "MESH/OFF/NO_CONFIG/ONGOING/AP/ERROR"
        self.iw_type = self.mesh_status.not_avail  # iw dev <wifi> info  - type
        self.network_interface = self.mesh_status.not_avail
        self.freq = self.mesh_status.not_avail
        self.txpower = self.mesh_status.not_avail
        self.iw_state = self.mesh_status.not_avail
        self.country = self.mesh_status.not_avail
        self.hw_name = self.mesh_status.not_avail
        self.thread_running = True
        self.latest_stat = ""

    def _update_interface_name(self):
        """
        Update Wi-Fi device name ti self.network_interface

        :return: None
        """
        out = subprocess.Popen(['batctl', 'if'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        try:
            self.network_interface = out.stdout.readline().decode('utf-8').split(":")[0]
        except (TypeError, IndexError):
            self.network_interface = self.mesh_status.not_avail

    def _update_survey_dump(self):
        """
        Update survey dump info, self.freq, self.noise

        :return: None
        """
        dump = subprocess.Popen(['iw', 'dev', self.network_interface, 'survey', 'dump'],
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE)

        self.device_noise_dict = {}
        freq = 0
        noise = 0

        try:
            for line in dump.stdout:
                line = line.decode("utf-8")
                if "frequency:" in line:
                    freq = re.findall(r'\s*frequency:\s+(\d+) ', line)[0]
                    noise = 0
                elif "noise:" in line:
                    noise = re.findall(r'\s*noise:\s+(-*\d+) ', line)[0]

                if noise and freq:
                    self.device_noise_dict[freq] = noise
                    freq = 0
                    noise = 0
        except (IndexError, TypeError):
            self.device_noise_dict = [self.mesh_status.not_avail]
            print("ERROR survey")

    def _update_station_dump_info(self):
        """
        Update station dump info, self.device_rssi_list

        :return: None
        """
        dump = subprocess.Popen(['iw', 'dev', self.network_interface, 'station', 'dump'],
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE)

        self.device_rssi_list = []
        mac = ""
        rssi = []
        for line in dump.stdout:
            line = line.decode("utf-8")
            if "Station" in line:
                mac = line.split(" ")[1]
                rssi = []
            elif "signal:" in line:
                rssi = re.findall(r'-\d+', line)
            if mac and rssi:
                self.device_rssi_list.append((mac, rssi))
                mac = ""

    def _update_iw_info(self):
        """
        Update device "self.freq" and "self.txpower" info

        :return: None
        """

        dump = subprocess.Popen(['iw', 'dev', self.network_interface, 'info'],
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE)
        try:
            for line in dump.stdout:
                line = line.decode("utf-8")
                if "channel" in line:
                    self.freq = re.findall(r'\((\d+) MHz\)', line)[0]
                elif "txpower" in line:
                    self.txpower = re.findall(r'\d+.\d+', line)[0]
                elif "type" in line:
                    self.iw_state = re.findall(r'type (\w+)', line)[0]
        except (IndexError, TypeError):
            self.freq = self.mesh_status.not_avail
            self.txpower = self.mesh_status.not_avail
            self.iw_state = self.mesh_status.not_avail

    def _update_iw_type(self):
        """
        Update device self.status code

        :return: None
        """

        if self.iw_state == "managed":
            self.status = self.mesh_status.no_config
        elif self.iw_state == "AP":
            self.status = self.mesh_status.accesspoint
        elif self.iw_state == "mesh":
            self.status = self.mesh_status.mesh
        elif self.iw_state == "IBSS":
            self.status = self.mesh_status.mesh
        elif self.iw_state == self.mesh_status.not_avail:
            self.status = self.mesh_status.not_avail
        else:
            self.status = self.mesh_status.error

    def _update_iw_reg(self):
        """
        Update device "self.country" code

        :return: None
        """

        dump = subprocess.Popen(['iw', 'reg', 'get'], stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE)
        try:
            for line in dump.stdout:
                line = line.decode("utf-8")
                if "country" in line:
                    self.country = re.findall(r'country (\w+)', line)[0]
        except (IndexError, TypeError):
            self.country = self.mesh_status.not_avail

    @property
    def _get_my_mac(self):
        """
        Get device mac

        :return: mac or not_avail
        """
        self._update_interface_name()
        try:
            return netifaces.ifaddresses(self.network_interface)[netifaces.AF_PACKET][0]['addr']
        except (ValueError, TypeError, IndexError):
            return self.mesh_status.not_avail

    def _get_my_rssi(self, mac):
        """
        Get device rssi values

        :return: rssi value or not_avail
        """
        result = [r for (m, r) in self.device_rssi_list if m == mac]
        if result:
            return result[0]

        return [self.mesh_status.not_avail]

    @property
    def status(self):
        """
        Status getter

        :return: status
        """
        return self._status

    @status.setter
    def status(self, new_status):
        """
        Status setter

        :return: None
        """
        self._status = new_status

    def _update_device_info(self):
        self._update_interface_name()  # Wi-Fi interface name
        self._update_station_dump_info()
        self._update_iw_info()
        self._update_iw_type()
        self._update_iw_reg()
        self._update_survey_dump()
        self.hw_name = "Wi-Fi"  # needs implementation if radio is not Wi-Fi

    def _create_template(self):
        """
        Create report template

        :return: None
        """

        self._update_device_info()

        self.topology = {'status': self.status,  # "MESH/OFF/NO_CONFIG/ONGOING/AP/"
                         'my_mac': self._get_my_mac,  # e.g. 00:11:22:33:44:55
                         'noise': self.device_noise_dict[self.freq]
                         if self.freq in self.device_noise_dict
                         else self.mesh_status.not_avail,
                         'freq': self.freq,
                         'txpower': self.txpower,
                         'country': self.country,
                         'hw': self.hw_name,
                         'devices': ''}  # list of devices (using device_template)

        self.device_template = {'a': '',  # active path
                                'o': '',  # originator
                                'ls': '',  # last-seen
                                'q': '',  # quality
                                'nh': ''}  # next-hop

    def update_stat_data(self):
        """
        Update topology as JSON

        :return: None
        """
        self._create_template()

        route = []

        try:
            proc = subprocess.Popen(['batctl', 'o'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            for line in proc.stdout:
                device = dict(self.device_template)
                aux = line.split()

                if b"Originator" not in aux[0] and b"B.A.T" not in line:
                    # active route check
                    if b"*" in aux[0]:
                        index = 1
                    else:
                        index = 0

                    device['a'] = str(index)
                    device['o'] = aux[0 + index].decode("utf-8")
                    device['or'] = self._get_my_rssi(device['o'])
                    device['ls'] = aux[1 + index].decode("utf-8").replace("s", "")
                    device['q'] = aux[2 + index].decode("utf-8").replace(')', '').replace('(', '')
                    device['nh'] = aux[3 + index].decode("utf-8")
                    device['nhr'] = self._get_my_rssi(device['nh'])
                    route.append(device)

            self.topology['devices'] = route
        except FileNotFoundError:
            self.topology['devices'] = self.device_template  # if not succeed, return empty template
            print("update_stat_data: ERROR")

        return self.topology

    def get_stat(self):
        """
        Get stat as JSON

        :return: "self.topology" in json compatible str
        """
        return str(self.topology).replace("'", "\"")

    def run(self):
        """
        Run method for task

        :return: None
        """
        while self.thread_running:
            self.update_stat_data()
            self.latest_stat = self.get_stat()
            sleep(1.0)


# Real user is mesh_executor
if __name__ == "__main__":
    # usage tip
    bat_status = STATUS()
    bat = Batman()
    bat.status = bat_status.not_avail  # update this in the other logic

    # get up-to-date information about the network
    bat.update_stat_data()

    # get topology and example pretty print
    obj = json.loads(bat.get_stat())
    print(json.dumps(obj, indent=3))
