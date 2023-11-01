"""
Batman stats helper
"""
import subprocess
import re
import json
from time import sleep
from datetime import datetime
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

# pylint: disable=too-many-instance-attributes
class Batman:
    """
    Batman mesh class
    """

    def __init__(self, loop_interval: float = 1.0, batman_interface: str = "bat0"):
        self.batman_interface = batman_interface
        self.topology: {} = {}
        self.device_template: {} = {}
        self.device_rssi_list_per_radio: [dict, ...] = []
        self.device_noise_list_per_radio: [dict, ...] = []
        self.mesh_status = STATUS()
        self.status: [str, ...] = [self.mesh_status.no_config]  # "MESH/OFF/NO_CONFIG/
                                                                 # ONGOING/AP/ERROR"
        self.iw_type: [str, ...] = [self.mesh_status.not_avail]  # iw dev <Wi-Fi> info  - type
        self.network_interface: [str, ...] = [self.mesh_status.not_avail]
        self.freq: [str, ...] = [self.mesh_status.not_avail]
        self.txpower: [str, ...] = [self.mesh_status.not_avail]
        self.iw_state: [str, ...] = [self.mesh_status.not_avail]
        self.country: [str, ...] = [self.mesh_status.not_avail]
        self.hw_name: [str, ...] = [self.mesh_status.not_avail]
        self.thread_running: bool = False
        self.latest_stat: str = ""
        self.interval: float = loop_interval

    def _update_interface_name(self):
        """
        Update Wi-Fi device name ti self.network_interface

        :return: None
        """
        with subprocess.Popen(['batctl', 'meshif', self.batman_interface, 'if'],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE) as out:

            try:
                self.network_interface = [line.decode('utf-8').split(":")[0] for line in out.stdout]
            except (TypeError, IndexError):
                self.network_interface = [self.mesh_status.not_avail]

    def _update_survey_dump(self):
        """
        Update survey dump info, freq, noise

        :return: None
        """

        self.device_noise_list_per_radio = []

        for interface in self.network_interface:
            with subprocess.Popen(['iw', 'dev', interface, 'survey', 'dump'],
                                    stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE) as dump:

                radio_noise_dict = {}
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
                            radio_noise_dict[freq] = noise
                            freq = 0
                            noise = 0
                    radio_noise_dict["status"] = self.mesh_status.mesh
                    self.device_noise_list_per_radio.append(radio_noise_dict)
                except (IndexError, TypeError):
                    print("ERROR survey")

    def _update_station_dump_info(self):
        """
        Update station dump info, self.device_rssi_list

        :return: None
        """
        self.device_rssi_list_per_radio = []

        for interface in self.network_interface:
            with subprocess.Popen(['iw', 'dev', interface, 'station', 'dump'],
                                    stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE) as dump:

                mac_and_rssi_dict = {}
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
                        mac_and_rssi_dict[mac] = rssi
                        mac = ""

                self.device_rssi_list_per_radio.append(mac_and_rssi_dict)

    def _update_iw_info(self):
        """
        Update device "self.freq" and "self.txpower" info

        :return: None
        """

        self.freq = []
        self.txpower = []
        self.iw_state = []

        for interface in self.network_interface:

            with subprocess.Popen(['iw', 'dev', interface, 'info'],
                                    stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE) as dump:
                try:
                    for line in dump.stdout:
                        line = line.decode("utf-8")
                        if "channel" in line:
                            self.freq.append(str(re.findall(r'\((\d+) MHz\)', line)[0]))
                        elif "txpower" in line:
                            self.txpower.append(str(re.findall(r'\d+.\d+', line)[0]))
                        elif "type" in line:
                            self.iw_state.append(str(re.findall(r'type (\w+)', line)[0]))
                except (IndexError, TypeError):
                    self.freq.append(self.mesh_status.not_avail)
                    self.txpower.append(self.mesh_status.not_avail)
                    self.iw_state.append(self.mesh_status.not_avail)

    def _update_iw_type(self):
        """
        Update device self.status code

        :return: None
        """

        self.status: [str, ...] = []

        # for loop network_interfaces with index
        for index in range(len(self.network_interface)):
            if self.iw_state[index] == "managed":
                self.status.append(self.mesh_status.no_config)
            elif self.iw_state[index] == "AP":
                self.status.append(self.mesh_status.accesspoint)
            elif self.iw_state[index] == "mesh":
                self.status.append(self.mesh_status.mesh)
            elif self.iw_state[index] == "IBSS":
                self.status.append(self.mesh_status.mesh)
            elif self.iw_state[index] == self.mesh_status.not_avail:
                self.status.append(self.mesh_status.not_avail)
            else:
                self.status.append(self.mesh_status.error)

    def _update_iw_reg(self):
        """
        Update device "self.country" code

        :return: None
        """

        with subprocess.Popen(['iw', 'reg', 'get'], stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE) as dump:
            try:
                for line in dump.stdout:
                    line = line.decode("utf-8")
                    if "country" in line:
                        self.country = re.findall(r'country (\w+)', line)[0]
                        break
            except (IndexError, TypeError):
                self.country = self.mesh_status.not_avail

    @property
    def _get_my_mac(self) -> [str, ...]:
        """
        Get device mac

        :return: mac or not_avail
        """
        self._update_interface_name()
        macs = []
        for interface in self.network_interface:
            try:
                macs.append(netifaces.ifaddresses(interface)[netifaces.AF_PACKET][0]['addr'])
            except (ValueError, TypeError, IndexError):
                return [self.mesh_status.not_avail]

        return macs

    def _get_my_rssi(self, mac: str) -> [str, ...]:
        """
        Get device rssi values

        :return: rssi value or not_avail
        """
        for index in range(len(self.network_interface)):
            if mac in self.device_rssi_list_per_radio[index]:
                return self.device_rssi_list_per_radio[index][mac]

        return [self.mesh_status.not_avail]


    def _update_device_info(self):
        self._update_interface_name()  # Wi-Fi interface name
        self._update_station_dump_info()
        self._update_iw_info()
        self._update_iw_type()
        self._update_iw_reg()
        self._update_survey_dump()
        self.hw_name = "Wi-Fi"  # needs implementation if radio is not Wi-Fi

    @staticmethod
    def _timestamp() -> str:
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")

    def _create_template(self):
        """
        Create report template

        :return: None
        """

        noise_per_used_channel = []

        self._update_device_info()

        for index in range(0, len(self.network_interface)):
            if self.freq[index] in self.device_noise_list_per_radio[index]:
                noise_per_used_channel.append(
                    self.device_noise_list_per_radio[index][self.freq[index]])
            else:
                noise_per_used_channel.append(self.mesh_status.not_avail)

        self.topology = {'ts': self._timestamp(), # timestamp
                         'batman': self.batman_interface,  # batman interface name
                         'netifs': self.network_interface,  # network interfaces
                         'status': self.status,  # "MESH/OFF/NO_CONFIG/ONGOING/AP/"
                         'my_mac': self._get_my_mac,  # e.g. 00:11:22:33:44:55
                         'noise': noise_per_used_channel,
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
            with subprocess.Popen(['batctl', 'meshif', self.batman_interface, 'o' , '-H'],
                                  stdout=subprocess.PIPE, stderr=subprocess.PIPE) as proc:

                for line in proc.stdout:
                    device = dict(self.device_template)
                    aux = line.split()

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
        self.thread_running = True
        while self.thread_running:
            self.update_stat_data()
            self.latest_stat = self.get_stat()
            sleep(self.interval)


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
