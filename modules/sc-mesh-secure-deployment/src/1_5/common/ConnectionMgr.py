import fcntl
import os as osh
import random
import socket
import string
import struct
import subprocess
import sys
from os import getenv
from pathlib import Path
from threading import Thread

import yaml

from .gw import main
from .utils import Utils

ut = Utils()

script_path = Path(__file__).parent.resolve()
src_path = str(script_path).split('common', maxsplit=1)[0].split('1_5')[0]
docker_path = str(script_path).split('common')[0].split('1_5')[0].split('src')[0].split('sc-mesh-secure-deployment')[0] + 'utils/docker'


class ConnectionMgr:
    def __init__(self):
        self.config_11s_mesh_path = osh.path.join(
            getenv("MESH_COM_ROOT", ""), f"{src_path}/bash/conf-11s-mesh.sh"
        )
        self.config_mesh_path = osh.path.join(
            getenv("MESH_COM_ROOT", ""), f"{src_path}bash/conf-mesh.sh"
        )
        self.mesh_ip = None
        self.mesh_if = None
        self.mesh_mac = None
        # reference to utils
        self.util = ut
        self.auth_ap_if = None
        self.auth_ap_ip = None
        self.auth_ap_mac = None
        self.sta_if = None
        self.sta_mac = None
        # 11s :default mesh mode
        self.mesh_mode = "11s"
        self.gw = False
        self.bridge = False
        self.concurrency = ""

    def starting_mesh(self):  # Get the mesh_com config
        """
        start mesh service(11s/ibss)
        """
        com = [self.config_11s_mesh_path, self.mesh_if]
        if self.mesh_mode == 'ibss':
            com = [self.config_mesh_path, self.mesh_if]
        subprocess.call(com, shell=False)
        if self.gw:
            print("============================================")
            gw_service = main.AutoGateway()
            gw_thread = Thread(target=gw_service.run, daemon=True)
            gw_thread.start()
            gw_thread.join()
        if str(self.concurrency).replace(' ', '') == "ap+mesh_mcc":
            # MCC mode: run mcc_settings.sh
            command = docker_path + '/mcc_settings.sh'
            subprocess.run(command, shell=True)
        elif self.bridge:
            # Bridge mode: run bridge_settings.sh
            command = docker_path + '/bridge_settings.sh'
            subprocess.run(command, shell=True)

    @property
    def create_mesh_config(self):
        """
        load mesh_conf as yaml file
        """
        print('> Loading yaml conf... ')
        try:  # may be this is redundant, since we'll always have the file.
            yaml_conf = self.util.read_yaml()
            confc = yaml_conf['client']
            confs = yaml_conf['server']
            print(confs)
        except (IOError, yaml.YAMLError) as error:
            print(error)
            sys.exit()
        config = confs['secos']
        if config['key'] == '':
            return TypeError
        mesh_vif = 'wlp1s0'
        if confc['mesh_service']:
            mesh_vif = self.util.get_interface_by_pattern(confc['mesh_inf'])
        command1 = ['iw', 'dev', mesh_vif, 'info']
        with subprocess.Popen(command1, stdout=subprocess.PIPE, shell=False) as proc1:
            command = ['awk', '/wiphy/ { printf $2 }']
            with subprocess.Popen(command, stdin=proc1.stdout, stdout=subprocess.PIPE, shell=False) as proc2:
                output, _ = proc2.communicate()
        phy_name = f"phy{output.decode().strip()}"
        mesh_ip = config['ip']
        mesh_mac = self.util.get_mac_by_interface(mesh_vif)
        mcc_channel = config['mcc_channel']
        # Create mesh service config
        Path("/opt/mesh_com").mkdir(parents=True, exist_ok=True)
        with open('/opt/mesh.conf', 'w', encoding='UTF-8') as mesh_config:
            mesh_config.write('MODE=mesh\n')
            mesh_config.write(f'IP={mesh_ip}' + '\n')
            mesh_config.write('MASK=255.255.255.0\n')
            mesh_config.write('MAC=' + config['ap_mac'] + '\n')
            mesh_config.write('KEY=' + config['key'] + '\n')
            mesh_config.write('ESSID=' + config['ssid'] + '\n')
            mesh_config.write('FREQ=' + str(config['frequency']) + '\n')
            mesh_config.write('TXPOWER=' + str(config['tx_power']) + '\n')
            mesh_config.write('COUNTRY=fi\n')
            mesh_config.write(f'MESH_VIF={mesh_vif}' + '\n')
            mesh_config.write(f'PHY={phy_name}' + '\n')
            mesh_config.write(f'MCC_CHANNEL={mcc_channel}' + '\n')
        if confc['gw_service']:
            self.gw = True
        if config['type'] == '11s':
            self.mesh_mode = "11s"
        if config['type'] == 'ibss':
            self.mesh_mode = "ibss"
        self.mesh_if = self.util.get_interface_by_pattern(confc['mesh_inf'])
        self.mesh_ip = mesh_ip
        self.mesh_mac = mesh_mac
        self.bridge = config['bridge']
        self.concurrency = config['concurrency']
        return self.mesh_ip, self.mesh_mac

    # @staticmethod
    # def set_provisioned_state(config):
    #     # TBD
    #     cmd = None
    #     # subprocess.call(cmd, shell=False)

    # @staticmethod
    def start_ap(self):
        cmd = ["hostapd", "-B", "/etc/ap.conf", "-f", "/tmp/hostapd.log"]
        subprocess.call(cmd, shell=False)
        cmd2 = ["ifconfig", self.auth_ap_if, self.auth_ap_ip]
        subprocess.call(cmd2, shell=False)
        # udhcpd.conf need to be available
        cmd3 = ["udhcpd", "/etc/udhcpd.conf"]
        subprocess.call(cmd3, shell=False)

    # to be used in the future
    # @staticmethod
    # def create_dhcpd_conf(self, start, end):
    #     config_lines = [
    #         '\n',
    #         f'\tstart="{start}"',
    #         f'\tend="{end}"',
    #         f'\tinterface="{self.auth_ap_if}"',
    #         '\toption subnet 255.255.255.0',
    #         '\toption domain local',
    #     ]
    #     config = '\n'.join(config_lines)
    #     print(config)
    #     with open("/etc/udhcpd.conf", "a+", encoding="UTF-8") as wifi:
    #         wifi.write(config)

    # def create_ap_conf(self, ssid, psk):
    #     config_lines = [
    #         '\n',
    #         'network={',
    #         f'\tinterface="{self.auth_ap_if}"',
    #         f'\tssid="{ssid}"',
    #         f'\tpsk="{psk}"',
    #         '\tkey_mgmt=WPA-PSK',
    #         '\tmode=2',
    #         '}',
    #     ]
    #
    #     config = '\n'.join(config_lines)
    #     print(config)
    #
    #     with open("/etc/ap.conf", "a+") as wifi:
    #         wifi.write(config)
    #
    # def connect_to_ap(self, config):
    #     cmd = f"wpa_supplicant - B - i {self.sta_if}- c /etc/wpa_supplicant/wpa_supplicant_sta.conf"
    #     subprocess.call(cmd, shell=False)
    #     cmd = f"udhcpc -i {self.sta_if}"
    #     subprocess.call(cmd, shell=False)
    #
    # @staticmethod
    # def create_sta_conf(ssid, psk):
    #     config_lines = ['\n', 'network={', f'\tssid="{ssid}"', f'\tpsk="{psk}"', '}']
    #
    #     config = '\n'.join(config_lines)
    #     print(config)
    #
    #     with open("/etc/wpa_supplicant/wpa_supplicant_sta.conf", "a+") as wifi:
    #         wifi.write(config)
    #
    # @staticmethod
    # def get_ip_address(ifname):
    #     with  socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    #         return socket.inet_ntoa(fcntl.ioctl(
    #             sock.fileno(),
    #             0x8915,  # SIOCGIFADDR
    #             struct.pack('256s', ifname[:15].encode())
    #         )[20:24])

    def get_password(self):
        yaml_conf = self.util.read_yaml()
        instances = yaml_conf['server']['secos']
        return instances['key'] or ''

    @staticmethod
    def create_password(wpa=False):
        '''
        get random password pf length 8 with letters, digits
        '''
        characters: str = (
            string.ascii_letters + string.digits if wpa else string.digits
        )

        return ''.join(random.choice(characters) for _ in range(10))
