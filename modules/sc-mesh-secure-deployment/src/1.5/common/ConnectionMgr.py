from pathlib import Path
from os import getenv
import subprocess
import os as osh
import yaml
import socket
import fcntl
import struct
from .gw import main
import random
import string
from threading import Thread

from .utils import Utils

ut = Utils()




class ConnectionMgr:
    def __init__(self):
        self.config_11s_mesh_path = osh.path.join(getenv("MESH_COM_ROOT", ""), "../../../bash/conf-11s-mesh.sh")
        self.config_mesh_path = osh.path.join(getenv("MESH_COM_ROOT", ""), "../../../bash/conf-mesh.sh")
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

    def start_mesh(self):  # Get the mesh_com config
        """
        start mesh service(11s/ibss)
        """
        if self.mesh_mode == '11s':
            com = [self.config_11s_mesh_path, self.mesh_if]
        elif self.mesh_mode == 'ibss':
            com = [self.config_mesh_path, self.mesh_if]
        subprocess.call(com, shell=False)

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
            exit()
        config = confs['secos']
        if confc['mesh_service']:
            mesh_vif = self.util.get_interface_by_pattern(confc['mesh_inf'])
            cmd = f"iw dev {mesh_vif}" + " info | awk '/wiphy/ {printf \"phy\" $2}'"
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)  # check how to transform it Shell=False
            phy_name = proc.communicate()[0].decode('utf-8').strip()
        mesh_ip = confs['ubuntu']['ip']
        mesh_mac = self.util.get_mac_by_interface(mesh_vif)
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
        if confc['gw_service']:
            print("============================================")
            Thread(target=main.AutoGateway(), daemon=True).start()
        if config['type'] == '11s':
            self.mesh_mode = "11s"
        if config['type'] == 'ibss':
            self.mesh_mode = "ibss"
        self.mesh_if = self.util.get_interface_by_pattern(confc['mesh_inf'])
        self.mesh_ip = mesh_ip
        self.mesh_mac = mesh_mac
        return self.mesh_ip, self.mesh_mac

    @staticmethod
    def set_provisioned_state(config):
        # TBD
        cmd = None
        # subprocess.call(cmd, shell=False)

    @staticmethod
    def create_dhcpd_conf(self, start, end):
        config_lines = [
            '\n',
            '\tstart="{}"'.format(start),
            '\tend="{}"'.format(end),
            '\tinterface="{}"'.format(self.auth_ap_if),
            '\toption subnet 255.255.255.0',
            '\toption domain local',
        ]
        config = '\n'.join(config_lines)
        print(config)

        with open("/etc/udhcpd.conf", "a+") as wifi:
            wifi.write(config)

    # @staticmethod
    def start_ap(self, config):
        cmd = ["hostapd", "-B", "/etc/ap.conf", "-f", "/tmp/hostapd.log"]
        subprocess.call(cmd, shell=False)
        cmd2 = ["ifconfig", self.auth_ap_if, self.auth_ap_ip]
        subprocess.call(cmd2, shell=False)
        # udhcpd.conf need to be available
        cmd3 = ["udhcpd", "/etc/udhcpd.conf"]
        subprocess.call(cmd3, shell=False)

    def create_ap_conf(self, ssid, psk):
        config_lines = [
            '\n',
            'network={',
            '\tinterface="{}"'.format(self.auth_ap_if),
            '\tssid="{}"'.format(ssid),
            '\tpsk="{}"'.format(psk),
            '\tkey_mgmt=WPA-PSK',
            '\tmode=2',
            '}'
        ]

        config = '\n'.join(config_lines)
        print(config)

        with open("/etc/ap.conf", "a+") as wifi:
            wifi.write(config)

    def connect_to_ap(self, config):
        cmd = f"wpa_supplicant - B - i {self.sta_if}- c /etc/wpa_supplicant/wpa_supplicant_sta.conf"
        subprocess.call(cmd, shell=False)
        cmd = f"udhcpc -i {self.sta_if}"
        subprocess.call(cmd, shell=False)

    @staticmethod
    def create_sta_conf(ssid, psk):
        config_lines = [
            '\n',
            'network={',
            '\tssid="{}"'.format(ssid),
            '\tpsk="{}"'.format(psk),
            '}'
        ]

        config = '\n'.join(config_lines)
        print(config)

        with open("/etc/wpa_supplicant/wpa_supplicant_sta.conf", "a+") as wifi:
            wifi.write(config)

    @staticmethod
    def get_ip_address(ifname):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(fcntl.ioctl(
            s.fileno(),
            0x8915,  # SIOCGIFADDR
            struct.pack('256s', ifname[:15].encode())
        )[20:24])

    def get_password(self):
        yaml_conf = self.util.read_yaml()
        instances = yaml_conf['server']['ubuntu']
        return instances['key'] or ''

    @staticmethod
    def create_password(WPA=False):
        '''
        get random password pf length 8 with letters, digits
        '''
        characters: str = (
            string.ascii_letters + string.digits if WPA else string.digits
        )

        return ''.join(random.choice(characters) for _ in range(10))
