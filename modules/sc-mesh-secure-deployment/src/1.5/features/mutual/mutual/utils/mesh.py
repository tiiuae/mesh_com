import subprocess
import os as osh
from os import getenv
import yaml
import random
import string
import netifaces
from pathlib import Path
from .gw import main

mesh_file_name = '../mesh_com.conf'  # maybe it's better to add the absolute path


def read_yaml():
    with open(mesh_file_name, 'r') as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)


def get_interface(pattern):
    '''
    Using this function from previous script, to obtain the mesh_interface.
    Maybe it's redundant if secure OS will have 'wlan1' as default
    '''
    interface_list = netifaces.interfaces()
    interface = filter(lambda x: pattern in x, interface_list)
    pre = list(interface)
    if not pre:
        print('> ERROR: Interface ' + pattern + ' not found!')
    else:
        return pre[0]


def get_mac(pattern):
    for interf in netifaces.interfaces():
        # TODO: what it if doesn't start with wlan???
        if interf.startswith(pattern):
            interface = interf
            mac = netifaces.ifaddresses(interface)[netifaces.AF_LINK]
            return mac[0]['addr']


def update_password(password):
    '''
    Update the mesh_conf file with the password.
    '''

    config = read_yaml()
    instances = config['server']['ubuntu']
    instances['key'] = password
    with open(mesh_file_name, 'w') as fp:
        yaml.dump(config, fp)


def get_auth_role():
    config = read_yaml()
    return config['client']['auth_role']


def get_password():
    config = read_yaml()
    instances = config['server']['ubuntu']
    return instances['key'] or ''


def create_mesh(ID):  # Get the mesh_com config
    '''
    load mesh_conf as yaml file
    '''
    print('> Loading yaml conf... ')
    try:  # may be this is redundant, since we'll always have the file.
        yaml_conf = read_yaml()
        confc = yaml_conf['client']
        confs = yaml_conf['server']
        debug = yaml_conf['debug']
        print(confs)
    except (IOError, yaml.YAMLError) as error:
        print(error)
        exit()
    resp = confs['ubuntu']
    if confc['mesh_service']:
        mesh_vif = get_interface(confc['mesh_inf'])
        cmd = "iw dev " + mesh_vif + " info | awk '/wiphy/ {printf \"phy\" $2}'"
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
        phy_name = proc.communicate()[0].decode('utf-8').strip()
    mesh_ip = confs['ubuntu']['ip']
    mesh_mac = get_mac(mesh_vif)
    # Create mesh service config
    Path("/opt/mesh_com").mkdir(parents=True, exist_ok=True)
    with open('/opt/mesh.conf', 'w', encoding='UTF-8') as mesh_config:
        mesh_config.write('MODE=mesh\n')
        mesh_config.write('IP=' + mesh_ip + '\n')
        mesh_config.write('MASK=255.255.255.0\n')
        mesh_config.write('MAC=' + resp['ap_mac'] + '\n')
        mesh_config.write('KEY=' + resp['key'] + '\n')
        mesh_config.write('ESSID=' + resp['ssid'] + '\n')
        mesh_config.write('FREQ=' + str(resp['frequency']) + '\n')
        mesh_config.write('TXPOWER=' + str(resp['tx_power']) + '\n')
        mesh_config.write('COUNTRY=fi\n')
        mesh_config.write('MESH_VIF=' + mesh_vif + '\n')
        mesh_config.write('PHY=' + phy_name + '\n')
    # Are we a gateway node? If we are we need to set up the routes
    if confc['gw_service']:
        print("============================================")
        main.AutoGateway()
    # Set hostname
    if confc['set_hostname']:
        print('> Setting hostname...')
        command = ['hostname', 'node', ID]
        subprocess.call(command, shell=False)
        #subprocess.call('echo ' + '"' + address + '\t' + 'node' + str(nodeId) + '"' + ' >' + '/etc/hosts', shell=True)
        command2 = ['echo', mesh_ip, 'node', ID, '>', '/etc/hosts']
        subprocess.call(command2, shell=False)
    execution_ctx = osh.environ.get('EXECUTION_CTX')
    print("EXECUTION CTX:")
    print(execution_ctx)
    # if execution_ctx != "docker":
    #     if conf['disable_networking']:
    #         subprocess.call('sudo nmcli networking off', shell=True)
    #         subprocess.call('sudo systemctl stop network-manager.service', shell=True)
    #         subprocess.call('sudo systemctl disable network-manager.service', shell=True)
    #         subprocess.call('sudo systemctl disable wpa_supplicant.service', shell=True)
    #         # subprocess.call('pkill wpa_supplicant', shell=True)
    #         # subprocess.call('pkill -f "/var/run/wpa_supplicant-" 2>/dev/null', shell=True)
    # Copy mesh service to /etc/systemd/system/
    if confc['mesh_service']:
        mesh_interface = get_interface(confc['mesh_inf'])
        config_11s_mesh_path = osh.path.join(getenv("MESH_COM_ROOT", ""), "src/bash/conf-11s-mesh.sh")
        config_mesh_path = osh.path.join(getenv("MESH_COM_ROOT", ""), "src/bash/conf-mesh.sh")
        print(config_mesh_path)
        if resp['type'] == '11s':
            com = [config_11s_mesh_path, mesh_interface]
        if resp['type'] == 'ibss':
            com = [config_mesh_path,  mesh_interface]
        subprocess.call(com, shell=False)
    return mesh_ip, mesh_mac


def create_password(WPA=False):
    '''
    get random password pf length 8 with letters, digits
    '''
    if WPA:
        characters: str = string.ascii_letters + string.digits
    else:  # WEP
        characters: str = string.digits
    return ''.join(random.choice(characters) for i in range(10))
