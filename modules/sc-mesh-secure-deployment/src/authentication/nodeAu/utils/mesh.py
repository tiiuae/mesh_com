import subprocess
import os as osh
import ruamel.yaml
import random
import string
import netifaces
from pathlib import Path


mesh_file_name = '../mesh_com.conf'  #maybe it's better to add the absolute path


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

    config, ind, bsi = ruamel.yaml.util.load_yaml_guess_indent(open(mesh_file_name))
    instances = config['server']['ubuntu']
    instances['key'] = password
    yaml = ruamel.yaml.YAML()
    yaml.indent(mapping=ind, sequence=ind, offset=bsi)
    with open(mesh_file_name, 'w') as fp:
        yaml.dump(config, fp)


def get_auth_role():
    config, ind, bsi = ruamel.yaml.util.load_yaml_guess_indent(open(mesh_file_name))
    return config['client']['auth_role']


def get_password():
    config, ind, bsi = ruamel.yaml.util.load_yaml_guess_indent(open(mesh_file_name))
    instances = config['server']['ubuntu']
    return instances['key'] or ''


def create_mesh(ID):    # Get the mesh_com config
    '''
    load mesh_conf as yaml file
    '''
    print('> Loading yaml conf... ')
    try:  # may be this is redundant, since we'll always have the file.
        yaml = ruamel.yaml.YAML(typ='safe')  # default, if not specified, is 'rt' (round-trip)
        doc = open(mesh_file_name, 'r')
        yaml_conf = yaml.load(doc.read())
        confc = yaml_conf['client']
        confs = yaml_conf['server']
        debug = yaml_conf['debug']
        print(confs)
    except (IOError, yaml.YAMLError) as error:
        print(error)
        exit()
    config_mesh = confs['ubuntu']
    if confc['mesh_service']:
        mesh_vif = get_interface(confc['mesh_inf'])
        cmd = "iw dev " + mesh_vif + " info | awk '/wiphy/ {printf \"phy\" $2}'"
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
        phy_name = proc.communicate()[0].decode('utf-8').strip()
    mesh_ip = confs['ubuntu']['ip']
    mesh_mac = get_mac(mesh_vif)
    # Create mesh service config
    Path("/etc/mesh_com").mkdir(parents=True, exist_ok=True)
    with open('/etc/mesh_com/mesh.conf', 'w') as mesh_config:
        mesh_config.write('MODE=mesh\n')
        mesh_config.write('IP=' + mesh_ip + '\n')
        mesh_config.write('MASK=255.255.255.0\n')
        mesh_config.write('MAC=' + config_mesh['ap_mac'] + '\n')
        mesh_config.write('KEY=' + config_mesh['key'] + '\n')
        mesh_config.write('ESSID=' + config_mesh['ssid'] + '\n')
        mesh_config.write('FREQ=' + str(config_mesh['frequency']) + '\n')
        mesh_config.write('TXPOWER=' + str(config_mesh['tx_power']) + '\n')
        mesh_config.write('COUNTRY=fi\n')
        mesh_config.write('MESH_VIF=' + mesh_vif + '\n')
        mesh_config.write('PHY=' + phy_name + '\n')
    # Are we a gateway node? If we are we need to set up the routes
    # if confc['gw_service']:
    #     print("============================================")
    #     gw_inf = get_interface(confc['gw_inf'])
    #     ubuntu_gw(gw_inf)
    # elif confc['dflt_service']:
    #     # We aren't a gateway node, set up the default route (to gw) service
    #     prefix = address.split('.')[:-1]
    #     prefix = '.'.join(prefix)
    #     mesh_gateway = prefix + '.2'
    #     ubuntu_node(mesh_gateway)
    if confc['set_hostname']:
        print('> Setting hostname...')
        subprocess.call('hostname node' + ID, shell=True)
        subprocess.call('echo ' + '"' + config_mesh['ip'] + '\t' + 'node' + ID + '"' + ' >' + '/etc/hosts', shell=True)
    execution_ctx = osh.environ.get('EXECUTION_CTX')
    print("EXECUTION CTX:")
    print(execution_ctx)
    if execution_ctx != "docker" and confc['disable_networking']:
        subprocess.call('sudo nmcli networking off', shell=True)
        subprocess.call('sudo systemctl stop network-manager.service', shell=True)
        subprocess.call('sudo systemctl disable network-manager.service', shell=True)
        subprocess.call('sudo systemctl disable wpa_supplicant.service', shell=True)
    # Copy mesh service to /etc/systemd/system/
    if confc['mesh_service']:
        mesh_interface = get_interface(confc['mesh_inf'])
        if confs['ubuntu']['type'] == '11s':
            subprocess.call('../../bash/conf-11s-mesh.sh ' + mesh_interface, shell=True)
        if confs['ubuntu']['type'] == 'ibss':
            subprocess.call('../../bash/conf-mesh.sh ' + mesh_interface, shell=True)
    return mesh_ip, mesh_mac


def create_password(WPA=False):
    '''
    get random password pf length 8 with letters, digits
    '''
    if WPA:
        characters: str = string.ascii_letters + string.digits
    else: #WEP
        characters: str = string.digits
    return ''.join(random.choice(characters) for i in range(10))