#!/usr/bin/python

import argparse
import json
import subprocess
import time

import netifaces
import requests
from termcolor import colored
from pathlib import Path

# Construct the argument parser
ap = argparse.ArgumentParser()

# Add the arguments to the parser
ap.add_argument("-s", "--server", required=True, help="Server IP:Port Address. Ex: 'http://192.168.15.14:5000'")
ap.add_argument("-c", "--certificate", required=True)
args = ap.parse_args()

URL = args.server
print(URL)

def get_os():
    proc = subprocess.Popen(['lsb_release', '-a'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = proc.communicate()
    for element in out.split():
        aux = element.decode('utf-8')
        if 'Ubuntu' in aux:
            os = aux
    return os


def get_data(cert_file, os):
    message = '/api/add_message/' + os
    response = requests.post(URL + message,
                             files={'key': open(cert_file, 'rb')})
    if response.content == b'Not Valid Certificate':
        print(colored('Not Valid Certificate', 'red'))
        exit()
    else:
        print('Encrypted Message: ' + str(response.content))
        with open('payload.enc', 'wb') as file:
            file.write(response.content)


def decrypt_reponse():  # assuming that data is on a file called payload.enc generated on the function get_data
    proc = subprocess.Popen(['src/ecies_decrypt', args.certificate], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = proc.communicate()
    aux_list = [element.decode() for element in out.split()]
    new_list = aux_list[40:64]
    joined_list = ''.join(new_list)
    output_dict = joined_list.replace("\'", '"')
    print('Decrypted Message: ', output_dict)  # res =  json.loads(output_dict)
    return output_dict


def create_config_openwrt(response):
    res = json.loads(response)
    if res['gateway']:
        config_file = open('/etc/config/lime-node', 'a+')
    else:
        nodeId = int(res['addr'].split('.')[-1]) - 1  # the IP is sequential, then it gaves me the nodeId.
        config_file = open('/etc/config/lime-node', 'w')
        config_file.write('config lime ' + "'system'" + '\n')
        config_file.write('\t' + 'option hostname ' + "'" + 'node' + str(nodeId) + "'" + '\n')
        config_file.write('\t' + 'option firstbootwizard_configured ' + "'" + 'true' + "'" + '\n\n')
    config_file.write('config wifi radio1' + '\n')
    config_file.write('\t' + 'list modes ' + "'ieee80211s'" + '\n')
    config_file.write('\t' + 'option ieee80211s_mesh_fwding ' + "'" + res['ieee80211s_mesh_fwding'] + "'" + '\n')
    config_file.write('\t' + 'option ieee80211s_mesh_id ' + "'" + res['ieee80211s_mesh_id'] + "'" + '\n')
    config_file.write('\t' + 'option channel ' + "'" + str(res['channel']) + "'" + '\n\n')
    config_file.write('config lime ' + "'network'" + '\n')
    config_file.write('\t' + 'list protocols ' + "'ieee80211s'" + '\n')
    config_file.write('\t' + 'list protocols ' + "'lan'" + '\n')
    config_file.write('\t' + 'list protocols ' + "'anygw'" + '\n')
    config_file.write('\t' + 'list protocols ' + "'" + 'batadv:' + res['batadv'] + "'" + '\n')
    config_file.write('\t' + 'list protocols ' + "'" + 'babeld:' + res['babeld'] + "'" '\n')
    config_file.write('\t' + 'list protocols ' + "'" + 'bmx7:' + res['bmx7'] + "'" + '\n')
    config_file.write('\t' + 'option main_ipv4_address ' + "'" + res['addr'] + '/24' + "'")
    config_file.close()
    subprocess.call('lime-config', shell=True)
    time.sleep(2)
    subprocess.call('reboot', shell=True)

def get_ap_interface():
    interface_list = netifaces.interfaces()
    interface = filter(lambda x: 'wla' in x or 'wlp' in x, interface_list)
    return list(interface)[0]

def get_mesh_interface():
    interface_list = netifaces.interfaces()
    interface = filter(lambda x: 'wlx' in x, interface_list)
    return list(interface)[0]


def ubuntu_gw(ap_inf):
    print('> Configuring Ubuntu gateway node...')
    # Create Gateway Service
    subprocess.call('sudo cp ../../common/scripts/mesh-gw.sh /usr/local/bin/.', shell=True)
    subprocess.call('sudo chmod 744 /usr/local/bin/mesh-gw.sh', shell=True)
    subprocess.call('sudo cp services/gw@.service /etc/systemd/system/.', shell=True)
    subprocess.call('sudo chmod 644 /etc/systemd/system/gw@.service', shell=True)
    subprocess.call('sudo systemctl enable gw@' + str(ap_inf) + '.service', shell=True)
    # Auto connect wlx to AP at boot using wpa_supplicant
    copy = 'sudo cp tools/wpa_tools/access_point.conf /etc/wpa_supplicant/wpa_supplicant-' + str(ap_inf) + '.conf'
    subprocess.call(copy, shell=True)
    subprocess.call('chmod 600 /etc/wpa_supplicant/wpa_supplicant-' + str(ap_inf) + '.conf', shell=True)
    subprocess.call('sudo systemctl enable wpa_supplicant@' + str(ap_inf) + '.service', shell=True)
    
def ubuntu_node(gateway):
    print('> Configuring Ubuntu mesh node...')
    # Add gateway to mesh conf
    with open('/etc/mesh_com/mesh.conf', 'a+') as mesh_config:
        mesh_config.write('GW=' + gateway + '\n')
    subprocess.call('route add default gw ' + gateway + ' bat0', shell=True)
    # Create default route service
    subprocess.call('sudo cp ../../common/scripts/mesh-default-gw.sh /usr/local/bin/.', shell=True)
    subprocess.call('sudo chmod 744 /usr/local/bin/mesh-default-gw.sh', shell=True)
    subprocess.call('sudo cp services/default@.service /etc/systemd/system/.', shell=True)
    subprocess.call('sudo chmod 644 /etc/systemd/system/default@.service', shell=True)
    subprocess.call('sudo systemctl enable default@' + gateway + '.service', shell=True)

def create_config_ubuntu(response):
    res = json.loads(response)
    print('Interfaces: ' + str(res))
    address = res['addr']
    # Create mesh service config
    Path("/etc/mesh_com").mkdir(parents=True, exist_ok=True)
    with open('/etc/mesh_com/mesh.conf', 'w') as mesh_config:
        mesh_config.write('MODE=mesh\n')
        mesh_config.write('IP=' + address + '\n')
        mesh_config.write('MASK=255.255.255.0\n')
        mesh_config.write('MAC=' + res['ap_mac'] + '\n')
        mesh_config.write('KEY=' + res['key'] + '\n')
        mesh_config.write('ESSID=' + res['ssid'] + '\n')
        mesh_config.write('FREQ=' + res['frequency'] + '\n')
        mesh_config.write('TXPOWER=' + res['tx_power'] + '\n')
        mesh_config.write('COUNTRY=fi\n')
        mesh_config.write('PHY=phy1\n')
    # Are we a gateway node? If we are we need to set up the routes
    if res['gateway']:
        ap_interface = get_ap_interface()
        ubuntu_gw(ap_interface)
    else:
        # We aren't a gateway node, set up the default route (to gw) service
        prefix = address.split('.')[:-1]
        prefix = '.'.join(prefix)
        mesh_gateway = prefix + '.2'
        ubuntu_node(mesh_gateway)
    # Set hostname
    nodeId = int(res['addr'].split('.')[-1]) - 1  # the IP is sequential, then it gives the nodeId.
    command_hostname = 'sudo hostnamectl set-hostname node' + str(nodeId)
    subprocess.call(command_hostname, shell=True)
    command_hostname_host = 'echo ' + '"' + address + '\t' + 'node' + str(nodeId) + '"' + ' >' + '/etc/hosts'
    subprocess.call(command_hostname_host, shell=True)
    # Final settings
    subprocess.call('sudo nmcli networking off', shell=True)
    subprocess.call('sudo systemctl stop network-manager.service', shell=True)
    subprocess.call('sudo systemctl disable network-manager.service', shell=True)
    subprocess.call('sudo systemctl disable wpa_supplicant.service', shell=True)
    # Copy mesh service to /etc/systemd/system/
    mesh_interface = get_mesh_interface()
    subprocess.call('sudo cp ../../common/scripts/mesh-ibss.sh /usr/local/bin/.', shell=True)
    subprocess.call('sudo chmod 744 /usr/local/bin/mesh-ibss.sh', shell=True)
    subprocess.call('sudo cp services/mesh@.service /etc/systemd/system/.', shell=True)
    subprocess.call('sudo chmod 664 /etc/systemd/system/mesh@.service', shell=True)
    subprocess.call('sudo systemctl enable mesh@' + mesh_interface + '.service', shell=True)
    # Ensure our nameserver persists as 8.8.8.8
    subprocess.call('sudo cp conf/resolved.conf /etc/systemd/resolved.conf', shell=True)
    time.sleep(2)
    subprocess.call('reboot', shell=True)


if __name__ == "__main__":
    os = get_os()
    get_data(args.certificate, os)
    res = decrypt_reponse()
    if os == 'Ubuntu':
        create_config_ubuntu(res)
    if os == 'openwrt':
        create_config_openwrt(res)
