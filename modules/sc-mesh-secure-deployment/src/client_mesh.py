#!/usr/bin/env python

import argparse
import json
import yaml
import subprocess
import os as osh
from os import getenv
import netifaces
from getmac import get_mac_address
import requests
from termcolor import colored
from pathlib import Path
import sys
import primitives as pri
sys.path.append("/opt/mesh_com/modules/sc-mesh-secure-deployment/src/gw")
from gw import main
# Construct the argument parser
ap = argparse.ArgumentParser()

# Add the arguments to the parser
ap.add_argument("-s", "--server", required=True, help="Server IP:Port Address. Ex: 'http://192.168.15.14:5000'")
ap.add_argument("-c", "--certificate", required=True)
ap.add_argument("-t", "--test", required=False, default=False, action='store_true')
ap.add_argument("-m", "--mode", required=True)
args = ap.parse_args()
# Get the mesh_com config
mesh_mode = args.mode

# Get the mesh_com config
if mesh_mode == 'ibss':
    conf_path = "src/mesh_com.conf"
elif mesh_mode == '11s':
    conf_path = "src/mesh_com_11s.conf"

print(getenv("MESH_COM_ROOT", ""))
config_path = osh.path.join(getenv("MESH_COM_ROOT", ""), conf_path)
print('> Loading yaml conf... ')
try:
    yaml_conf = yaml.safe_load(open(config_path, 'r'))
    conf = yaml_conf['client']
    debug = yaml_conf['debug']
    print(conf)
except (IOError, yaml.YAMLError) as error:
    print(error)
    sys.exit()

# Connect to server
URL = args.server
print('> Connecting to server: ' + str(URL))
local_cert = args.certificate
root_cert = "/etc/ssl/certs/root_cert.der"


# Function for test case
def Client_Test(**arg):
    if args.test:
        with open("/opt/container-data/mesh/mesh_com/modules/sc-mesh-secure-deployment/src/testclient1.txt", "w",
                  encoding='UTF-8') as testfile:
            if arg["color"] == "Green":
                testfile.write("True")
                testfile.close()
            elif arg["Color"] == "Red":
                testfile.write("False")
                testfile.close()


def Client_Mac(**arg):
    if args.test:
        with open("/opt/container-data/mesh/mesh_com/modules/sc-mesh-secure-deployment/src/testclientmac.txt", "w",
                  encoding='UTF-8') as testfile:
            if arg["macs"] is not None:
                testfile.write(str(arg["macs"]))
                testfile.close()


def get_os():  # this is not being used
    with subprocess.Popen(['lsb_release', '-a'], stdout=subprocess.PIPE, stderr=subprocess.PIPE) as proc:
        out, _ = proc.communicate()
        for element in out.split():
            aux = element.decode('utf-8')
            if 'Ubuntu' in aux:
                oss = aux
        return oss


def is_sec_os():  # this is not being used
    execution_ctx = osh.environ.get('EXECUTION_CTX')
    if execution_ctx == "docker":
        return "secos" if osh.environ.get('HOSTNAME') == "br_hardened" else get_os()
    return ""


def get_data(oss='secos'):
    message = f'/api/add_message/{oss}'
    dig, sig = pri.hashSig(root_cert)
    with open(local_cert, 'rb') as lo_cert:
        resp = requests.post(URL + message,
                             files={'key': lo_cert,
                                    'sig': bytes(sig),
                                    'dig': dig})
        if resp.content == b'Not Valid Certificate':
            print(colored('Not Valid Certificate', 'red'))
            sys.exit()
        elif debug:
            print(f'> Encrypted message: {str(resp.content)}')
    return pri.decrypt_response(resp.content)


def get_signature():
    message = '/signature'
    resp = requests.post(URL + message)
    print(resp.content)
    return resp.content


def get_digest():
    message = '/digest'
    resp = requests.post(URL + message)
    return resp.content


def serializing(new_list):
    joined_list = ''.join(new_list)
    return joined_list.replace("\'", '"')


def get_interface(pattern):
    interface_list = netifaces.interfaces()
    interface = filter(lambda x: pattern in x, interface_list)
    pre = list(interface)
    if pre:
        return str(pre[0])

    print(f'> ERROR: Interface {pattern} not found!')
    return False


def conf_gw():
    print('> Configuring gateway node ')
    # Create Gateway Service
    subprocess.call('/opt/mesh_com/modules/sc-mesh-secure-deployment/src/bash/conf-gw.sh', shell=False)


def ubuntu_node(gateway):  # this is not being used
    print('> Configuring Ubuntu mesh node...')
    # Create default route service
    command = ['src/bash/conf-route.sh', gateway]
    subprocess.call(command, shell=False)


def create_config(respo):
    resp = json.loads(bytes(respo))
    print('> Interfaces: ' + str(resp))
    address = resp['addr']
    if conf['mesh_service']:
        mesh_vif = get_interface(conf['mesh_inf'])
        cmd = "iw dev " + mesh_vif + " info | awk '/wiphy/ {printf \"phy\" $2}'"
        with subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True) as proc:
            phy_name = proc.communicate()[0].decode('utf-8').strip()
    # Create mesh service config
    Path("/opt/mesh_com").mkdir(parents=True, exist_ok=True)
    with open('/opt/mesh.conf', 'w', encoding='UTF-8') as mesh_config:
        mesh_config.write('MODE=mesh\n')
        mesh_config.write('IP=' + address + '\n')
        mesh_config.write('MASK=255.255.255.0\n')
        mesh_config.write('MAC=' + resp['ap_mac'] + '\n')
        mesh_config.write('KEY=' + resp['key'] + '\n')
        mesh_config.write('ESSID=' + resp['ssid'] + '\n')
        mesh_config.write('FREQ=' + str(resp['frequency']) + '\n')
        mesh_config.write('TXPOWER=' + str(resp['tx_power']) + '\n')
        mesh_config.write('COUNTRY=fi\n')
        mesh_config.write('MESH_VIF=' + mesh_vif + '\n')
        mesh_config.write('PHY=' + phy_name + '\n')
        mesh_config.write('ROUTING=' + str(res['routing_protocol']) + '\n')
    # Are we a gateway node? If we are we need to set up the routes
    if conf['gw_service']:
        print("============================================")
        main.AutoGateway()
    # Set hostname
    if conf['set_hostname']:
        print('> Setting hostname...')
        nodeId = int(resp['addr'].split('.')[-1]) - 1  # the IP is sequential, then it gives the nodeId.
        command = ['hostname', 'node', str(nodeId)]
        subprocess.call(command, shell=False)
        #subprocess.call('echo ' + '"' + address + '\t' + 'node' + str(nodeId) + '"' + ' >' + '/etc/hosts', shell=True)
        command2 = ['echo', str(address), 'node', str(nodeId), '>', '/etc/hosts']
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
    if conf['mesh_service']:
        mesh_interface = get_interface(conf['mesh_inf'])
        config_11s_mesh_path = osh.path.join(getenv("MESH_COM_ROOT", ""), "src/bash/conf-11s-mesh.sh")
        config_mesh_path = osh.path.join(getenv("MESH_COM_ROOT", ""), "src/bash/conf-mesh.sh")
        print(config_mesh_path)
        if resp['type'] == '11s':
            com = [config_11s_mesh_path, mesh_interface]
        if resp['type'] == 'ibss':
            com = [config_mesh_path,  mesh_interface]
        subprocess.call(com, shell=False)


if __name__ == "__main__":
    res = get_data('secos')
    sig = get_signature()
    if pri.verify_certificate(sig, 'root', get_digest(), root_cert):
        print(colored('> Valid Server Certificate', 'green'))
        Client_Test(color="Green")
        mac = get_mac_address(interface=get_interface(conf['mesh_inf']))
        Client_Mac(macs=mac)
        response = requests.post(URL + '/mac/' + mac)
        create_config(res)
    else:
        print(colored("Not Valid Server Certificate", 'red'))
        Client_Test(color="Red")
        sys.exit(0)
