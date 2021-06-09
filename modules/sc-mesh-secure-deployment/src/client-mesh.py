#!/usr/bin/python

import argparse
import json
import subprocess
import time
import hashlib
import random

import netifaces
from getmac import get_mac_address
import requests
from termcolor import colored
from pathlib import Path
import os
import psutil

# Construct the argument parser
ap = argparse.ArgumentParser()

# Add the arguments to the parser
ap.add_argument("-s", "--server", required=True, help="Server IP:Port Address. Ex: 'http://192.168.15.14:5000'")
ap.add_argument("-c", "--certificate", required=True)
args = ap.parse_args()

URL = args.server
print('> Connecting to server: ' + str(URL))


def get_os():
    proc = subprocess.Popen(['lsb_release', '-a'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = proc.communicate()
    for element in out.split():
        aux = element.decode('utf-8')
        if 'Ubuntu' in aux:
            OS = aux
    return OS


def get_data(cert_file, os):
    message = '/add_message/' + os
    response = requests.post(URL + message,
                             files={'key': open(cert_file, 'rb')})
    if response.content == b'Not Valid Certificate':
        print(colored('Not Valid Certificate', 'red'))
        exit()
    else:
        print('> Received encrypted message: ' + str(response.content))
        with open('payload.enc', 'wb') as file:
            file.write(response.content)


def decrypt_response():  # assuming that data is on a file called payload.enc generated on the function get_data
    proc = subprocess.Popen(['src/ecies_decrypt', args.certificate], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = proc.communicate()
    aux_list = [element.decode() for element in out.split()]
    server_cert = aux_list[:39]
    new_list = aux_list[40:66]
    output_dict = serializing(new_list)
    print('> Decrypted Message: ', output_dict)  # res =  json.loads(output_dict)

    return output_dict, server_cert


def serializing(new_list):
    joined_list = ''.join(new_list)
    return joined_list.replace("\'", '"')


def verify_certificate(new):
    """
    Here we are validating the hash of the certificate. This is giving us the integrity of the file, not if the
    certificate is valid. To validate if the certificate is valid, we need to verify the features of the certificate
    such as NotBefore, notAfter, crl file and its signature, issuer validity, if chain is there then this for all but
    for this we need to use x509 certificates.

    """
    proc = subprocess.Popen(['src/ecies_decrypt', args.certificate], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = proc.communicate()
    local = [element.decode() for element in out.split()]
    local_cert = local[:39]
    old_aux = serializing(local_cert)
    new_aux = serializing(new)
    old_md5 = hashlib.md5(old_aux.encode('utf-8')).hexdigest()
    new_md5 = hashlib.md5(new_aux.encode('utf-8')).hexdigest()

    return old_md5 == new_md5


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
    subprocess.call('sudo cp conf/ap.conf /etc/wpa_supplicant/wpa_supplicant-' + str(ap_inf) + '.conf', shell=True)
    subprocess.call('chmod 600 /etc/wpa_supplicant/wpa_supplicant-' + str(ap_inf) + '.conf', shell=True)
    subprocess.call('sudo systemctl enable wpa_supplicant@' + str(ap_inf) + '.service', shell=True)


def ubuntu_node(gateway):
    print('> Configuring Ubuntu mesh node...')
    # Create default route service
    subprocess.call('route add default gw ' + gateway + ' bat0', shell=True)  # FIXME: Is this line necessary?
    subprocess.call('sudo cp ../../common/scripts/mesh-default-gw.sh /usr/local/bin/.', shell=True)
    subprocess.call('sudo chmod 744 /usr/local/bin/mesh-default-gw.sh', shell=True)
    subprocess.call('sudo cp services/default@.service /etc/systemd/system/.', shell=True)
    subprocess.call('sudo chmod 644 /etc/systemd/system/default@.service', shell=True)
    subprocess.call('sudo systemctl enable default@' + gateway + '.service', shell=True)


def authServer(addr):
    ip_prefix = '.'.join(addr.split('.')[0:2])
    new_ip = ip_prefix + '.' + str(random.randint(1, 254)) + '.1'  # TODO: Currently random number, this is gonna cause collision still
    requests.post(URL+'/authServer/'+new_ip)
    # to make it persistent
    name = os.listdir('/home')[-1]
    with open('/etc/mesh_com/server.conf', 'w') as mesh_config:
        mesh_config.write('NAME=' + name + '\n')
        mesh_config.write('IP=' + addr + '\n')
    subprocess.call('sudo cp ../../common/scripts/mesh-server.sh /usr/local/bin/.', shell=True)
    subprocess.call('sudo chmod 744 /usr/local/bin/mesh-server.sh', shell=True)
    subprocess.call('sudo cp services/mesh-server.service /etc/systemd/system/.', shell=True)
    subprocess.call('sudo chmod 644 /etc/systemd/system/mesh-server.service', shell=True)
    subprocess.call('sudo systemctl enable mesh-server.service', shell=True)


def create_config_ubuntu(response):
    res = json.loads(response)
    print('> Interfaces: ' + str(res))
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
    if res['gateway']:
        authServer(address)
    if int(res['addr'].split('.')[-1]) == 1:
        if ("mesh-server.sh" not in p.name() for p in psutil.process_iter()):
            # this mean we have a server running on the same node
            authServer(address)
        subprocess.call('sudo cp ../../common/scripts/mesh-ap-connect.sh /usr/local/bin/.', shell=True)
        subprocess.call('sudo chmod 744 /usr/local/bin/mesh-ap-connect.sh', shell=True)
        subprocess.call('sudo cp services/connect_ap.service /etc/systemd/system/.', shell=True)
        subprocess.call('sudo chmod 664 /etc/systemd/system/connect_ap.service', shell=True)
        subprocess.call('sudo systemctl enable connect_ap.service', shell=True)
        subprocess.call('reboot', shell=True)
    # Final settings
    subprocess.call('sudo nmcli networking off', shell=True)
    #subprocess.call('sudo systemctl stop network-manager.service', shell=True)
    #subprocess.call('sudo systemctl disable network-manager.service', shell=True)
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
    # subprocess.call('reboot', shell=True)
    # Are we an Auth-server node?


if __name__ == "__main__":
    OS = get_os()
    local_cert = args.certificate
    get_data(local_cert, OS)
    res, server_cert = decrypt_response()
    if verify_certificate(server_cert):
        print(colored('> Valid Server Certificate', 'green'))
        mac = get_mac_address(interface=get_mesh_interface())
        response = requests.post(URL + '/mac/' + mac)
        if OS == 'Ubuntu':
            create_config_ubuntu(res)
        else:
            print(colored('Wrong OS'), 'red')
            exit(0)
    else:
        print(colored("Not Valid Server Certificate", 'red'))
        exit(0)
