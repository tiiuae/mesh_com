#!/usr/bin/python

from flask import Flask, request, json
from getmac import get_mac_address
import subprocess
import netifaces
import pandas as pd
import argparse
from termcolor import colored
import hashlib
import os

# Construct the argument parser
ap = argparse.ArgumentParser()

# Add the arguments to the parser
ap.add_argument("-c", "--certificate", required=True)
ap.add_argument("-a", "--address", help='SubNetwork address to provision, example: 10.10.0.1', required=False,
                default='10.0.0.0')
args = ap.parse_args()

app = Flask(__name__)

aux_openwrt = {'batadv': '120', 'babeld': '17',
               'ieee80211s_mesh_id': 'ssrc',
               'bmx7': '18',
               'ieee80211s_mesh_fwding': '0',
               'channel': 5, 'gateway': False}

aux_ubuntu = {
    "api_version": 1,  # "interface version for future purposes"
    "ssid": "gold",  # "0-32 octets, UTF-8, shlex.quote chars limiting"
    "key": "1234567890",  # "key for the network"
    "enc": "wep",  # "encryption (wep, wpa2, wpa3, sae)"
    "ap_mac": "00:11:22:33:44:55",  # "bssid for mesh network"
    "country": "fi",  # "Country code, sets tx power limits and supported channels"
    "frequency": "5180",  # 5180 wifi channel frequency, depends on the country code and HW"
    # "ip": "192.168.1.1",              #"select unique IP address"
    "subnet": "255.255.255.0",  # "subnet mask"
    "tx_power": "30",
    # "select 30dBm, HW and regulations limiting it correct level. Can be used to set lower dBm levels for testing purposes (e.g. 5dBm)"
    "mode": "mesh",  # "mesh=mesh network, ap=debug hotspot"
    "authServer": False
}

IP_PREFIX = '.'.join(args.address.split('.')[0:3])

IP_ADDRESSES = {'0.0.0.0': IP_PREFIX + '.0'}

SERVER_CERT = args.certificate

if os.path.isfile("data/auth.csv"):
    MAC_ADDRESSES = pd.read_csv('data/auth.csv', names=['Mesh IP', 'MAC Address'])
    MAC_ADDRESSES.drop_duplicates(inplace=True)
else:
    MAC_ADDRESSES = pd.DataFrame(columns=['Mesh IP', 'MAC Address'])
if os.path.isfile("data/no_auth.csv"):
    NOT_AUTH = pd.read_csv('data/no_auth.csv', names=['Mesh IP', 'MAC Address'])
else:
    NOT_AUTH = pd.DataFrame(columns=['Mesh IP', 'MAC Address'])
if os.path.isfile("data/auth_routes.csv"):
    AUTH_ROUTES = pd.read_csv('data/auth_routes.csv')
else:
    AUTH_ROUTES = pd.DataFrame(columns=['MAC', 'Mesh_IP', 'Mesh_Network'])


@app.route('/add_message/<uuid>', methods=['GET', 'POST'])
def add_message(uuid):
    key = request.files['key']
    receivedKey = key.read()
    localCert = open(SERVER_CERT, 'rb')
    ip_address = request.remote_addr
    print("> Requester IP: " + ip_address)
    mac = get_mac_address(ip=ip_address)
    if verify_certificate(localCert, receivedKey):
        print(colored('> Valid Client Certificate', 'green'))
        ip_mesh = verify_addr(ip_address)
        print('> Assigned Mesh IP: ', end='')
        print(ip_mesh)
        aux = aux_ubuntu if uuid == 'Ubuntu' else aux_openwrt
        aux['gateway'] = False
        # if ip_mesh == IP_PREFIX + '.2':  # First node, then gateway
        #     aux['gateway'] = True
        #     add_default_route(ip_mesh, ip_address)  # we will need to add the default route to communicate
        # else:
        #     aux['gateway'] = False
        if int(ip_mesh.split('.')[
                   -1]) % 2 == 0:  # TODO: find smart way to set this value. Currently: only one server == '.3'
            aux['authServer'] = True
        aux['addr'] = ip_mesh
        SECRET_MESSAGE = json.dumps(aux)
        print('> Unencrypted message: ', end='')
        print(SECRET_MESSAGE)
        # use .call() to block and avoid race condition with open()
        proc = subprocess.call(['src/ecies_encrypt',
                                SERVER_CERT, SECRET_MESSAGE],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        enc = open('payload.enc', 'rb')
        encrypt_all = enc.read()
        print('> Sending encrypted message: ', end='')
        print(encrypt_all)
        return encrypt_all + localCert.read()
    else:
        if os.path.isfile("data/no_auth.csv"):
            NOT_AUTH = pd.read_csv('data/no_auth.csv', names=['Mesh IP', 'MAC Address'])
        else:
            NOT_AUTH = {}
        if mac not in NOT_AUTH.values:
            aux = {'Mesh IP': ip_address, 'MAC Address': mac}
            NOT_AUTH = NOT_AUTH.append(aux, ignore_index=True)
            NOT_AUTH.to_csv('data/no_auth.csv', index=False, header=False, mode='a')
            # FIX: this can cause overflow in case of IP/MAC spoofing
        print(colored("Not Valid Client Certificate", 'red'))
        return 'Not Valid Certificate'


def verify_certificate(old, new):
    """
    Here we are validating the hash of the certificate. This is giving us the integrity of the file, not if the
    certificate is valid. To validate if the certificate is valid, we need to verify the features of the certificate
    such as NotBefore, notAfter, crl file and its signature, issuer validity, if chain is there then this for all but
    for this we need to use x509 certificates.

    """
    old_md5 = hashlib.md5(old.read()).hexdigest()
    new_md5 = hashlib.md5(new).hexdigest()
    return old_md5 == new_md5


def add_default_route(ip_network, ip_gateway):
    inter = netifaces.interfaces()
    for interf in inter:
        # TODO: what it if doesn't start with wlan???
        if interf.startswith('wlan'):
            interface = interf

    command = 'ip route add ' + ip_network + '.0/24 ' + 'via ' + ip_gateway + ' dev ' + interface  # assuming only 2 interfaces are presented
    print(command)
    subprocess.call(command, shell=True)


def verify_addr(wan_ip):
    last_ip = IP_ADDRESSES[list(IP_ADDRESSES.keys())[-1]]  # get last ip
    last_octect = int(last_ip.split('.')[-1])  # get last ip octet
    if wan_ip not in IP_ADDRESSES:
        ip_mesh = IP_PREFIX + '.' + str(last_octect + 1)
        IP_ADDRESSES[wan_ip] = ip_mesh
    else:
        ip_mesh = IP_ADDRESSES[wan_ip]
    print('> All Addresses: ', end='')
    print(IP_ADDRESSES)
    return ip_mesh


def printing_auth():
    return MAC_ADDRESSES


def printing_no_auth():
    return NOT_AUTH


@app.route('/mac/<uuid>', methods=['GET', 'POST'])
def add_mac_addr(uuid):
    if os.path.isfile("data/auth.csv"):
        MAC_ADDRESSES = pd.read_csv('data/auth.csv', names=['Mesh IP', 'MAC Address'])
        MAC_ADDRESSES.drop_duplicates(inplace=True)
    else:
        if not os.path.exists("data"):
            os.mkdir("data")
        MAC_ADDRESSES = pd.DataFrame(columns=['Mesh IP', 'MAC Address'])
    mac = uuid
    ip_address = request.remote_addr
    if not MAC_ADDRESSES.empty and mac not in MAC_ADDRESSES.values:
        aux = {'Mesh IP': ip_address, 'MAC Address': mac}
        MAC_ADDRESSES = MAC_ADDRESSES.append(aux, ignore_index=True)
        MAC_ADDRESSES.to_csv('data/auth.csv', index=False, header=False, mode='a')
    print('> All Addresses: ', end='')
    return 'OK'


"""
This function adds elements to a dataframe, only the auth neighbor is store in a dataframe.
Also, it creates a default route to that network. 
"""


@app.route('/authServ/<address>')
def store_authServer(address):
    if os.path.isfile("data/auth_routes.csv"):
        AUTH_ROUTES = pd.read_csv('data/auth_routes.csv')
    else:
        if not os.path.exists("data"):
            os.mkdir("data")
        AUTH_ROUTES = pd.DataFrame(columns=['MAC', 'Mesh_IP', 'Mesh_Network'])
    ip_address = request.remote_addr
    mac = list(MAC_ADDRESSES.keys())[list(MAC_ADDRESSES.values()).index(ip_address)]
    if mac not in AUTH_ROUTES.values:
        aux = {'MAC': mac, 'Mesh_IP': ip_address, 'Mesh_Network': address}
        AUTH_ROUTES = AUTH_ROUTES.append(aux, ignore_index=True)
        AUTH_ROUTES.to_csv('data/auth_routes.csv', index=False, header=False, mode='a')
        add_default_route(address, ip_address)
    return AUTH_ROUTES


@app.route('/')
def debug():
    auth = printing_auth()
    bes = auth.to_html(classes='table table-striped', header=True, index=False)

    table = printing_no_auth()
    bes2 = table.to_html(classes='table table-striped', header=True, index=False)

    return '<h3>Authenticated Nodes</h3>' + bes + '\n' + "<h3>Not Valid Certificate Nodes</h3>" + bes2


if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)
