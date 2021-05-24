#!/usr/bin/python

from flask import Flask, request, json
from getmac import get_mac_address
import subprocess
import netifaces
import pandas as pd
import argparse
from termcolor import colored
import pathlib
import hashlib

# Construct the argument parser
ap = argparse.ArgumentParser()

# Add the arguments to the parser
ap.add_argument("-c", "--certificate", required=True)
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
    "mode": "mesh"  # "mesh=mesh network, ap=debug hotspot"
}

ADDRESSES = {'00:00:00:00:00:00': '10.20.15.1'}

IP_PREFIX = '10.20.15'

SERVER_CERT = args.certificate

NOT_AUTH = {}


@app.route('/api/add_message/<uuid>', methods=['GET', 'POST'])
def add_message(uuid):
    key = request.files['key']
    receivedKey = key.read()
    localCert = open(SERVER_CERT, 'rb')
    ip_address = request.remote_addr
    print("> Requester IP: " + ip_address)
    mac = get_mac_address(ip=ip_address)
    if verify_certificate(localCert, receivedKey):
        print(colored('> Valid Certificate', 'green'))
        ip_mesh = verify_addr(mac)
        print('> Assigned IP: ', end='')
        print(ip_mesh)
        aux = aux_ubuntu if uuid == 'Ubuntu' else aux_openwrt
        if ip_mesh == IP_PREFIX + '.2':  # First node, then gateway
            aux['gateway'] = True
            add_default_route(ip_address)  # we will need to add the default route to communicate
        else:
            aux['gateway'] = False
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
        return encrypt_all
    else:
        NOT_AUTH[mac] = ip_address
        print(colored("Not Valid Certificate", 'red'))
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


def add_default_route(ip_gateway):
    inter = netifaces.interfaces()
    for interf in inter:
        # TODO: what it if doesn't start with wlan???
        #if interf.startswith('wlan'):
        if interf.startswith('wl'):
            interface = interf

    command = 'ip route add ' + IP_PREFIX + '.0/24 ' + 'via ' + ip_gateway + ' dev ' + interface  # assuming only 2 interfaces are presented
    print(command)
    subprocess.call(command, shell=True)


def verify_addr(mac):
    last_ip = ADDRESSES[list(ADDRESSES.keys())[-1]]  # get last ip
    last_octect = int(last_ip.split('.')[-1])  # get last ip octet
    if mac not in ADDRESSES:
        ip_mesh = IP_PREFIX + '.' + str(last_octect + 1)
        ADDRESSES[mac] = ip_mesh
    else:
        ip_mesh = ADDRESSES[mac]
    print('> ALL Addresses: ', end='')
    print(ADDRESSES)
    return ip_mesh


def printing_auth():
    return ADDRESSES


def printing_no_auth():
    return NOT_AUTH


@app.route('/')
def debug():
    addresses_auth = printing_auth()
    table = pd.DataFrame.from_dict(addresses_auth, orient='index', columns=['Mesh IP Address'])
    table['MAC Address'] = table.index
    table.reset_index(drop=True, inplace=True)
    bes = table.to_html(classes='table table-striped', header=True, index=False)

    no_auth = printing_no_auth()
    table = pd.DataFrame.from_dict(no_auth, orient='index', columns=['IP Address'])
    table['MAC Address'] = table.index
    table.reset_index(drop=True, inplace=True)
    bes2 = table.to_html(classes='table table-striped', header=True, index=False)

    return '<h3>Authenticated Nodes</h3>' + bes + '\n' + "<h3>Not Valid Certificate Nodes</h3>" + bes2


if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)
