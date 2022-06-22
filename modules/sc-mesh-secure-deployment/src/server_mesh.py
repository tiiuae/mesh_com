#!/usr/bin/python

from flask import Flask, request, json
from getmac import get_mac_address
import subprocess
import netifaces
import pandas as pd
import yaml
import json
import argparse
from termcolor import colored
import pathlib
import hashlib
import os
import primitives as pri
from os import getenv

# Construct the argument parser
ap = argparse.ArgumentParser()

# Add the arguments to the parser
ap.add_argument("-c", "--certificate", required=True)
ap.add_argument("-t", "--test", required=False, default=False, action='store_true')
ap.add_argument("-m", "--mode", required=True)
args = ap.parse_args()
# Get the mesh_com config
print(os.getenv("MESH_COM_ROOT", ""))

# Get the mesh_com config
mesh_mode = args.mode

# Get the mesh_com config
if mesh_mode == 'ibss':
    conf_path = "src/mesh_com.conf"
elif mesh_mode == '11s':
    conf_path = "src/mesh_com_11s.conf"
config_path = os.path.join(getenv("MESH_COM_ROOT", ""), conf_path)
print('> Loading yaml conf... ')
try:
    yaml_conf = yaml.safe_load(open(config_path, 'r'))
    conf = yaml_conf['server']
    debug = yaml_conf['debug']
except (IOError, yaml.YAMLError) as error:
    print(error)
    exit()

args = ap.parse_args()
app = Flask(__name__)
IP_ADDRESSES = {'0.0.0.0': '10.20.15.1'}
MAC_ADDRESSES = {'00:00:00:00:00:00': '10.20.15.1'}
IP_PREFIX = '10.20.15'
SERVER_CERT = args.certificate
NOT_AUTH = {}


def Server_Test(**arg):
    if args.test:
        f = open("/opt/mesh_com/modules/sc-mesh-secure-deployment/src/testclient.txt", "w")
        if arg["color"] == "Green":
            f.write("True")
            f.close()
        elif arg["Color"] == "Red":
            f.write("False")
            f.close()


@app.route('/signature', methods=['GET', 'POST'])
def signature():
    _, sig = pri.hashSig(SERVER_CERT)
    if debug:
        print(f'> Signature: {bytes(sig)}')
    return bytes(sig)


@app.route('/digest', methods=['GET', 'POST'])
def digest():
    dig, _ = pri.hashSig(SERVER_CERT)
    if debug:
        print(f'> Digest: {dig}')
    return dig


@app.route('/api/add_message/<uuid>', methods=['GET', 'POST'])
def add_message(uuid):
    client_key = request.files['key']
    sig = request.files['sig']
    dig_received = request.files['dig']
    # Do we need the ubuntu or openwrt setup?
    aux = conf[str(uuid).lower()]
    # Requester a new IP
    ip_address = request.remote_addr
    print(f"> Requester IP: {ip_address}")
    # saving node cert
    node_name = ip_address.replace('.', '_')
    pri.import_cert(client_key, node_name)
    # Get MAC
    mac = get_mac_address(ip=ip_address)
    if pri.verify_certificate(sig.read(), node_name, dig_received.read(), SERVER_CERT):
        print(colored('> Valid Client Certificate', 'green'))
        ip_mesh = verify_addr(ip_address)
        print(f'> Assigned mesh IP: {ip_mesh}')
        # This is commented because it will be managed by the dynamic GW script
        # if ip_mesh == IP_PREFIX + '.2':  # First node, then gateway
        #     aux['gateway'] = True
        #     add_default_route(ip_address)  # we will need to add the default route to communicate
        # else:
        #     aux['gateway'] = False
        aux['addr'] = ip_mesh
        msg_json = json.dumps(aux)
        if debug:
            print('> Unencrypted message: ', end='')
            print(msg_json)
        enc = pri.encrypt_response(msg_json, node_name)
        print('> Sending encrypted message...')
        if debug:
            print(enc)
        return enc
    else:
        NOT_AUTH[mac] = ip_address
        print(colored("Not Valid Client Certificate", 'red'))
        pri.delete_key(node_name)
        os.remove(node_name + '.der')
        # delete via os
        return 'Not Valid Certificate'


def add_default_route(ip_gateway):
    try:
        for interf in netifaces.interfaces():
            # TODO: what it if doesn't start with wlan???
            if interf.startswith(conf['mesh_inf']):
                interface = interf
        interf = f'{IP_PREFIX}.0/24 '
        command = ['ip', 'route', 'add', interf, 'via', ip_gateway, 'dev', interface]
        print('> Adding default route to mesh... \'' + command + '\'')
        subprocess.call(command, shell=False)
    except:
        print("ERROR: No available interface for default route!")
        exit()


def verify_addr(wan_ip):
    last_ip = IP_ADDRESSES[list(IP_ADDRESSES.keys())[-1]]  # get last ip
    last_octect = int(last_ip.split('.')[-1])  # get last ip octet
    if wan_ip not in IP_ADDRESSES:
        ip_mesh = f'{IP_PREFIX}.{str(last_octect + 1)}'
        IP_ADDRESSES[wan_ip] = ip_mesh
    else:
        ip_mesh = IP_ADDRESSES[wan_ip]
    print('> All addresses: ', end='')
    print(IP_ADDRESSES)
    return ip_mesh


def printing_auth():
    return MAC_ADDRESSES


def printing_no_auth():
    return NOT_AUTH


@app.route('/mac/<uuid>', methods=['GET', 'POST'])
def add_mac_addr(uuid):
    mac = uuid
    ip_address = request.remote_addr
    MAC_ADDRESSES[mac] = IP_ADDRESSES[ip_address]
    if '00:00:00:00:00:00' in MAC_ADDRESSES.keys():
        del MAC_ADDRESSES['00:00:00:00:00:00']
    print('> All Addresses: ', end='')
    print(MAC_ADDRESSES)
    return MAC_ADDRESSES


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

    return (
            f'<h3>Authenticated Nodes</h3>{bes}'
            + '\n'
            + "<h3>Not Valid Certificate Nodes</h3>"
            + bes2
    )


if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)
