#!/usr/bin/python

from flask import Flask, request, json
from getmac import get_mac_address
import subprocess
import netifaces
#Enable pandas after availablity of _bz2 shared lib
#import pandas as pd
import yaml
import json
import argparse
from termcolor import colored
import pathlib
import hashlib
import os

# Construct the argument parser
ap = argparse.ArgumentParser()

# Add the arguments to the parser
ap.add_argument("-c", "--certificate", required=True)
ap.add_argument("-t","--test",required=False,default=False,action='store_true')
ap.add_argument("-m", "--mode", required=True)
args = ap.parse_args()
# Get the mesh_com config
print(os.getenv("MESH_COM_ROOT", ""))
mesh_mode = args.mode
if mesh_mode == '11s':
    config_path=os.path.join(os.getenv("MESH_COM_ROOT", ""), "src/mesh_com_11s.conf")
elif mesh_mode == 'ibss':
    config_path=os.path.join(os.getenv("MESH_COM_ROOT", ""), "src/mesh_com.conf")

print('> Loading yaml conf... ')
try:
    yaml_conf = yaml.safe_load(open(config_path, 'r'))
    conf = yaml_conf['server']
    debug = yaml_conf['debug']
except (IOError, yaml.YAMLError) as error:
    print(error)
    exit()

app = Flask(__name__)
IP_ADDRESSES = {'0.0.0.0': '10.20.15.1'}
MAC_ADDRESSES = {'00:00:00:00:00:00': '10.20.15.1'}
IP_PREFIX = '10.20.15'
SERVER_CERT = args.certificate
NOT_AUTH = {}

def Server_Test(**arg):
    if args.test:
        f = open("/opt/mesh_com/modules/sc-mesh-secure-deployment/src/testclient.txt","w")
        if arg["color"] == "Green":
            f.write("True")
            f.close()
        elif arg["Color"] == "Red":
            f.write("False")
            f.close()


@app.route('/api/add_message/<uuid>', methods=['GET', 'POST'])
def add_message(uuid):
    key = request.files['key']
    receivedKey = key.read()
    localCert = open(SERVER_CERT, 'rb')
    # Do we need the ubuntu or openwrt setup?
    aux = conf[str(uuid).lower()]
    # Requester a new IP
    ip_address = request.remote_addr
    print("> Requester IP: " + ip_address)
    # Get MAC
    mac = get_mac_address(ip=ip_address)
    if verify_certificate(localCert, receivedKey):
        print(colored('> Valid Client Certificate', 'green'))
        Server_Test(color="Green")
        ip_mesh = verify_addr(ip_address)
        print('> Assigned mesh IP: ' + ip_mesh)
        if ip_mesh == IP_PREFIX + '.2':  # First node, then gateway
            aux['gateway'] = True
            add_default_route(ip_address)  # we will need to add the default route to communicate
        else:
            aux['gateway'] = False
        aux['addr'] = ip_mesh
        msg_json = json.dumps(aux)
        if debug:
            print('> Unencrypted message: ', end='')
            print(msg_json)
        # Encrypt message use .call() to block and avoid race condition with open()
        proc = subprocess.call(['ecies_encrypt',
                                SERVER_CERT, msg_json],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        enc = open('payload.enc', 'rb')
        encrypt_all = enc.read()
        print('> Sending encrypted message...')
        if debug:
            print(encrypt_all)
        return encrypt_all + localCert.read()
    else:
        NOT_AUTH[mac] = ip_address
        print(colored("Not Valid Client Certificate", 'red'))
        Server_Test(color="Red")
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
    try:
        for interf in netifaces.interfaces():
            # TODO: what it if doesn't start with wlan???
            if interf.startswith(conf['mesh_inf']):
                interface = interf
        command = 'ip route add ' + IP_PREFIX + '.0/24 ' + 'via ' + ip_gateway + ' dev ' + interface
        print('> Adding default route to mesh... \'' + command + '\'')
        subprocess.call(command, shell=True)
    except:
        print("ERROR: No available interface for default route!")
        exit()


def verify_addr(wan_ip):
    last_ip = IP_ADDRESSES[list(IP_ADDRESSES.keys())[-1]]  # get last ip
    last_octect = int(last_ip.split('.')[-1])  # get last ip octet
    if wan_ip not in IP_ADDRESSES:
        ip_mesh = IP_PREFIX + '.' + str(last_octect + 1)
        IP_ADDRESSES[wan_ip] = ip_mesh
    else:
        ip_mesh = IP_ADDRESSES[wan_ip]
    print('> All addresses: ', end='')
    print(IP_ADDRESSES)
    json_object = json.dumps(IP_ADDRESSES,indent = 4)

    with open("/opt/mesh_com/modules/sc-mesh-secure-deployment/src/file.json","w") as outfile:
        outfile.write(json_object)
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

    return '<h3>Authenticated Nodes</h3>' + bes + '\n' + "<h3>Not Valid Certificate Nodes</h3>" + bes2


if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)
