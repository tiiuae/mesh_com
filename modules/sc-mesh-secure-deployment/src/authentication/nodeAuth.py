import gnupg
import os
from os import path
import glob
import wifi_ssrc as wf
from mesh import *
import socket
import time
import numpy as np
import pandas as pd


def folder():
    if not path.isdir(PATH):
        os.mkdir(PATH)


# always install it with python3 gnupg, do not use pip3 install gnupg (different libraries)
PATH = 'auth/'
folder()
try:
    gpg = gnupg.GPG(gnupghome='.')
except TypeError:
    gpg = gnupg.GPG(homedir='.')
os.environ['GNUPGHOME'] = 'auth/'


# def __init__():
def init():
    '''
    Import the keys: node Pub,Priv and server pub.
    Returns ID of the node: obtained from the certificate.
    '''
    files = glob.glob('hsm/*.asc')
    for fi in files:
        aux = open(fi, 'rb')  # assuming it's the first and only certificate
        fi2 = aux.read()
        gpg.import_keys(fi2)
    for ky in gpg.list_keys():
        if ky["uids"] == ["provServer <provServer>"]:
            fp = ky["fingerprint"]
    gpg.trust_keys(fp, 'TRUST_FULLY')  # trusting the server key
    for i in gpg.list_keys():
        if 'node' in i['uids'][0]:
            ID = i['uids'][0].split(' <')[0]
            fpr = fp
    return ID, fpr


def decrypt_conf(ID):
    '''
    Decrypt the mesh_conf file
    '''
    with open("hsm/" + ID + "_conf.conf.gpg", "rb") as f:
        status = gpg.decrypt_file(f, output="src/mesh_conf.conf")  # status has some info from the signer
        if status.ok:
            print("Mesh_conf file decrypt successful ")
        else:
            print("Mesh_conf file decrypt successful error")
            print("status: ", status.status)


def client(ID, ser_ip, message):
    HOST = ser_ip  # The server's hostname or IP address
    PORT = int(ID.split('node')[1])  # Port to listen on (non-privileged ports are > 1023)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        s.sendall(message)
        data = s.recv(1024)

    print('Received', repr(data))


def server_auth(ID, interface='wlan0'):
    '''
    Create a socket server and get the key information to import it.
    '''
    ip = wf.get_ip_address(interface)  # assuming that wlan0 will be (or connected to) the 'AP'
    HOST = ip
    PORT = int(ID.split('node')[1])  # Port to listen on (non-privileged ports are > 1023)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        conn, addr = s.accept()
        with conn:
            print("Connected by", addr)
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                conn.sendall(data)
                return data, addr


def authentication(ID):
    '''
    first import the key (I think this can be improved without importing the key, only scanning)
    Then we will check if the key has been signed for the Server(level 3), or another authenticated node (level 3)
    or for the server and for another authenticated node (level 4).
    If not, delete key.
    '''
    client_key, addr = server_auth(ID)
    ck = gpg.import_keys(client_key)
    fpr = ck.fingerprint
    level = 0
    for ky in gpg.list_keys(sigs=True):  # checking if key was signed by Server
        if ky["fingerprint"] == fpr[0] and len(ky['sigs']) > 1:  # this avoid self signed certificate
            for sig in ky["sigs"]:
                if ("provServer" in sig[1]  # signed by server
                        and float(ky['expires']) > time.time()):  # not expired
                    level = 3
                    gpg.trust_keys(fpr, 'TRUST_FULLY')  # trusting the server key
                for key_id in range(1, len(ky['sigs'])):
                    levels = []
                    for key in gpg.list_keys():
                        if ky['sigs'][key_id][0] == key["keyid"] and (
                                key["ownertrust"] == 'f'  # signed by a trusted node
                                and float(ky['expires']) > time.time()
                        ):  # not expired
                            levels.append(3)
                    count_level_signs = np.unique(levels, return_counts=True)[1]  # getting the number of 3s
                    if len(count_level_signs) == 0:  # no trusted signed
                        gpg.delete_keys(fpr, expect_passphrase=False)  # for public
                    else:
                        if count_level_signs[0] == 1:  # case only one signed and trusted
                            level = 3
                            gpg.trust_keys(fpr, 'TRUST_FULLY')  # trusting the imported key
                        if count_level_signs[0] > 1:  # more than one and all trusted
                            level = 4
                            gpg.trust_keys(fpr, 'TRUST_ULTIMATE')  # trusting the imported key
        else:
            gpg.delete_keys(fpr, expect_passphrase=False)  # for public
    return level, fpr, addr


def get_my_key(my_fpr):
    for key in gpg.list_keys():
        if key['fingerprint'] == my_fpr:
            key_data = gpg.export_keys(my_fpr, armor=False)
    return key_data


def get_id_from_frp(fpr):
    for key in gpg.list_keys():
        if key['fingerprint'] == fpr:
            nodeID = key['uids'][0].split(' <')[0]
    return nodeID


def create_table():
    '''
    Function to create table of authenticated devices
    '''
    columns = ['ID', 'MAC', 'IP', 'PubKey_fpr', 'trust_level']
    if not path.isfile('auth/dev.csv'):
        table = pd.DataFrame(columns=columns)
        table.to_csv('auth/dev.csv', header=columns, index=False)
    else:
        table = pd.read_csv('auth/dev.csv')
    return table


def update_table(info):
    '''
    this function update the table with the node's info
    '''
    table = create_table()
    if info['ID'] not in set(table['ID']):
        table = table.append(info, ignore_index=True)
        table.to_csv('auth/dev.csv', mode='a', header=False, index=False)
    elif table.loc[table['ID'] == info['ID']]['PubKey_fpr'] != info['PubKey_fpr']:
        table = table.append(info, ignore_index=True)
        table.to_csv('auth/dev.csv', mode='a', header=False, index=False)


if __name__ == "__main__":
    myID, my_fpr = init()
    candidate = wf.scan_wifi()  # scan wifi to authenticate with
    key_data = get_my_key(my_fpr)
    if candidate:
        wf.connect_wifi(candidate)
        level, fpr, addr = authentication(ID)
        if level > 3:
            nodeID = get_id_from_frp(fpr)
            info = {'ID': 'provServer', 'mac': 'mac', 'IP': '0.0.0.0', 'PubKey_fpr': fpr, 'trust_level': level}
            update_table(info)
            client(nodeID, addr, key_data)
            password = get_password()
            if password == '':  #this means still not connected with anyone
                decrypt_conf(myID)  # decrypt config provided by server
                password = create_password()  # create a password (only if no mesh exist)
                update_password(password)  # update password in config file
            client(nodeID, addr, gpg.encrypt(password, fpr, armor=False).data)
            create_mesh(myID)
    else:  # no wifi available need to start mesh by itself
        wf.create_ap(myID)  # create AuthAPnodeID for authentication

