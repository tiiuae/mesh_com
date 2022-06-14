#!/bin/python3
import gnupg
import os
from os import path
import glob
from .utils import wifi_ssrc as wf
from .utils import mesh as mesh
from .utils import funsocket as fs
import time
import numpy as np
import pandas as pd
# import argparse
import sys


# # Construct the argument parser
# ap = argparse.ArgumentParser()
# # Add the arguments to the parser
# ap.add_argument("-c", "--clean", help='clean all (delete all keys and files)', required=False,
#                 default=False, const=True, nargs='?')
# ap.set_defaults(clean=False)
# args: argparse.Namespace = ap.parse_args()


def folder():
    if not path.isdir(PATH):
        os.mkdir(PATH)


# always install it with python3 gnupg, do not use pip3 install gnupg (different libraries)
PATH = '../auth'
folder()
try:
    gpg = gnupg.GPG(gnupghome='.')
except TypeError:
    gpg = gnupg.GPG(homedir='.')
os.environ['GNUPGHOME'] = '.'


def get_id_from_frp(fpr):
    '''
    get id from key fingerprint
    '''
    for key in gpg.list_keys():
        if key['fingerprint'] == fpr:
            nodeID = key['uids'][0].split(' <')[0]
    return nodeID


def decrypt_conf(ID):
    '''
    Decrypt the mesh_conf file
    '''
    with open(f"../hsm/{ID}.asc_conf.conf.gpg", "rb") as f:
        status = gpg.decrypt_file(f, output="../mesh_com.conf")  # status has some info from the signer
        if status.ok:
            print("Mesh_conf file decrypt successful ")
        else:
            print("Mesh_conf file decrypt error")
            print("status: ", status.status)


class Mutual:
    '''
    set_level function to establish trust level of an imported key
    run is the state machine of the entire class
    interface should be the wireless interface of the mutual AP.
    '''
    def __init__(self, interface):
        '''
        Import the keys: node Pub,Priv and server pub.
        Returns ID of the node: obtained from the certificate.
        '''
        self.interface = interface
        print('Loading keys')
        self.create_table()
        if files := glob.glob('../hsm/*.asc'):
            for fi in files:
                aux = open(fi, 'rb')  # assuming it's the first and only certificate
                fi2 = aux.read()
                gpg.import_keys(fi2)
            for ky in gpg.list_keys():
                if ky["uids"] == ["provServer <provServer>"]:
                    fpS = ky["fingerprint"]
                    gpg.trust_keys(fpS, 'TRUST_ULTIMATE')  # trusting the server key
                if 'node' in ky['uids'][0]:
                    self.myID = ky['uids'][0].split(' <')[0]
                    self.myfpr = ky["fingerprint"]
                if ky['fingerprint'] == self.my_fpr:
                    self.mykey = gpg.export_keys(self.my_fpr, armor=False)
        else:
            print('no keys in hsm folder')

    def terminate(self):
        '''
        to clean the previous keys
        '''
        keys = gpg.list_keys(True)
        for key in keys:
            gpg.trust_keys(key['fingerprint'], "TRUST_UNDEFINED")
            gpg.delete_keys(key['fingerprint'], True, expect_passphrase=False)  # for private
            gpg.delete_keys(key['fingerprint'], expect_passphrase=False)  # for public
        keys = gpg.list_keys()
        for key in keys:
            gpg.trust_keys(key['fingerprint'], "TRUST_UNDEFINED")
            gpg.delete_keys(key['fingerprint'], True, expect_passphrase=False)  # for private
            gpg.delete_keys(key['fingerprint'], expect_passphrase=False)  # for public
        try:
            os.remove('../auth/dev.csv')
        except FileNotFoundError:
            print('Done')
        sys.exit()

    def set_level(self, client_key):
        '''
        first import the key (I think this can be improved without importing the key, only scanning)
        Then we will check if the key has been signed for the Server(level 3), or another authenticated node (level 3)
        or for the server and for another authenticated node (level 4).
        If not, delete key.
        '''
        ck = gpg.import_keys(client_key)
        print(ck.results)
        fpr = ck.fingerprints
        level = 0
        levels = []
        for ky in gpg.list_keys(sigs=True):  # checking if key was signed by Server
            if ky["fingerprint"] == fpr[0] and len(ky['sigs']) > 1:  # this avoids self signed certificate
                for sig in ky["sigs"]:
                    if ("provServer" in sig[
                        1]  # signed by server #note: if the server signed the keys in a different 'moment' this will fail
                            and float(ky['expires']) > time.time()):  # not expired
                        levels.append(3)
                        gpg.trust_keys(fpr, 'TRUST_ULTIMATE')  # trusting the server key
                    for key_id in range(1, len(ky['sigs'])):
                        for key in gpg.list_keys():
                            if ky['sigs'][key_id][0] == key["keyid"] and (
                                    key["ownertrust"] == 'f' or key[
                                "ownertrust"] == 'u'  # signed by a trusted node fully or ultimate
                                    and float(ky['expires']) > time.time()
                            ):  # not expired
                                levels.append(3)
                        count_level_signs = np.unique(levels, return_counts=True)[1]  # getting the number of 3s
                        if len(count_level_signs) == 0:  # no trusted signed
                            gpg.delete_keys(fpr, expect_passphrase=False)  # for public
                        else:
                            if count_level_signs[0] == 1:  # case only one signed and trusted
                                level = 3
                                gpg.trust_keys(fpr, 'TRUST_ULTIMATE')  # trusting the imported key
                            if count_level_signs[0] > 1:  # more than one and all trusted
                                level = 4
                                gpg.trust_keys(fpr, 'TRUST_ULTIMATE')  # trusting the imported key
        print('PubKey authenticated with level ' + str(level))
        return level, fpr

    def get_status(self):
        '''
        status of the current authenticated nodes
        '''
        return self.table.to_dict()

    def create_table(self):
        '''
        Function to create table of authenticated devices
        '''
        columns = ['ID', 'MAC', 'IP', 'PubKey_fpr', 'trust_level']
        if not path.isfile('../auth/dev.csv'):
            table = pd.DataFrame(columns=columns)
            table.to_csv('../auth/dev.csv', header=columns, index=False)
        else:
            table = pd.read_csv('../auth/dev.csv')
        self.table = table

    def update_table(self, info):
        '''
        this function update the table with the node's info
        '''
        if info['ID'] not in set(self.table['ID']):
            self.table = self.table.append(info, ignore_index=True)
            self.table.to_csv('../auth/dev.csv', mode='a', header=False, index=False)
        elif self.table.loc[self.table['ID'] == info['ID']]['PubKey_fpr'].all() != info['PubKey_fpr']:
            self.table = self.table.append(info, ignore_index=True)
            self.table.to_csv('../auth/dev.csv', mode='a', header=False, index=False)

    def start(self):
        set_mesh = False
        cli = False
        decrypt_conf(self.myID)  # decrypt config provided by server
        print("I'm node " + str(self.myID))
        auth_role = mesh.get_auth_role()
        if auth_role == 'server':
            print("I'm an authentication Server. Creating AP")
            wf.create_ap(self.myID, self.interface)  # create AuthAPnodeID for authentication
            time.sleep(2)
            print('1) server default')
        else:  # Client role
            time.sleep(7)
            candidate = wf.scan_wifi(self.interface)  # scan wifi to authenticate with
            print(f'AP available: {candidate}')
            wf.connect_wifi(candidate)
            serverIP = (".".join(wf.get_ip_address(self.interface).split(".")[:3]) + ".1")
            print('2) send my key')
            fs.client_auth(candidate, serverIP, self.mykey)
            print('3) get node key')
            cli = True
        client_key, addr = fs.server_auth(self.myID)
        level, client_fpr = self.set_level(client_key)
        if level >= 3:
            print('Authenticated, now send my pubkey')
            nodeID = self.get_id_from_frp(client_fpr[0])
            if not set_mesh:
                print('creating mesh')
                my_mesh_ip, my_mac_mesh = mesh.create_mesh(self.myID.split('node')[1])
                set_mesh = True
            if not cli:  # initial server
                print('4) server key')
                fs.client_auth(nodeID, addr[0], self.mykey)  # send my public key
                password = mesh.get_password()
                if password == '':  # this means still not connected with anyone
                    password = mesh.create_password()  # create a password (only if no mesh exist)
                    mesh.update_password(password)  # update password in config file
                print(password)
                encrpt_pass = gpg.encrypt(password, client_fpr[0], armor=False).data
                print(f'encrypted pass: {str(encrpt_pass)}')
                try:
                    time.sleep(2)
                    fs.client_auth(nodeID, addr[0], encrpt_pass)  # send mesh password
                except ConnectionRefusedError:
                    time.sleep(7)
                    fs.client_auth(nodeID, addr[0], encrpt_pass)
                client_mesh_ip, _ = fs.server_auth(self.myID)
                print(f'Client Mesh IP: {str(client_mesh_ip)}')
                try:
                    time.sleep(2)
                    fs.client_auth(nodeID, addr[0], my_mac_mesh.encode())  # send my mac
                except ConnectionRefusedError:
                    time.sleep(7)
                    fs.client_auth(nodeID, addr[0], my_mac_mesh.encode())
                client_mac, _ = fs.server_auth(self.myID)
                try:
                    fs.client_auth(nodeID, addr[0], my_mesh_ip.encode())  # send my mesh ip
                except ConnectionRefusedError:
                    time.sleep(7)
                    fs.client_auth(nodeID, addr[0], my_mesh_ip.encode())
            else:  # client
                print('5) get password')
                enc_pass, _ = fs.server_auth(self.myID)
                password = gpg.decrypt(enc_pass)
                print(password)
                mesh.update_password(str(password))  # update password in config file
                try:
                    fs.client_auth(nodeID, addr[0], my_mesh_ip.encode())  # send my mesh ip
                except ConnectionRefusedError:
                    time.sleep(7)
                    fs.client_auth(nodeID, addr[0], my_mesh_ip.encode())
                client_mac, _ = fs.server_auth(self.myID)
                print(f'Client MAC: {str(client_mac)}')
                try:
                    time.sleep(2)
                    fs.client_auth(nodeID, addr[0], my_mac_mesh.encode())  # send my mac
                except ConnectionRefusedError:
                    time.sleep(7)
                    fs.client_auth(nodeID, addr[0], my_mac_mesh.encode())
                client_mesh_ip, _ = fs.server_auth(self.myID)
            info = {'ID': 'provServer', 'MAC': client_mac.decode(), 'IP': client_mesh_ip.decode(),
                    'PubKey_fpr': client_fpr[0], 'trust_level': level}
            self.update_table(info)  # #update csv file


if __name__ == "__main__":
    mutual = Mutual()
    mutual.terminate()
    mutual.start()



