#!/bin/python3

import os
from os import path
from .utils import wifi_ssrc as wf
from .utils import funsocket as fs
import time
import pandas as pd
from .utils import primitives as pri
from termcolor import colored
import sys

sys.path.insert(0, '../../')
'''
only for testing 
'''
from common import ConnectionMgr

co = ConnectionMgr.ConnectionMgr()


def folder():
    if not path.isdir(PATH):
        os.mkdir(PATH)


PATH = 'auth/'
folder()

root_cert = "/etc/ssl/certs/root_cert.der"
local_cert = "/etc/ssl/certs/mesh_cert.der"

try:
    open(local_cert, 'rb')
except FileNotFoundError:
    print("Local certificate not found. Run generate_keys.sh")
    exit()
try:
    open(root_cert, 'rb')
except FileNotFoundError:
    print("Root certificate not found. Need to get it from provisioning")
    exit()


class Mutual:
    """
    set_level function to establish trust level of an imported key
    run is the state machine of the entire class
    interface should be the wireless interface of the mutual AP.
    """

    def __init__(self, interface):
        """
        Import the keys: node Pub,Priv and server(root) pub.
        """
        self.interface = interface
        print('Loading keys')
        self.root_cert = root_cert
        self.local_cert = local_cert
        self.debug = True
        self.cli = False
        self.myID = pri.get_labels()
        print("loading root_cert")
        pri.import_cert(root_cert, 'root')
        print("my ID: ", self.myID)
        self.table = self.create_table()


    def create_table(self):
        '''
        Function to create table of authenticated devices
        '''
        columns = ['ID', 'MAC', 'IP', 'PubKey_fpr', 'MA_level']
        if not path.isfile('auth/dev.csv'):
            table = pd.DataFrame(columns=columns)
            table.to_csv('auth/dev.csv', header=columns, index=False)
        else:
            table = pd.read_csv('auth/dev.csv')
        return table

    def update_table(self, info):
        '''
        this function update the table with the node's info
        '''
        if info['ID'] not in set(self.table['ID']):
            self.table = self.table.append(info, ignore_index=True)
            self.table.to_csv('auth/dev.csv', mode='a', header=False, index=False)
        elif self.table.loc[self.table['ID'] == info['ID']]['PubKey_fpr'].all() != info['PubKey_fpr']:
            self.table = self.table.append(info, ignore_index=True)
            self.table.drop_duplicates(inplace=True)
            self.table.to_csv('auth/dev.csv', mode='a', header=False, index=False)

    def get_status(self):
        '''
        status of the current authenticated nodes
        '''
        return self.table.to_dict()

    def signature(self):
        _, sig = pri.hashSig(self.root_cert)
        if self.debug:
            print(f'> Signature: {bytes(sig)}')
        return bytes(sig)

    def digest(self):
        dig, _ = pri.hashSig(self.root_cert)
        if self.debug:
            print(f'> Digest: {dig}')
        return dig

    def exchange(self, candidate, serverIP):
        cert = open(self.local_cert, 'rb').read()
        message = self.signature() + cert + self.myID.encode('utf-8')
        fs.client_auth(candidate, serverIP, message, self.interface)

    def client(self, candidate):
        print(f'AuthAP available: {candidate}')
        wf.connect_wifi(candidate, self.interface)
        print('2) sending signature of root certificate')
        serverIP = (".".join(co.get_ip_address(self.interface).split(".")[:3]) + ".1")
        self.exchange(candidate, serverIP)
        print('3) getting node certificate')
        server_sig, addr = fs.server_auth(self.myID, self.interface)
        self.cli = True
        return server_sig, addr

    def server(self):
        print('No AP available')
        print("I'm an authentication server. Creating AP")
        wf.create_ap(self.myID, self.interface)  # create AuthAPnodeID for authentication
        time.sleep(2)
        print('1) server default')
        client_cert, addr = fs.server_auth(self.myID, self.interface)
        return client_cert, addr

    def decode_cert(self, client):
        sig = self.signature()
        client_sig = client[:len(sig)]
        client_cert = client[len(sig):-5]
        cliID = client[-5:]  ##### ID must be 5 bits
        try:
            return client_sig, client_cert, cliID.decode('utf-8')
        except AttributeError:
            return client_sig, client_cert, cliID

    def set_password(self, node_name, cliID):
        password = co.get_password()
        if password == '':  # this means still not connected with anyone
            password = co.create_password()  # create a password (only if no mesh exist)
            co.util.update_mesh_password(password)  # update password in config file
        print(password)
        encrypt_pass = pri.encrypt_response(password, cliID)
        if self.debug:
            print(f'encrypted pass: {str(encrypt_pass)}')
        return encrypt_pass

    def send_password(self, cliID, addr, my_mac_mesh, my_ip_mesh, encrypt_pass):
        try:
            time.sleep(2)
            fs.client_auth(cliID, addr[0], encrypt_pass, self.interface)  # send mesh password
        except ConnectionRefusedError:
            time.sleep(7)
            fs.client_auth(cliID, addr[0], encrypt_pass, self.interface)
        client_mesh_ip, _ = fs.server_auth(self.myID, self.interface)
        if self.debug:
            print(f'Client Mesh IP: {str(client_mesh_ip)}')
        try:
            time.sleep(2)
            fs.client_auth(cliID, addr[0], my_mac_mesh.encode(), self.interface)  # send my mac
        except ConnectionRefusedError:
            time.sleep(7)
            fs.client_auth(cliID, addr[0], my_mac_mesh.encode(), self.interface)
        client_mac, _ = fs.server_auth(self.myID, self.interface)
        if self.debug:
            print(f'Client Mesh MAC: {str(client_mesh_ip)}')
        try:
            fs.client_auth(cliID, addr[0], my_ip_mesh.encode(), self.interface)  # send my mesh ip
        except ConnectionRefusedError:
            time.sleep(7)
            fs.client_auth(cliID, addr[0], my_ip_mesh.encode(), self.interface)
        return client_mac, client_mesh_ip

    def start_mesh(self):
        print('creating mesh')
        try:
            self.my_ip_mesh, self.my_mac_mesh = co.create_mesh_config
            co.starting_mesh()
        except Exception:
            TypeError
            print("No password provided for the mesh. Please get the password via provisioning server")
            exit()

    def define_role(self):
        candidate = wf.scan_wifi(self.interface)  # scan wifi to authenticate with
        if candidate:  # client
            sigs, addr = self.client(candidate)
            sig, client_cert, cliID = self.decode_cert(sigs)
            cli = True
        else:  # server
            client, addr = self.server()
            client_cert, sig, cliID, cli = self.send_my_key(client, addr)
        return addr, client_cert, sig, cliID, cli

    def send_my_key(self, client, addr):
        sig, client_cert, cliID = self.decode_cert(client)
        print("Client ID", cliID)
        print('4) local key')
        self.exchange(cliID, addr[0])
        cli = False
        return client_cert, sig, cliID, cli

    def cert_validation(self, sig, node_name, cliID, cli, addr):
        if pri.verify_certificate(sig, node_name, self.digest(), self.root_cert):
            print(colored('> Valid Certificate', 'green'))
            print('Authenticated, now send my pubkey')
            client_fpr, _ = pri.hashSig(node_name + '.der')
            pri.derive_ecdh_secret(node_name, cliID)
            if cli:  # client
                print('5) get password')
                enc_pass, _ = fs.server_auth(self.myID, self.interface)
                password = pri.decrypt_response(enc_pass, cliID)
                print(bytes(password).decode())
                co.util.update_mesh_password(bytes(password).decode())
                self.start_mesh()
                try:
                    fs.client_auth(cliID, addr[0], self.my_ip_mesh.encode(), self.interface)  # send my mesh ip
                except ConnectionRefusedError:
                    time.sleep(2)
                    fs.client_auth(cliID, addr[0], self.my_ip_mesh.encode(), self.interface)
                client_mac, _ = fs.server_auth(self.myID, self.interface)
                print(f'Client MAC: {str(client_mac)}')
                try:
                    time.sleep(2)
                    fs.client_auth(cliID, addr[0], self.my_mac_mesh.encode(), self.interface)  # send my mac
                except ConnectionRefusedError:
                    time.sleep(2)
                    fs.client_auth(cliID, addr[0], self.my_mac_mesh.encode(), self.interface)
                client_mesh_ip, _ = fs.server_auth(self.myID, self.interface)
            elif pri.verify_certificate(sig, node_name, self.digest(), self.root_cert):
                # print(colored('> Valid Certificate', 'green'))
                print("4.1) Setting Password")
                encrypt_pass = self.set_password(node_name, cliID)
                self.start_mesh()
                client_mac, client_mesh_ip = self.send_password(cliID, addr, self.my_mac_mesh, self.my_ip_mesh,
                                                                encrypt_pass)
            info = {'ID': self.myID, 'MAC': self.my_mac_mesh, 'IP': self.my_ip_mesh,
                    'PubKey_fpr': pri.hashSig(local_cert)[0], 'MA_level': 1}
            self.update_table(info)
            info = {'ID': cliID, 'MAC': client_mac.decode(), 'IP': client_mesh_ip.decode(),
                    'PubKey_fpr': client_fpr, 'MA_level': 1}
            self.update_table(info)  # #update csv file
        else:
            info = {'ID': cliID, 'MAC': "00:00:00:00", 'IP': addr[0],
                    'PubKey_fpr': "___", 'MA_level': 0}
            self.update_table(info)  # #update csv file
            print(colored("Not Valid Client Certificate", 'red'))
            pri.delete_key(node_name)
            os.remove(node_name + '.der')

    async def start(self):
        addr, client_cert, sig, cliID, cli = self.define_role()
        node_name = addr[0].replace('.', '_')
        pri.import_cert(client_cert, node_name)
        self.cert_validation(sig, node_name, cliID, cli, addr)

    def test(self):  # unit test
        raise NotImplementedError


if __name__ == "__main__":
    mutual = Mutual('wlan1')
    # mutual.terminate()
    mutual.start()
