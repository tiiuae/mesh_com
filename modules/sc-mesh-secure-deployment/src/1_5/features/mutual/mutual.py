#!/bin/python3

import os
import random
from os import path
from .utils import wifi_ssrc as wf
from .utils import funsocket as fs
import time
import pandas as pd
from .utils import primitives as pri
from termcolor import colored
import sys
import shutil
import json
import base64
import hmac
import hashlib
from datetime import datetime
import pickle

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
        #self.salt = os.urandom(16)


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
        '''
        Sign root pubkey with my privKey
        '''
        _, sig = pri.hashSig(self.root_cert)
        if self.debug:
            print(f'> Signature: {bytes(sig)}')
        return bytes(sig)

    def digest(self):
        '''
        Get digest of the signature
        '''
        dig, _ = pri.hashSig(self.root_cert)
        if self.debug:
            print(f'> Digest: {dig}')
        return dig

    def message_generator(self, secret):
        '''
        Function to create an structure for the message, it is created a dictionary.
        The output is a json of a dictionary with the HMAC of the structure
        '''
        msg_to_mac_dict = {
        "root_sig": base64.b64encode(self.signature()).decode(),
        "client_cert":base64.b64encode(open(self.local_cert,'rb').read()).decode(),
        "id": self.myID,
        "u": random.randint(1,9999999999),
        "time_flag": time.time()
        }
        # convert dict to json
        msg_to_mac = json.dumps(msg_to_mac_dict)
        print("Message to MAC = ", msg_to_mac)
        # mac = MAC with secret as key (server id,client id,message,share u, time_flag)
        mac = hmac.new(bytes(str(secret), 'utf-8'), msg_to_mac.encode('utf-8'), hashlib.sha256).digest()
        #print("MAC =", mac)
        # message to send = {server id,client id,message,share u, timestamp, sa, MAC(server id,client id,message,share u)secret}
        # msg_to_send = msg_to_mac + ',' + str(sa) + ',' + str(mac)
        msg_to_send_dict = msg_to_mac_dict.copy()
        msg_to_send_dict["mac"] = base64.b64encode(mac).decode()
        return msg_to_send_dict

    def exchange(self, candidate, serverIP):
        '''
        function to exchange info as a client
        '''
        message = self.message_generator(open(root_cert, 'rb').read())
        fs.client_auth(candidate, serverIP, json.dumps(message).encode(), self.interface)

    def client(self, candidate):
        '''
        Function to conect to wifi AP and then exchange data
        '''
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
        '''
        Server (AP)
        '''
        print('No AP available')
        print("I'm an authentication server. Creating AP")
        wf.create_ap(self.myID, self.interface)  # create AuthAPnodeID for authentication
        time.sleep(2)
        print('1) server default')
        client_cert, addr = fs.server_auth(self.myID, self.interface)
        return client_cert, addr

    def verify_fresh(self, timestamp):
        '''
        Function to verify freshness of messages
        currently should be less than 20 seconds
        '''
        new_timestamp=datetime.fromtimestamp(timestamp)
        now = datetime.now()
        if abs(now - new_timestamp) >= 20:
            return False
        print("Fresh Message")
        return True

    def verify_hmac(self, client_sig, client_cert, cliID, nonce, timestamp, client_hmac):        # Compute fresh MAC
        '''
        Function to verify the HMAC of the received message
        '''
        msg_to_mac_dict = {
            "root_sig": client_sig,
            "client_cert": client_cert,
            "id": cliID,
            "u": nonce,
            "time_flag": timestamp
        }
        # convert dict to json
        msg_to_mac = json.dumps(msg_to_mac_dict)
        if (
            hmac.new(
                open(root_cert, 'rb').read(),
                msg_to_mac.encode('utf-8'),
                hashlib.sha256,
            ).digest()
            != client_hmac
        ):
            return False
        print("Valid HMAC")
        return True

    def verify_nonce(self, nonce):
        '''
        Function to verify the nonce of the message.
        Currently the list store all the information that has been exchanged.
        It should be good to clean it from time to time
        '''
        if os.path.isfile('auth/nonces.bin'):
            # Load the list back into memory
            with open('auth/nonces.bin', 'rb') as f:
                received_shares = pickle.loads(f.read())
        else:
            received_shares = []
        if received_shares.count(nonce) == 0:
            print(colored('> Share has been used previously', 'red'))
            received_shares.append(nonce.to_bytes(2, byteorder='big'))
            # Open a file and map it into memory
            with open('auth/nonces.bin', 'wb') as f:
                pickle.dump(received_shares, f)
        else:
            print(colored('> Share has been used previously', 'red'))
            # return "fail"
            return 0
        # Access the list through the memory-mapped file


    def decode_cert(self, message):
        ''''
        Decode the current message, reconstruct the dictionary and start the process verification
        '''
        me = json.loads(message)
        client_sig = me['root_sig']
        client_cert = me['client_cert']
        cliID = me['id']
        nonce = me['u']
        timestamp = me['time_flag']
        client_hmac = me['mac']
        if self.verify_fresh(time):
            if self.verify_hmac(client_sig, client_cert, cliID, nonce, timestamp, client_hmac):
                return client_sig, client_cert, cliID.decode('utf-8')
            print(colored('> Not valid HMAC', 'red'))
        else:
            print(colored('> Stale message', 'red'))

        return 0

    def set_password(self, node_name, cliID):
        '''
        Function to create a random password in case the provisioning is not given a password for the mesh
        '''
        password = co.get_password()
        if password == '':  # this means still not connected with anyone
            password = co.create_password()  # create a password (only if no mesh exist)
            co.util.update_mesh_password(password)  # update password in config file
        print(password)
        secret_byte = pri.derive_ecdh_secret(node_name, '')
        encrypt_pass = pri.encrypt_response(password, cliID, secret_byte)
        if self.debug:
            print(f'encrypted pass: {str(encrypt_pass)}')
        return encrypt_pass

    def send_password(self, cliID, addr, my_mac_mesh, my_ip_mesh, encrypt_pass):
        '''
        This function send the created password, mac, mesh IP and other important info to the client.
        '''
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
        '''
        Start mesh based on the configuration from provisioning
        '''
        print('creating mesh')
        try:
            self.my_ip_mesh, self.my_mac_mesh = co.create_mesh_config
            co.starting_mesh()
        except Exception:
            TypeError
            print("No password provided for the mesh. Please get the password via provisioning server")
            exit()

    def define_role(self):
        '''
        Define role (client/server)
        '''
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
        '''
        Send my public key
        '''
        sig, client_cert, cliID = self.decode_cert(client)
        print("Client ID", cliID)
        print('4) local key')
        self.exchange(cliID, addr[0])
        cli = False
        return client_cert, sig, cliID, cli

    def cert_validation(self, sig, node_name, cliID, cli, addr):
        '''
        mostly main function, it will verify the signature received and then will exchange some info to validate the node.
        At the end if the auth is successfully, the mesh is established and the node is added to the sec table.
        '''
        if pri.verify_certificate(sig, node_name, self.digest(), self.root_cert):
            print(colored('> Valid Certificate', 'green'))
            print('Authenticated, now send my pubkey')
            client_fpr, _ = pri.hashSig(node_name + '.der')
            #pri.derive_ecdh_secret(node_name, cliID, self.local_cert, self.salt)
            if cli:  # client
                print('5) get password')
                enc_pass, _ = fs.server_auth(self.myID, self.interface)
                secret_byte = pri.derive_ecdh_secret(node_name, '')
                password = pri.decrypt_response(enc_pass, cliID, secret_byte)
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

            # Create directory pubKeys to store neighbor node's public key certificates to use for secret derivation
            if not os.path.exists('pubKeys/'):
                os.mkdir('pubKeys/')

            # Copy client certificate to pubKey/{client_mesh_ip}.der to use for secret derivation
            client_mesh_name = client_mesh_ip.decode().replace('.','_')
            shutil.copyfile(f'{node_name}.der', f'pubKeys/{client_mesh_name}.der')
        else:
            info = {'ID': cliID, 'MAC': "00:00:00:00", 'IP': addr[0],
                    'PubKey_fpr': "___", 'MA_level': 0}
            self.update_table(info)  # #update csv file
            print(colored("Not Valid Client Certificate", 'red'))
            pri.delete_key(node_name)
            os.remove(node_name + '.der')

    async def start(self):
        '''
        Start the entire process
        '''
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
