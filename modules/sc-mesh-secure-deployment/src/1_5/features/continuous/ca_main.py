#!/usr/bin/python
import contextlib
import subprocess
from . import server
from . import client
import multiprocessing
from common import ConnectionMgr, mesh_utils
import pandas as pd
import numpy as np
from queue import Queue

import logging

co = ConnectionMgr.ConnectionMgr()

import sys
import os

sys.path.insert(0, '../../')
from features.mutual.utils import primitives as pri
from features.utils import utils as ut

class CA:

    def __init__(self, ID):
        self.ID = ID

    def as_client(self, server_ip):
        client.initiate_client(server_ip, self.ID)

    def as_server(self, ip, return_dict, id_dict, num_neighbors, ips_sectable):
        with contextlib.suppress(OSError):
            server.initiate_server(ip, return_dict, id_dict, num_neighbors, ips_sectable)

    def test(self):
        """
        unit test should be run as
        ca = ca_main.CA(random.randint(1000, 64000))
        ca.test()
        stores log to test_logs/cont_auth.txt
        """
        if not os.path.exists('test_logs/'):
            os.mkdir('test_logs/')
        orig_stdout = sys.stdout
        f = open('test_logs/cont_auth.txt', 'w')
        sys.stdout = f
        # Secret key derivation test
        print('====================================================================')
        print('Secret Key Derivation Test:')
        # Generate dummy EC key pair
        LIB = '/usr/lib/softhsm/libsofthsm2.so'
        pin = pri.recover_pin()
        command = ['pkcs11-tool', '--keypairgen', '--key-type', 'EC:prime256v1', '--login', '--pin', pin, '--module', LIB, '--label', 'test', '--id', '401']
        subprocess.call(command, shell=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        # Export to test.der
        command = ['pkcs11-tool','--read-object', '--id', '401', '--type', 'pubkey', '--module', LIB, '--output-file', 'test.der']
        subprocess.call(command, shell=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        secret_byte = pri.derive_ecdh_secret('test', '') # Secret derivation
        secret = int.from_bytes(secret_byte, byteorder=sys.byteorder)
        print('Derived secret = ', secret) # Key has been derived
        #logging.basicConfig(level=logging.DEBUG, filename='mylog.log')
        #logging.info('Derived secret = ', secret)

        # Delete test key pair
        command = ['pkcs11-tool', '--login', '--pin', pin, '--module', LIB, '--delete-object', '--type', 'privkey', '--label', 'test']
        subprocess.call(command, shell=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        command = ['pkcs11-tool', '--login', '--pin', pin, '--module', LIB, '--delete-object', '--type', 'pubkey', '--label', 'test']
        subprocess.call(command, shell=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # Authentication
        print('====================================================================')
        print('Authentication Test:')
        ip = '127.0.0.1'
        manager = multiprocessing.Manager()
        return_dict = manager.dict()
        id_dict = manager.dict()
        server_proc = multiprocessing.Process(target=self.as_server, args=(ip, return_dict, id_dict, 1, [ip]), daemon=True)
        server_proc.start()
        client_proc = multiprocessing.Process(target=self.as_client, args=(ip,), daemon=True)
        client_proc.start()
        client_proc.join()
        server_proc.join()

        print("CA Result dict = ", return_dict[ip]) # Authentication result 1 = pass, 0 = fail

        sys.stdout = orig_stdout
        f.close()

    def test_exchange_table(self):
        """
        unit test should be run as
        ca = ca_main.CA(random.randint(1000, 64000))
        ca.test_exchange_table()
        stores log to test_logs/exchange_table.txt
        global table is stored in test_logs/global_table.csv
        """
        if not os.path.exists('test_logs/'):
            os.mkdir('test_logs/')
        orig_stdout = sys.stdout
        f = open('test_logs/exchange_table.txt', 'w')
        sys.stdout = f

        sectable = pd.read_csv('test_inputs/dev.csv') # Read sample test security table
        ip = '10.10.10.4'  # My sample mesh IP
        neigh = ['10.10.10.5', '10.10.10.6']  # List of neighbor node IPs

        # Create initial exchange table where Source_IP = ip, Destination_IP ='', To_Send = ''
        exchange_table = ut.create_exchange_table(sectable, ip)
        print('Initial Exchange Table:')
        print(exchange_table)

        # Compute initial IPs to send
        exchange_table = ut.compute_ips_to_send(exchange_table, neigh)
        print('')
        print('Initial IPs to send:')
        print(exchange_table)

        exchange_table['Destination_IP'] = np.where(exchange_table['Destination_IP'] == '', exchange_table['To_Send'],
                                                    np.where(exchange_table['To_Send'] == '',
                                                             exchange_table[["Destination_IP", "To_Send"]].apply(
                                                                 "".join, axis=1),
                                                             exchange_table[["Destination_IP", "To_Send"]].apply(
                                                                 ",".join, axis=1)))

        print('')
        for neigh_ip in neigh:
            print('***************************************************************************')
            print('To send to ', neigh_ip)
            df_to_send = exchange_table[[(neigh_ip in x) for x in exchange_table['To_Send']]].copy()
            df_to_send['To_Send'] = ''
            print(df_to_send)

        exchange_table['To_Send'] = ''  # Clear To_Send column

        # Sample tables received from neighbors
        received_table_neigh1 = pd.read_csv('test_inputs/exchange_table_neigh1.csv')
        received_table_neigh2 = pd.read_csv('test_inputs/exchange_table_neigh2.csv')

        received_table_q = Queue()  # Create empty queue to store received tables from other nodes

        # Push received tables into queue
        received_table_q.put(received_table_neigh1)
        received_table_q.put(received_table_neigh2)

        while not received_table_q.empty():
            # get item from received_table_q
            df_received = received_table_q.get()
            # Concat received df to exchange_table
            exchange_table = pd.concat([exchange_table, df_received], ignore_index=True)

        print('')
        print('Exchange table after 1st exchange:')
        print(exchange_table)

        exchange_table = exchange_table.drop(columns=['Source_IP', 'Destination_IP', 'To_Send'])
        exchange_table.drop_duplicates(inplace=True)
        exchange_table.to_csv('test_logs/global_table.csv', mode='w', header=True, index=False)
        print('Global security table:')
        print(exchange_table)

        sys.stdout = orig_stdout
        f.close()