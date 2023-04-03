import contextlib
import multiprocessing
import random
import sys
from queue import Queue
from time import sleep

import numpy as np
import pandas as pd

sys.path.insert(0, '../../')
from features.utils import utils as ut
from common import ConnectionMgr, mesh_utils, utils
from . import server
from . import client
#from header import MESHINT, MUTUALINT



co = ConnectionMgr.ConnectionMgr()
MUTUALINT = 'wlan1'
MESHINT = mesh_utils.get_mesh_int()





class CA:

    def __init__(self, ID):
        self.ID = ID

    def as_client(self, server_ip, logger=None):
        client.initiate_client(server_ip, self.ID, logger)

    def as_server(self, ip, return_dict, id_dict, num_neighbors, ips_sectable, logger=None):
        with contextlib.suppress(OSError):
            server.initiate_server(ip, return_dict, id_dict, num_neighbors, ips_sectable, logger)

    def test(self):
        '''
        unit test should be run as
        ca = ca_main.CA(random.randint(1000, 64000))
        ca.test()
        stores log to logs/continuous_1_node-log.txt
        '''
        common_ut = utils.Utils()
        logger = common_ut.setup_logger('continuous_1_node')
        ip = '127.0.0.1'
        manager = multiprocessing.Manager()
        return_dict = manager.dict()
        id_dict = manager.dict()
        server_proc = multiprocessing.Process(target=self.as_server, args=(ip, return_dict, id_dict, 1, [ip], logger),
                                              daemon=True)
        server_proc.start()
        client_proc = multiprocessing.Process(target=self.as_client, args=(ip, logger), daemon=True)
        client_proc.start()
        client_proc.join()
        server_proc.join()

        print("CA Result dict = ", return_dict)  # Authentication result 1 = pass, 0 = fail

        logger.info("Continuous authentication complete")
        logger.debug("Result dict = %s", return_dict)

        common_ut.close_logger(logger)

    def test_multiple_nodes(self, num_nodes):
        """
        prerequisite: mesh must be established between the nodes
        unit test should be run as
        ca = ca_main.CA(random.randint(1000, 64000))
        ca.test_multiple_nodes(num_nodes)
        stores log to logs/continuous_{num_nodes}}_nodes-log.txt
        """
        common_ut = utils.Utils()
        logger = common_ut.setup_logger(f'continuous_{num_nodes}_nodes')

        sectable = pd.read_csv('auth/dev.csv')

        #myip = co.get_ip_address(MESHINT)
        myip = mesh_utils.get_mesh_ip_address(MESHINT)

        manager = multiprocessing.Manager()
        return_dict = manager.dict()
        id_dict = manager.dict()

        ip2_send = list(set(sectable['IP'].tolist() + mesh_utils.get_neighbors_ip()))
        print('Checkpoint, neighbor IPs = ', mesh_utils.get_neighbors_ip())
        num_neighbors = len(ip2_send) - 1  # Number of neighbor nodes that need to be authenticated
        server_proc = multiprocessing.Process(target=self.as_server, args=(myip, return_dict, id_dict, num_neighbors, list(set(sectable['IP'].tolist())), logger), daemon=True)
        server_proc.start()
        sleep(random.randint(1, 10))

        client_jobs = []
        for IP in ip2_send:
            if IP != myip:
                client_proc = multiprocessing.Process(target=self.as_client, args=(IP, logger), daemon=True)
                client_proc.start()
                client_jobs.append(client_proc)

        for client_proc in client_jobs:
            client_proc.join()


        server_proc.join()
        print("CA Result dict = ", return_dict) # Authentication result 1 = pass, 0 = fail
        logger.info("Continuous authentication complete")
        logger.debug("Result dict = %s", return_dict)

        common_ut.close_logger(logger)

    def test_exchange_table(self):
        """
        prerequisite: mesh must be established between the nodes and security table must be created by running one round of continuous authentication
        unit test should be run as
        ca = ca_main.CA(random.randint(1000, 64000))
        ca.test_exchange_table()
        stores log to logs/exchange_table_1_node-log.txt
        global table is stored in logs/global_table.csv
        """
        common_ut = utils.Utils()
        logger = common_ut.setup_logger('exchange_table_1_node')

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
        try:
            exchange_table.to_csv('logs/global_table.csv', mode='w', header=True, index=False)
            logger.info("Global table created")
        except Exception as exp:
            logger.error("Global table creation failed with exception %s", exp)
        print('Global security table:')
        print(exchange_table)
        logger.debug("Global table:\n%s", exchange_table)

        common_ut.close_logger(logger)

    def test_exchange_table_multiple_nodes(self, num_nodes):
        """
        prerequisite: mesh must be established between the nodes
        unit test should be run as
        ca = ca_main.CA(random.randint(1000, 64000))
        ca.test_exchange_table_multiple_nodes(num_nodes)
        stores log to logs/exchange_table_{num_nodes}_nodes-log.txt
        """
        common_ut = utils.Utils()
        logger = common_ut.setup_logger(f'exchange_table_{num_nodes}_nodes')
        try:
            sectable = pd.read_csv('auth/dev.csv')
            sectable.drop_duplicates(inplace=True)
            start_server_thread = ut.start_server()
            sleep(0.5) # So that messages are not sent and dropped before other nodes start server
            ut.exchage_table(sectable, start_server_thread, logger)
        except FileNotFoundError:
            print("SecTable not available. Need to be requested during provisioning")
        common_ut.close_logger(logger)
