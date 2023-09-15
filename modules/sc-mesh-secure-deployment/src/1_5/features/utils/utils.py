import contextlib
import socket
import sys
import threading
import glob
from queue import Queue

import subprocess

import pandas as pd
import numpy as np

sys.path.append('../../')
from common import mesh_utils

received_table_q = Queue() # Create empty queue to store received tables from other nodes

def start_server(initial_socket_timeout = 90):
    thread = threading.Thread(target=exchange_server, args=(initial_socket_timeout,), daemon=True)
    thread.start()
    return thread

def exchage_table(sectable, start_server_thread, logger=None, debug=True):
    #threading.Thread(target=exchange_server, args=(), daemon=True).start()
    #time.sleep(0.5) # So that messages are not sent and dropped before other nodes start server

    ip = mesh_utils.get_mesh_ip_address() # My mesh IP
    neigh = mesh_utils.get_neighbors_ip() # List of neighbor node IPs

    # Create initial exchange table where Source_IP = ip, Destination_IP ='', To_Send = ''
    exchange_table = create_exchange_table(sectable, ip)
    if debug:
        print('Initial Exchange Table:')
        print(exchange_table)
    # Compute initial IPs to send
    exchange_table = compute_ips_to_send(exchange_table, neigh)
    if debug:
        print('Initial IPs to send:')
        print(exchange_table)

    count = 1  # Number of exchanges
    send_flag = 0 if (exchange_table['To_Send'] == '').all() else 1 # send_flag = 0 if 'To_Send' is empty for all exchange_table entries

    # Run while sending table rows is not completed for all nodes
    #while send_flag > 0:
    while start_server_thread.is_alive() or send_flag == 1:
        if send_flag == 1:
            print('===================================================================================')
            print('Exchange ', count)
            print('===================================================================================')
            print('Send to neighbors in to send and append their ids into Destination_IP')
            exchange_table = send_table(exchange_table, neigh)

            #time.sleep(2)
        if not received_table_q.empty():
            print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            print('Tables after exchange:')
            #with received_table_q.mutex:
            # lock queue while popping dfs from it
            while not received_table_q.empty():
                # get item from received_table_q
                df_received = received_table_q.get()
                # Concat received df to exchange_table
                exchange_table = pd.concat([exchange_table, df_received], ignore_index=True)
            print(exchange_table[['ID', 'CA_Server', 'Destination_IP', 'To_Send']])

            print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            print('Compute IPs to send for next exchange:')
            exchange_table = compute_ips_to_send(exchange_table, neigh)
            print(exchange_table[['ID', 'CA_Server', 'Destination_IP', 'To_Send']])
        send_flag = 0 if (exchange_table['To_Send'] == '').all() else 1
        #print('send_flag: ', send_flag)
        count += 1

    exchange_table = exchange_table.drop(columns=['Source_IP', 'Destination_IP', 'To_Send'])
    exchange_table.drop_duplicates(inplace=True)
    try:
        exchange_table.to_csv('auth/global_table.csv', mode='w', header=True, index=False)
        if logger:
            logger.info("Global table created")
    except Exception as e:
        print("Global table creation failed with exception ", e)
        if logger:
            logger.error("Global table creation failed with exception %s", e)
    print('Global security table:')
    print(exchange_table)
    if logger:
        logger.debug("Global table:\n%s", exchange_table)

def exchange_server(initial_socket_timeout, debug=False):
    #table = 'auth/dev.csv'
    #sectable = pd.read_csv(table)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        with contextlib.suppress(OSError):
            sock.bind(("0.0.0.0", 5005))
            sock.listen()
        sock.settimeout(initial_socket_timeout) # Initial timeout to wait for cont auth to be done or exit infinite loop if nothing is received
        while 1:
            try:
                data, addr = sock.recvfrom(4096)
                print("Message received from " + addr[0])
                if debug:
                    print(data, addr)
                    print("Received: " + data.decode() + " from " + addr[0])
                with contextlib.suppress(ValueError):
                    new = pd.read_json(data.decode())
                    received_table_q.put(new)
                    print('Received_table_q size: ', received_table_q.qsize())
                    print('Received_table_q items: ')
                    print(received_table_q.queue)
            except socket.timeout:
                print('Socket timeout')
                break # If timeout, break while loop
            sock.settimeout(15)  # Setting timeout to exit infinite loop if nothing is received for 15 seconds

def exchange_client(IP, message, debug=False):
    print('Checkpoint inside exchange_client')
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.setblocking(False)
        # while num_message:
        if debug:
            print(f"Sending: {str(message)} to " + IP)
        sock.sendto(bytes(message, "utf-8"), (IP, 5005))
        # num_message -= 1
    # sleep(1)


def checkiptables():
    '''
    this function check if any iptables was blocked before
    '''
    files = glob.glob('features/quarantine/iptables*')
    if files:
        data = 0
        for fi in files:
            da = fi.split('-')[1]
            if da > data:
                data = da
        path = f'features/quarantine/iptables-{str(data)}'
        command = ['iptables-restore', '<', path]
        subprocess.call(command, shell=False)

def create_exchange_table(sectable, ip):
    '''
    Initial exchange table where Source_IP = ip, Destination_IP ='', To_Send = ''
    '''
    exchange_table = sectable.copy()
    exchange_table['Source_IP'] = ip
    exchange_table['Destination_IP'] = ''
    exchange_table['To_Send'] = ''
    return exchange_table

def compute_ips_to_send(exchange_table, neigh):
    '''
    Compute list of ips to send for each row of given exchange table
    '''
    print('Neighbors: ', neigh)
    for neigh_ip in neigh:
        for index, row in exchange_table.iterrows():
            if neigh_ip != row['Source_IP'] and neigh_ip not in row['Destination_IP'].split(','):
                if row['To_Send'] == '':
                    exchange_table.loc[index, ['To_Send']] = neigh_ip
                else:
                    exchange_table.loc[index, ['To_Send']] += ',' + neigh_ip
    return exchange_table

def send_table(exchange_table, neigh):
    # Append ips in To_Send to Destination_IP
    exchange_table['Destination_IP'] = np.where(exchange_table['Destination_IP'] == '', exchange_table['To_Send'], np.where(exchange_table['To_Send'] == '', exchange_table[["Destination_IP", "To_Send"]].apply("".join, axis=1), exchange_table[["Destination_IP", "To_Send"]].apply(",".join, axis=1)))
    threads = []
    for neigh_ip in neigh:
        print('***************************************************************************')
        print('To send to ', neigh_ip)
        df_to_send = exchange_table[[(neigh_ip in x) for x in exchange_table['To_Send']]].copy()
        df_to_send['To_Send'] = ''
        print(df_to_send)
        if not df_to_send.empty:
            # If there is dataframe to be sent to neigh_ip
            print('Sending df to ', neigh_ip)
            message = df_to_send.to_json()
            thread = threading.Thread(target=exchange_client, args=(neigh_ip, message,), daemon=True)
            thread.start()
            threads.append(thread)
    for thread in threads:
        # To avoid overwriting of 'To_Sent' column in exchange_table while messages are being sent out
        thread.join()
    exchange_table['To_Send'] = '' # Clear To_Send column
    print('Checkpoint end of send table')
    return exchange_table