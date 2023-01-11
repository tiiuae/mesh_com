import queue
import socket
import sys
import threading
import glob
import json
import time

import subprocess

import pandas as pd

sys.path.append('../../')
from common import mesh_utils


def exchage_table(df):
    '''
    this function exchange the table (json) with the neighbors
    '''
    message = df.to_json()
    threading.Thread(target=exchange_server, args=(), daemon=True).start()
    neigh = mesh_utils.get_arp()
    ip2_send = list(set(df['IP'].tolist() + list(neigh.keys())))
    for IP in ip2_send:
        if IP != mesh_utils.get_mesh_ip_address():
            threading.Thread(target=exchange_client, args=(IP, message,),
                             daemon=True).start()  # lock variable add later
            time.sleep(1)


def exchange_server(debug=True):
    table = 'auth/dev.csv'
    sectable = pd.read_csv((table))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("0.0.0.0", 5005))
        sock.listen()
    except OSError:
        pass
    while 1:
        data, addr = sock.recvfrom(4096)
        print("Message received")
        if debug:
            print(data, addr)
            print("Received: " + data.decode() + " from " + addr[0])
        if debug:
            print('Test sectable before update: ', sectable)
        try:
            new = pd.read_json(data.decode())
            if "CA_Result" in new.columns:
                try:
                    sectable.dropna(subset=['CA_Result', 'CA_Server'], how='all', inplace=True)
                except KeyError:
                    pass
            sectable = pd.concat([sectable, new], ignore_index=True)
            sectable.drop_duplicates(inplace=True)
            if debug:
                print('Test sectable after update: ', sectable)
            sectable.to_csv(table, mode='w', header=True, index=False)
        except ValueError:
            pass


def exchange_client(IP, message, debug=True):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.setblocking(False)
        # while num_message:
        if debug:
            print("Sending: " + str(message) + " to " + IP)
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
