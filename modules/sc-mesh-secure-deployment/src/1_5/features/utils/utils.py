import queue
import socket
import sys
import threading
import glob

import subprocess

sys.path.append('../../')
from common import mesh_utils


def exchage_table(df):
    '''
    this function exchange the table (json) with the neighbors
    '''
    q = queue.Queue()
    client_q = {}
    message = df.to_json()
    threading.Thread(target=exchange_server, args=(q,), daemon=True).start()
    neigh = mesh_utils.get_arp()
    ip2_send = list(set(df['IP'].tolist() + list(neigh.keys())))
    for IP in ip2_send:
        if IP != mesh_utils.get_mesh_ip_address():
            client_q = exchange_process(IP, message, client_q, q)
    # p.join()
    return client_q


def exchange_process(IP, message, client_q, q):
    p = threading.Thread(target=exchange_client, args=(IP, message,), daemon=True).start()  # lock variable add later
    # jobs = [p]
    try:
        client_q[IP] = q.get(timeout=2)
        print(client_q[IP])
    except queue.Empty:
        pass
    # for pr in jobs:
    #    pr.join()
    return client_q


def exchange_server(q, debug=False):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("0.0.0.0", 5005))
        sock.listen()
    except OSError:
        pass
    while 1:
        data, addr = sock.recvfrom(1024)
        print("Message received")
        if debug:
            print(data, addr)
        q.put(data)


def exchange_client(IP, message, debug=False):
    num_message = 3
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.setblocking(False)
        # while num_message:
        if debug:
            print(f"this is client and it will send message {num_message} more times")
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


