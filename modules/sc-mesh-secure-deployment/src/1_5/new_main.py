import multiprocessing
import queue
import random
from time import time, sleep
import socket
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
import threading
import glob
import subprocess
import asyncio

import contextlib
import pandas as pd
import numpy as np

from common import ConnectionMgr
from common import mesh_utils
from features.continuous import ca_main
from features.mba import mba
from features.mutual import mutual
from features.ness import ness_main
from features.quarantine import quarantine

co = ConnectionMgr.ConnectionMgr()
_executor = ThreadPoolExecutor()
executor = ProcessPoolExecutor()
mutual_int = 'wlan1'
mesh_int = 'bat0'
client_q = {}
ness = ness_main.NESS()
qua = quarantine.Quarantine()


async def launchCA(ID, sectable):
    """
    this function should start server within localhost and client within the neighbors
    It should be the trigger in time-bases ex: every X seconds
    """
    with contextlib.suppress(OSError):
        ca = ca_main.CA(ID)
    myip = co.get_ip_address(mesh_int)
    max_count = 3
    flag_ctr = 0
    proc = multiprocessing.Process(target=ca.as_server, args=(myip,))
    proc.start()
    sleep(random.randint(1, 10))
    neigh = mesh_utils.get_macs_neighbors()
    for ne in neigh:
        if ne not in sectable["MAC"]:  # means that this is neighbor but was authenticated by someone else
            ne_ips = mesh_utils.get_neighbors_ip()
            for ip in ne_ips:
                if ip not in sectable["IP"]:
                    info = {'ID': '---', 'MAC': ne, 'IP': ip,
                            'PubKey_fpr': "___", 'MA_level': -1}
                    mut = mutual.Mutual(mutual_int)
                    mut.update_table(info)
                IP = ip
                sectable = pd.read_csv('auth/dev.csv')
        else:
            ind = sectable.index[sectable['MAC'] == ne].tolist()[0]
            IP = sectable.loc[ind]["IP"]
        print("neighbor IP:", IP)
        manager = multiprocessing.Manager()
        return_dict = manager.dict()
        p = multiprocessing.Process(target=ca.as_client, args=(IP, return_dict))
        jobs = [p]
        p.start()
        for proc in jobs:
            proc.join()
        sleep(5)
        for key in return_dict.keys():
            count = np.unique(return_dict[key], return_counts=True)
            if len(count[1]) > 1:
                final = count[0][0] if count[1][0] > count[1][1] else count[0][1]
            else:
                final = count[0][0]
        client_q[IP] = final
        flag_ctr += 1
        if flag_ctr == max_count:
            p.terminate()
            break
    proc.terminate()
    return client_q


def update_table_ca(df, result):
    """
    function to update the mutual authentication table with the CA result.
    Note: this function should be called after the CA result is received.
    Note2: the new table is not being saved only converted to json and sent to the neighbors.
    """
    filetable = 'auth/dev.csv'
    old_table = pd.read_csv(filetable)
    df = df.assign(CA_Result=0)
    for index, row in df.iterrows():
        for res in result:
            for ip in res.keys():
                if row['IP'] == ip:
                    IP = row['IP']
                    if res[ip] in ['pass', 1]:
                        df.loc[index, 'CA_Result'] = 1
                    elif res[ip] == 'fail':
                        df.loc[df['IP'] == IP, 'CA_Result'] = 2
                    else:
                        df.loc[df['IP'] == IP, 'CA_Result'] = 3
                    df.loc[df['IP'] == IP, 'CA_ts'] = time()
    df.drop_duplicates(inplace=True)
    if 'CA_Result' in set(old_table):
        df.to_csv(filetable, mode='a', index=False)
    else:
        df.to_csv(filetable, mode='a', header=False, index=False)
    return df


def exchage_table(df):
    '''
    this function exchange the table (json) with the neighbors
    '''
    q = queue.Queue()
    client_q = {}
    message = df.to_json()
    neigh = mesh_utils.get_macs_neighbors()
    for ne in neigh:
        ind = df.index[df['MAC'] == ne].tolist()[0]
        IP = df.loc[ind]["IP"]
        print(IP)
        proc = threading.Thread(target=server, args=(IP, message,))  # lock variable add later
        proc.daemon = True
        proc.start()
        p = threading.Thread(target=client, args=(q,))
        jobs = [p]
        p.start()
        client_q[IP] = q.get()
        print(client_q[IP])
        for proc in jobs:
            proc.join()
    return client_q


def client(q, debug=False):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(("0.0.0.0", 5005))
    data, addr = sock.recvfrom(1024)
    print("mba: message received")
    if debug:
        print(data, addr)
    q.put(data)


def server(IP, message, debug=False):
    num_message = 5
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setblocking(False)
    while num_message:
        if debug:
            print(f"this is server and it will send message {num_message} more times")
        sock.sendto(bytes(message, "utf-8"), (IP, 5005))
        num_message -= 1
        sleep(1)


def listeningMBA():
    q = queue.Queue()
    mal = mba.MBA(mesh_utils.get_mesh_ip_address())
    threading.Thread(target=mal.client, args=(q,)).start()
    return q.get()


def announcing(message):
    mal = mba.MBA(mesh_utils.get_mesh_ip_address())
    threading.Thread(target=mal.server, args=(message, True,), daemon=True).start()


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


def continuous_authentication(sectable):
    print("Starting Continuous Authentication")
    loop_ca = asyncio.get_event_loop()
    ca_task = loop_ca.create_task(launchCA(random.randint(1000, 64000), sectable))
    with contextlib.suppress(asyncio.CancelledError):
        result = loop_ca.run_until_complete(asyncio.gather(ca_task))
    sectable = update_table_ca(sectable, result)
    print("End of Continuous Authentication")
    return sectable


def decision_engine(first_round, ness_result, sectable, ma, q):
    print("Starting Decision Engine")
    if first_round:
        latest_status_list, good_server_status_list, flags_list, servers_list, nt, mapp = ness.first_table(sectable)
        threading.Thread(target=ma.client, args=(q,), daemon=True).start()
        first_round = False
    else:
        latest_status_list, good_server_status_list, flags_list, servers_list, nt, mapp = ness.adapt_table(ness_result)
    ness_result = ness.run(latest_status_list, good_server_status_list, flags_list, servers_list, nt)
    print("Decision Engine Done")
    return ness_result, first_round, mapp


def quarantine(ness_result, q, sectable, ma, mapp):
    for node in ness_result:
        if ness_result[node] == 194 or (not q.empty() and q.get() == 'malicious'):
            print("Malicious Node: ", ness.remapping(mapp, node))
            mac = sectable.iloc[node]['IP']
            threading.Thread(target=ma.server, args=("malicious: " + mac, True), daemon=True).start()
            threading.Thread(target=qua.block, args=(mac,), daemon=True).start()


def sec_beat(first_round, ness_result):
    q = queue.Queue()
    checkiptables()
    ma = mba.MBA(mesh_utils.get_mesh_ip_address())
    sectable = pd.read_csv('auth/dev.csv')
    sectable.drop_duplicates(inplace=True)
    sectable = continuous_authentication(sectable)
    received = exchage_table(sectable)
    for ind in received:
        new = pd.read_json(received[ind].decode())
        sectable = pd.concat([sectable, new], ignore_index=True)
    ness_result, first_round, mapp = decision_engine(first_round, ness_result, sectable, ma, q)
    quarantine(ness_result, q, sectable, ma, mapp)
    return first_round, ness_result


def mutual_authentication():
    print("Starting Mutual Authentication")
    mut = mutual.Mutual(mutual_int)
    loop = asyncio.get_event_loop()
    mutual_task = loop.create_task(mut.start())
    with contextlib.suppress(asyncio.CancelledError):
        loop.run_until_complete(mutual_task)
    sleep(5)
    print("End of Mutual Authentication")


if __name__ == "__main__":
    sec_beat_time = 5
    first_round = True  # flag for the decision engine. If not information available, create a table.
    # Mutual Authentication
    mutual_authentication()
    ness_result = {}
    while True:
        if mesh_utils.verify_mesh_status():  # verifying that mesh is running
            sleep(sec_beat_time - time() % sec_beat_time)  # sec beat time
            first_round, ness_result = sec_beat(first_round, ness_result)
        else:
            print("No mesh established")
