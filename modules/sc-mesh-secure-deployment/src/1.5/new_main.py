import multiprocessing
import queue
import random
import time
import socket
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
from threading import Thread

import pandas as pd

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
ne = ness_main.NESS()
ma = mba.MBA()
qua = quarantine.Quarantine()


def get_neighbors():
    return mesh_utils.get_macs_neighbors()


def launchCA(ID, sectable):  # need to see how to get the IP address of the neighbors
    '''
    this function should start server within localhost and client within the neighbors
    It should be the trigger in time-bases ex: every X seconds
    '''
    ca = ca_main.CA(ID)
    myip = co.get_ip_address(mesh_int)
    max_count = 8
    flag_ctr = 0
    proc = multiprocessing.Process(target=ca.as_server, args=(myip,))  # lock variable add later
    proc.start()
    time.sleep(random.randint(1, 10))
    neigh = get_neighbors()
    for ne in neigh:
        ind = sectable.index[sectable['MAC'] == ne].tolist()[0]
        IP = sectable.loc[ind]["IP"]
        print(IP)
        manager = multiprocessing.Manager()
        return_dict = manager.dict()
        p = multiprocessing.Process(target=ca.as_client, args=(IP, return_dict,))
        jobs = [p]
        p.start()
        for proc in jobs:
            proc.join()
        client_q[IP] = return_dict.values()
        flag_ctr += 1
        if flag_ctr == max_count:
            p.terminate()
            break
    proc.terminate()

    return client_q


def update_table_ca(df):
    df = df.assign(CA_Result=0)
    for index, row in df.iterrows():
        IP = row['IP']
        df.loc[df['IP'] == IP, 'CA_Result'] = 1
        df.loc[df['IP'] == IP, 'CA_ts'] = time.time()
    return df


def exchage_table(df):
    q = queue.Queue()

    client_q = {}
    message = df.to_json()
    neigh = get_neighbors()
    for ne in neigh:
        ind = sectable.index[sectable['MAC'] == ne].tolist()[0]
        IP = sectable.loc[ind]["IP"]
        print(IP)
        proc = Thread(target=server, args=(IP, message,))  # lock variable add later
        proc.daemon = True
        proc.start()
        p = Thread(target=client, args=(q,))
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
        time.sleep(1)


def listeningMBA():
    q = queue.Queue()
    mal = mba.MBA(mesh_utils.get_mesh_ip_address())
    Thread(target=mal.client, args=(q,)).start()
    return q.get()


def announcing(message):
    mal = mba.MBA(mesh_utils.get_mesh_ip_address())
    Thread(target=mal.server, args=(message, True,), daemon=True).start()


if __name__ == "__main__":
    mut = mutual.Mutual(mutual_int)
    mut.start()
    time.sleep(5)
    q = queue.Queue()  # queue for the malicious announcement
    if mesh_utils.verify_mesh_status():  # verifying that mesh is running
        sectable = pd.read_csv('../auth/dev.csv')
        sectable.drop_duplicates(inplace=True)
        result = launchCA(random.randint(1000, 64000),
                          sectable)  # currently is with a generic ID, need to be taken from the provisioning
        sectable = update_table_ca(sectable)
        received = exchage_table(sectable)
        for ind in received:
            new = pd.read_json(received[ind].decode())
            sectable = sectable.append(new, ignore_index=True)
        latest_status_list, good_server_status_list, flags_list, servers_list, nt = ne.adapt_table(sectable)
        ness_status = ne.run(latest_status_list, good_server_status_list, flags_list, servers_list,
                             nt)  # needs to get MAC
        Thread(target=ma.client, args=(q,)).start()  # malicious announcement client thread
        if ness_status == 194 or q.get() == 'malicious':
            sever_thread = Thread(target=ma.server, args=("malicious, IP ", True,),
                                  daemon=True)  # malicious announcement server #need to send IP and MAC
            qua.block()
