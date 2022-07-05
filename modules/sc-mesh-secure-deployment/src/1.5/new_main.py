import multiprocessing
import queue
import random
import time
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
from threading import Thread

import pandas as pd

from common import ConnectionMgr
from common import mesh_utils
from features.continuous import ca_main
from features.mba import mba
from features.mutual import mutual
from features.ness import ness_main

co = ConnectionMgr.ConnectionMgr()
_executor = ThreadPoolExecutor()
executor = ProcessPoolExecutor()
mutual_int = 'wlan1'
mesh_int = 'bat0'
client_q = {}
ne = ness_main.NESS()


def get_neighbors():
    return mesh_utils.get_macs_neighbors()


def launchCA(ID, sectable):  # need to see how to get the IP address of the neighbors
    '''
    this function should start server within localhost and client within the neighbors
    It should be the trigger in time-bases ex: every X seconds
    '''
    ca = ca_main.CA(ID)
    myip = co.get_ip_address('bat0')
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
        jobs = []
        p = multiprocessing.Process(target=ca.as_client, args=(IP, return_dict,))
        jobs.append(p)
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
    if mesh_utils.verify_mesh_status():
        sectable = pd.read_csv('../auth/dev.csv')
        result = launchCA(random.randint(1000, 64000), sectable)
        sectable = update_table_ca(sectable)
        latest_status_list, good_server_status_list, flags_list, servers_list, nt = ne.adapt_table(sectable)
        ne.run(latest_status_list, good_server_status_list, flags_list, servers_list, nt)

