import multiprocessing
import queue
import time
from threading import Thread

import pandas as pd
from common import mesh_utils

from common import ConnectionMgr
from features.continuous import ca_main
from features.mba import mba
from features.mutual import mutual
import random
import asyncio



co = ConnectionMgr.ConnectionMgr()

mutual_int = 'wlan1'
mesh_int = 'bat0'
client_q = {}


def launchCA(ID, sectable):  # need to see how to get the IP address of the neighbors
    '''
    this function should start server within localhost and client within the neighbors
    It should be the trigger in time-bases ex: every X seconds
    '''
    ca = ca_main.CA(ID)
    myip = co.get_ip_address('bat0')
    # server_tread = Thread(target=ca.as_server, args=(myip, server_q), daemon=True)
    # server_tread.start()
    proc = multiprocessing.Process(target=ca.as_server, args=(myip))
    proc.start()
    for index, row in sectable.iterrows():
        IP = row['IP']
        print(IP)
        client_q[IP] = ca.as_client(IP)
    proc.terminate()
    # server_tread._stop()
    return (client_q)


def update_table(df,client_q):
    df = df.assign(CA_Result='no')
    for index, row in df.iterrows():
        IP = row['IP']
        df.loc[df['IP'] == IP, 'CA_Result'] = client_q[IP]
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
        neighbors = mesh_utils.get_mac_mesh(mesh_int)
        result = launchCA(random.randint(1000, 64000), sectable)
        sectable = update_table(sectable, result)




