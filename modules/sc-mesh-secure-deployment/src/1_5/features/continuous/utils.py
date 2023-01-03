import multiprocessing
import pandas as pd
from time import time, sleep
import random
import numpy as np
import sys

from .ca_main import *
from ..mutual import mutual

sys.path.append('../../..')
from common import ConnectionMgr, mesh_utils

co = ConnectionMgr.ConnectionMgr()
MUTUALINT = 'wlan1'
MESHINT = 'bat0'

client_q = {}


async def launchCA(sectable):
    """
    this function should start server within localhost and client within the neighbors
    It should be the trigger in time-bases ex: every X seconds
    """
    ID = random.randint(1000, 64000)
    with contextlib.suppress(OSError):
        ca = CA(ID)
    myip = co.get_ip_address(MESHINT)
    max_count = 3
    flag_ctr = 0

    manager = multiprocessing.Manager()
    return_dict = manager.dict()

    ip2_send = list(set(sectable['IP'].tolist() + mesh_utils.get_neighbors_ip()))  # Need to confirm if we can use this
    print('Checkpoint, neighbor IPs = ', mesh_utils.get_neighbors_ip())
    print('Checkpoint, ip2send = ', ip2_send)

    num_neighbors = len(ip2_send) - 1 # Number of neighbor nodes that need to be authenticated
    server_proc = ca_server(myip, return_dict, num_neighbors)
    # neigh = mesh_utils.get_arp()
    # ip2_send = list(set(sectable['IP'].tolist() + list(neigh.keys())))
    for IP in ip2_send:
        if IP != myip:
            flag_ctr = ca_client(IP, flag_ctr, max_count, ca)
    #server_proc.terminate()
    server_proc.join()
    print("Checkpoint server proc terminating")


    for key in return_dict.keys():
        count = np.unique(return_dict[key], return_counts=True)
        try:
            if len(count[1]) > 1:
                final = count[0][0] if count[1][0] > count[1][1] else count[0][1]
            elif len(count[1]) == 0:
                # Empty result
                final = 0
            else:
                final = count[0][0]
        except IndexError:
            final = 1
        try:
            client_q[key] = final
        except UnboundLocalError:
            print("Connection Refused")
            pass
    print('Final return_dict: ', return_dict)
    return client_q


def ca_server(myip, return_dict, num_neighbors):
    with contextlib.suppress(OSError):
        ca = CA(random.randint(1000, 64000))
    proc = multiprocessing.Process(target=ca.as_server, args=(myip, return_dict, num_neighbors), daemon=True)
    proc.start()
    sleep(random.randint(1, 10))
    return proc


def ca_client(IP, flag_ctr, max_count, ca):
    print("neighbor IP:", IP)
    p = multiprocessing.Process(target=ca.as_client, args=(IP,), daemon=True)
    jobs = [p]
    p.start()
    for proc in jobs:
        proc.join()
    sleep(5)
    flag_ctr += 1
    if flag_ctr == max_count:
        p.terminate()
        # break
    return flag_ctr


def update_table_ca(df, result, myID):
    print(result)
    """
    function to update the mutual authentication table with the CA result.
    Note: this function should be called after the CA result is received.
    Note2: the new table is not being saved only converted to json and sent to the neighbors.
    """
    #myID = int(list(set(df.loc[df['IP'] == co.get_ip_address(MESHINT), "ID"]))[0])
    df = df.assign(CA_Result=0)
    for index, row in df.iterrows():
        for res in result:
            for ip in res.keys():
                if row['IP'] == ip:
                    IP = row['IP']
                    if res[ip] in ['pass', 1]:
                        df.loc[index, 'CA_Result'] = 1
                    elif res[ip] == 'fail' or res[ip] == 0:
                        df.loc[df['IP'] == IP, 'CA_Result'] = 2
                    else:
                        df.loc[df['IP'] == IP, 'CA_Result'] = 3
                    # df.loc[df['IP'] == IP, 'CA_ts'] = time()
                    df.loc[df['IP'] == IP, 'CA_Server'] = myID
    df.drop_duplicates(inplace=True)
    print('result: ', result)
    print('df: ', df)
    return df
