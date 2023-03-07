import multiprocessing
import pandas as pd
from time import time, sleep
import random
import numpy as np
import sys

from .ca_main import *
from ..mutual.mutual import *

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
    id_dict = manager.dict()

    neighbor_ips = mesh_utils.get_neighbors_ip()
    ip2_send = list(set(sectable['IP'].tolist() + neighbor_ips))

    #ip2_send = list(set(sectable['IP'].tolist() + list(neigh.keys())))
    print('Checkpoint, neighbor IPs = ', neighbor_ips)
    print('Checkpoint, ip2send = ', ip2_send)
    neigh = mesh_utils.get_arp()
    print('Checkpoint, IP_get_arp = ', list(neigh.keys()))

    num_neighbors = len(ip2_send) - 1 # Number of neighbor nodes that need to be authenticated
    server_proc = ca_server(myip, return_dict, id_dict, num_neighbors, list(set(sectable['IP'].tolist())))

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
    print('id_dict: ', id_dict)
    return client_q, id_dict


def ca_server(myip, return_dict, id_dict, num_neighbors, ips_sectable):
    with contextlib.suppress(OSError):
        ca = CA(random.randint(1000, 64000))
    proc = multiprocessing.Process(target=ca.as_server, args=(myip, return_dict, id_dict, num_neighbors, ips_sectable), daemon=True)
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

    for res_tuple in result:
        res, cliIDs = res_tuple
        for ip in res.keys():
            if ip in df['IP'].values:
                if res[ip] in ['pass', 1]:
                    df.loc[df['IP'] == ip, 'CA_Result'] = 1
                elif res[ip] == 'fail' or res[ip] == 0:
                    df.loc[df['IP'] == ip, 'CA_Result'] = 2
                else:
                    df.loc[df['IP'] == ip, 'CA_Result'] = 3
                df.loc[df['IP'] == ip, 'CA_Server'] = myID
            else:
                neigh = mesh_utils.get_arp()
                try:
                    mac = neigh[ip]
                except KeyError:
                    mac = '----'
                client_mesh_name = ip.replace('.', '_')
                client_fpr, _ = pri.hashSig(f'pubKeys/{client_mesh_name}.der')
                if res[ip] in ['pass', 1]:
                    CA_Result = 1
                elif res[ip] == 'fail' or res[ip] == 0:
                    CA_Result = 2
                else:
                    CA_Result = 3
                # ip is used as ID (Need to check)
                try:
                    cliID = cliIDs[ip]
                except KeyError:
                    cliID = ip
                info = {'ID': cliID, 'MAC': mac, 'IP': ip, 'PubKey_fpr': client_fpr, 'MA_level': 1, 'CA_Result': CA_Result,
                        'CA_Server': myID} # Check if MA_level should be 1 or something else
                df = df.append(info, ignore_index=True)
    """
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
    """
    df.drop_duplicates(inplace=True)
    print('result: ', result)
    print('df: ', df)

    return df
