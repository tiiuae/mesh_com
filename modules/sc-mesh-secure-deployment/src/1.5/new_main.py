import multiprocessing
from multiprocessing import Lock, Manager
import functools
import queue
import time
from threading import Thread
import asyncio
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor

import pandas as pd
from common import mesh_utils

from common import ConnectionMgr
from features.continuous import ca_main
from features.mba import mba
from features.mutual import mutual
import random
import asyncio



co = ConnectionMgr.ConnectionMgr()
_executor = ThreadPoolExecutor()
executor = ProcessPoolExecutor()

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
    max_count = 8
    flag_ctr = 0
    #lock = Lock()
    # server_tread = Thread(target=ca.as_server, args=(myip, server_q), daemon=True)
    # server_tread.start()
    proc = multiprocessing.Process(target=ca.as_server, args=(myip,)) #lock variable add later
    proc.start()
    time.sleep(random.randint(1,10))
    #proc.join() #This might crash it
    #with ProcessPoolExecutor() as pool:
    #        with Manager() as manager:
    for index, row in sectable.iterrows():
                                       
                    manager = multiprocessing.Manager()
                    return_dict = manager.dict()
                    jobs = []

                    IP = row['IP']
                    print(IP)

                    p = multiprocessing.Process(target=ca.as_client, args=(IP, return_dict,))
                    jobs.append(p)
                    p.start()

                    for proc in jobs:
                        proc.join()

                    client_q[IP] = return_dict.values()

                    flag_ctr += 1
                    if (flag_ctr == max_count):
                        p.terminate()
                        break
                                    
    proc.terminate()
    # server_tread._stop()
    return (client_q)


def update_table(df,client_q):
    df = df.assign(CA_Result='no')
    for index, row in df.iterrows():
        IP = row['IP']
        print(f"I AM PRININTG HERE!!!!! {client_q[IP]}")
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
#@asyncio.coroutine
#async def main():
    mut = mutual.Mutual(mutual_int)
    mut.start()
    time.sleep(5)
    if mesh_utils.verify_mesh_status():
        sectable = pd.read_csv('../auth/dev.csv')
        neighbors = mesh_utils.get_mac_mesh(mesh_int)
        result = launchCA(random.randint(1000, 64000), sectable) # result = yield from loop.run_in_executor(executor,launchCA,(random.randint(1000, 64000), sectable))
        sectable = update_table(sectable, result)

#asyncio.run(main())
#loop = asyncio.get_event_loop()
#loop.run_until_complete(main())
#loop.close()
