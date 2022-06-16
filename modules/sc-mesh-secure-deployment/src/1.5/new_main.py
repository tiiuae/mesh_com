from features.mutual import mutual
from features.continuos import continuos
from features.mba import mba
from features.utils import mesh_utils
from threading import Thread
import queue


def launchCA(neigh):  # need to see how to get the IP address of the neighbors
    '''
    this function should start server within localhost and client within the neighbors
    It should be the trigger in time-bases ex: every X seconds
    '''
    co = continuos.Continous()
    for ne in neigh:
        co.client('localhost')
        co.server(ne)


def listeningMBA():
    q = queue.Queue()
    mal = mba.MBA(mesh_utils.get_mesh_ip_address())
    Thread(target=mal.client, args=(q,)).start()
    return q.get()


def announcing(message):
    mal = mba.MBA(mesh_utils.get_mesh_ip_address())
    Thread(target=mal.server, args=(message, True,), daemon=True).start()


if __name__ == "__main__":
    mut = mutual.Mutual('wlan0')
    mut.start()
    if mesh_utils.verify_mesh_status():
        table = mutual.get_status()  # this should be triggered every x second
        neighbors = mesh_utils.get_macs()
        launchCA(neighbors)
