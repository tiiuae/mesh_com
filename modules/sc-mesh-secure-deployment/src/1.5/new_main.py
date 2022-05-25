import subprocess
from features.mutual.mutual import mutual
from features.continuos import continuos


def get_macs():
    macs = []
    proc = subprocess.Popen(['batctl', 'n'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    i = 0
    for x in proc.stdout:
        aux = x.split()
        i += 1
        if i > 2:
            macs.append((aux[1]).decode("utf-8"))
    return macs


def verify_mesh_status():
    macs = get_macs()
    if len(macs) > 1:
        return True
    else:
        return False


def launchCA(neigh):  # need to see how to get the IP address of the neighbors
    '''
    this function should start server within localhost and client within the neighbors
    It should be the trigger in time-bases ex: every X seconds
    '''
    co = continuos.Continous()
    for ne in neigh:
        co.client('localhost')
        co.server(ne)


if __name__ == "__main__":
    mut = mutual.Mutual('wlan0')
    mut.start()
    if verify_mesh_status():
        table = mutual.get_status()  # this should be triggered every x second
        neighbors = get_macs()
        launchCA(neighbors)
