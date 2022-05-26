import subprocess
import fcntl
import struct
import socket


def get_macs():
    """
    get the mac address from batctl n and parsing them
    return a list of macs
    """
    macs = []
    proc = subprocess.call(['batctl', 'n'], shell=False)
    for i, x in enumerate(proc.stdout, start=1):
        if i > 2:
            aux = x.split()
            macs.append((aux[1]).decode("utf-8"))
    return macs


def verify_mesh_status():
    """
    verify the output of the batctl n. If there is more than one line means that mesh was established
    return False/True
    """
    macs = get_macs()
    return len(macs) > 1


def get_mesh_ip_address(ifname='bat0'):
    """
    function to get the ip address.
    Another option can be to get the ip from the conf file, but in this case it will need to be loaded here.
    return str
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', bytes(ifname[:15], 'utf-8')))[20:24])