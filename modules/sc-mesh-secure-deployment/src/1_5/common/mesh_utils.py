import subprocess
import fcntl
import struct
import socket
import netifaces

from threading import Thread

try:
    import queue
except ImportError:
    import Queue as queue
import re


def get_macs_neighbors():
    """
    get the mac address from batctl n and parsing them
    return a list of macs
    """
    macs = []
    proc = subprocess.run(['batctl', 'n'], capture_output=True)
    splitout = proc.stdout.decode().split('\n')
    if len(splitout) > 3:
        for aux in splitout[2:-1]:
            macs.append(aux.split('\t')[1].split(' ')[2])
    return macs


def get_mac_mesh(pattern):
    for interf in netifaces.interfaces():
        # TODO: what it if doesn't start with wlan???
        if interf.startswith(pattern):
            interface = interf
            mac = netifaces.ifaddresses(interface)[netifaces.AF_LINK]
            return mac[0]['addr']


def verify_mesh_status():
    """
    verify the output of the batctl n. If there is more than one line means that mesh was established
    return False/True
    """
    macs = get_macs_neighbors()
    return len(macs) >= 1


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


def get_mesh_interface(pattern):
    '''
    Using this function from previous script, to obtain the mesh_interface.
    Maybe it's redundant if secure OS will have 'wlan1' as default
    '''
    interface_list = netifaces.interfaces()
    interface = filter(lambda x: pattern in x, interface_list)
    pre = list(interface)
    if not pre:
        print('> ERROR: Interface ' + pattern + ' not found!')
    else:
        return pre[0]


def get_neighbors_ip():
    # some global vars
    num_threads = 15
    ips_q = queue.Queue()
    out_q = queue.Queue()
    my_ip = get_mesh_ip_address()
    aux = my_ip.split('.')[0:3]
    # build IP array
    ips = []
    final = []
    for i in range(1, 50):
        ips.append('.'.join(aux) + '.' + str(i))

    def thread_pinger(i, q):
        """Pings hosts in queue"""
        while True:
            # get an IP item form queue
            ip = q.get()
            # ping it
            args = ['/bin/ping', '-c', '1', '-W', '1', str(ip)]
            p_ping = subprocess.Popen(args,
                                      shell=False,
                                      stdout=subprocess.PIPE)
            # save ping stdout
            p_ping_out = str(p_ping.communicate()[0])

            if (p_ping.wait() == 0):
                # rtt min/avg/max/mdev = 22.293/22.293/22.293/0.000 ms
                search = re.search(r'rtt min/avg/max/mdev = (.*)/(.*)/(.*)/(.*) ms',
                                   p_ping_out, re.M | re.I)
                out_q.put(str(ip))

            # update queue : this ip is processed
            q.task_done()

    # start the thread pool
    for i in range(num_threads):
        worker = Thread(target=thread_pinger, args=(i ,ips_q), daemon=True).start()

    # fill queue
    for ip in ips:
        ips_q.put(ip)

    # wait until worker threads are done to exit
    ips_q.join()

    # print result
    while True:
        try:
            msg = out_q.get_nowait()
        except queue.Empty:
            break
        final.append(msg)
    final.remove(my_ip)
    return final
