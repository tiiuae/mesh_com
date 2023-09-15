import subprocess
import fcntl
import struct
import socket
import netifaces
import yaml

from threading import Thread

try:
    import queue
except ImportError:
    import Queue as queue

def get_mesh_interface_from_file():
    '''
    get the mesh interface (br-lan in case of bridge, bat0 if no bridge) from common/mesh_com_11s.conf
    '''
    with open('common/mesh_com_11s.conf', 'r') as stream:
        try:
            yaml_conf = yaml.safe_load(stream)
            confs = yaml_conf['server']
            config = confs['secos']
            return config['meshint']
        except yaml.YAMLError as exc:
            print(exc)
            return None

def get_macs_neighbors():
    """
    get the mac address from batctl n and parsing them
    return a list of macs
    """
    macs = []
    cmd = ['batctl', 'n']
    proc = subprocess.run(cmd, capture_output=True, shell=False)
    splitout = proc.stdout.decode().split('\n')
    if len(splitout) > 3:
        macs.extend(aux.split('\t')[1].split(' ')[2] for aux in splitout[2:-1])
    return macs


def get_mac_mesh(pattern=get_mesh_interface_from_file()):
    for interf in netifaces.interfaces():
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


def get_mesh_ip_address(ifname=get_mesh_interface_from_file()):
    """
    function to get the ip address.
    Another option can be to get the ip from the conf file, but in this case it will need to be loaded here.
    return str
    """
    with  socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        return socket.inet_ntoa(fcntl.ioctl(
            s.fileno(),
            0x8915,  # SIOCGIFADDR
            struct.pack('256s', bytes(ifname[:15], 'utf-8')))[20:24])


def get_mesh_interface(pattern=get_mesh_interface_from_file()):
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
    ips = []
    final = []
    my_ip = get_mesh_ip_address()
    aux = my_ip.split('.')[:3]
    for i in range(1, 255):
        if str(i) != my_ip.split('.')[3]:
            ips.append('.'.join(aux) + '.' + str(i)) # All ips in the mesh subnet except my_ip

    threads = []

    def thread_pinger(ip):
        #args = ['/bin/ping', '-c', '1', '-W', '1', str(ip)]
        args = ['/bin/ping', '-c', '1', str(ip)]
        with subprocess.Popen(args, shell=False, stdout=subprocess.PIPE) as p_ping:

        # save ping stdout
        #p_ping_out = str(p_ping.communicate()[0])

            if p_ping.wait() == 0:
                # rtt min/avg/max/mdev = 22.293/22.293/22.293/0.000 ms
                # search = re.search(r'rtt min/avg/max/mdev = (.*)/(.*)/(.*)/(.*) ms',p_ping_out, re.M | re.I)
                # out_q.put(str(ip))
                final.append(ip)

    for ip in ips:
        worker = Thread(target=thread_pinger, args=(ip,), daemon=True)
        threads.append(worker)
        worker.start()

    for thread in threads:
        thread.join()

    #final.remove(my_ip)
    return final

def get_arp():
    neig = {}
    # get_neighbors_ip()
    # time.sleep(2)
    args = ['ip', 'neigh', 'show', 'dev', get_mesh_interface_from_file(), 'nud', 'stale']
    output = subprocess.check_output(args, shell=False)

    aux = output.decode().split('\n')
    aux.remove('')
    for entry in aux:
        string = entry.split()
        ip = string[0]
        if len(ip.split('.')) > 1:
            mac = string[-2]
            neig[ip] = mac
    return neig


