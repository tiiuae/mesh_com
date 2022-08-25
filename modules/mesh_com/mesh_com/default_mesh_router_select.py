# This script finds the best gateway for the Default Mesh network based on 'batctl gwl' TQ parameter

import subprocess
import re
import syslog
import psutil
import time

INTERFACE_READY_RETRY_COUNT = 120
TABLE_READ_RETRY_COUNT = 60
FETCH_IP_RETRY_COUNT = 10
INTERFACE_NAME = "bat0"

# Table comparison code block
# Find the best TransmissionQuality router and its virtual MAC
def select_best_tq(l):
    best_val = 0
    best = None
    for elem in l:
        if elem[1] > best_val:
            best_val = elem[1]
            best = elem
    return best


def select_best_tq_2(l):
    return l[(tqs := list(map(lambda elem: elem[1], l))).index(max(tqs))]

# Find the best_TQ_router_physical_adapter MAC
def match_tg(mac, l):
    result = []
    for elem in l:
        if elem[1] == mac:
            result.append(elem)
    return result


def match_tg_2(mac, l):
    return filter(lambda elem: elem[1] == mac, l)

# Find IP for the gateway with the best TQ
def find_ip(dc, tg):
    dc_map = {}
    for elem in dc:
        dc_map[elem[1]] = elem[0]

    for elem in tg:
        if elem[0] in dc_map:
            return dc_map[elem[0]]

    # We should never get here
    return None

def find_best_gateway_ip(gwl, dc, tg):
    gwl = [[elem[0], int(elem[1])] for elem in gwl]
    best_tq = select_best_tq(gwl)
    candidates = match_tg_2(best_tq[0], tg)
    ip = find_ip(dc, candidates)
    return ip

def is_interface_up_and_got_carrier(interface):
    wait_message = "Wait 1s and try again. Attempt {:d}/{:d}"
    for attempt in range(1, INTERFACE_READY_RETRY_COUNT + 1):
        # Check if interface exists and it is up.
        if interface in (stats := psutil.net_if_stats()) and stats[interface].isup:
            print("Interface [{:s}] is up.".format(interface))
            break
        print("Interface [{:s}] is down.".format(interface))
        print(wait_message.format(attempt, INTERFACE_READY_RETRY_COUNT))
        time.sleep(1)
    else:
        print("Waited for network interface, but it was not available.")
        raise IOError

    for attempt in range(attempt + 1, INTERFACE_READY_RETRY_COUNT + 1):
        try:
            # Check that the link carrier has been acquired.
            with open("/sys/class/net/{:s}/carrier".format(interface), "r") as carrier_file:
                dat = carrier_file.readline().strip()
            if dat == "1":
                print("Got carrier in interface [{:s}].".format(interface))
                break
            else:
                print("No carrier in interface [{:s}].".format(interface))
                print(wait_message.format(attempt, INTERFACE_READY_RETRY_COUNT))
                time.sleep(1)
        except FileNotFoundError:
            print("Carrier file does not exist (yet).")
            print(wait_message.format(attempt, INTERFACE_READY_RETRY_COUNT))
            time.sleep(1)
    else:
        print("Waited for network interface, but it was not available.")
        raise IOError

is_interface_up_and_got_carrier(INTERFACE_NAME)

# Define MAC, IP and TQ patterns for RE
parse_mac=re.compile(r'(?:[0-9a-fA-F]:?){12}')
parse_ip=re.compile("\d{1,3}.\d{1,3}.\d{1,3}.\d{1,3}")
parse_tq=re.compile(r'\(([0-9]+)\)')

for _ in range(TABLE_READ_RETRY_COUNT):
    # Parse '# batctl tg' output
    try:
        batctl_tg=str(subprocess.Popen(['batctl', 'tg'], stdout=subprocess.PIPE).communicate()[0])
        batctl_tg_err=str(subprocess.Popen(['batctl', 'tg'], stdout=subprocess.PIPE).communicate()[1])
        batctl_tg=batctl_tg.split("\n")
        batctl_tg=batctl_tg[0].split('\\')
    except:
        print(batctl_tg_err)
        syslog.syslog(batctl_tg_err)
    
    batctl_tg_list=[]
    for i in range(2, len(batctl_tg)-1):
        a=re.findall(parse_mac, batctl_tg[i])
        batctl_tg_list.append(a)

    if len(batctl_tg_list):
        break
    else:
        print("BATMAN TransGlobal table empty")
        syslog.syslog("BATMAN TransGlobal table empty")
        time.sleep(1)
else:
    print("Waited for BATMAN TransGlobal table, but it was not available.")
    raise IOError

for _ in range(TABLE_READ_RETRY_COUNT):
    # Parse '# batctl dc' output
    try:
        batctl_dc=str(subprocess.Popen(['batctl', 'dc'], stdout=subprocess.PIPE).communicate()[0])
        batctl_dc_err=str(subprocess.Popen(['batctl', 'dc'], stdout=subprocess.PIPE).communicate()[1])
        batctl_dc=batctl_dc.split("\n")
        batctl_dc=batctl_dc[0].split('\\')
    except:
        print(batctl_dc_err)
        syslog.syslog(batctl_dc_err)

    batctl_dc_list=[]
    for i in range(3, len(batctl_dc)-1):
        a=re.findall(parse_mac, batctl_dc[i])
        b=re.findall(parse_ip, batctl_dc[i])
        batctl_dc_list.append([b[0], a[0]])

    if len(batctl_dc_list):
        break
    else:
        print("BATMAN Distributed ARP Table empty")
        syslog.syslog("BATMAN Distributed ARP Table empty")
        time.sleep(1)
else:
    print("Waited for BATMAN ARP table, but it was not available.")
    raise IOError

for _ in range(TABLE_READ_RETRY_COUNT):
    # Parse '# batctl gwl' output
    try:
        batctl_gwl=str(subprocess.Popen(['batctl', 'gwl'], stdout=subprocess.PIPE).communicate()[0])
        batctl_gwl_err=str(subprocess.Popen(['batctl', 'gwl'], stdout=subprocess.PIPE).communicate()[1])
        batctl_gwl=batctl_gwl.split("\n")
        batctl_gwl=batctl_gwl[0].split('\\')
    except:
        print(batctl_gwl_err)
        syslog.syslog(batctl_tg_err)

    batctl_gwl_list=[]
    for i in range(2, len(batctl_gwl)-1):
        a=re.findall(parse_mac, batctl_gwl[i])
        b=re.findall(parse_tq, batctl_gwl[i])
        if not len(a) or not len(b):
            print("Invalid parameters.")
            syslog.syslog("Invalid parameters.")
            time.sleep(1)
            continue
        batctl_gwl_list.append([a[0], b[0]])

    if len(batctl_gwl_list):
        break
    else:
        print("BATMAN Gateway List empty")
        syslog.syslog("BATMAN Gateway List empty")
        time.sleep(1)
else:
    print("Waited for BATMAN Gateway List, but it was not available.")
    raise IOError

for _ in range(FETCH_IP_RETRY_COUNT):
    ip = find_best_gateway_ip(batctl_gwl_list, batctl_dc_list, batctl_tg_list)
    if ip is not None:
        break
    time.sleep(1)
else:
    print("Waited for best gateway, but it was not available.")
    raise IOError

print(ip)
