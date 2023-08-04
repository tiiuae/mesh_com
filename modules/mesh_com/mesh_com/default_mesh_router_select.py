#!/usr/bin/env python3
#Replaced by primitive ping-based gateway selection script for demo purposes
import os
import time
import subprocess
import re

def check_gateway(gateway):
    try:
        print(f"Trying to ping {gateway}...")
        # Use subprocess to execute the ping command with a 1-second timeout
        ping_process = subprocess.Popen(['ping', '-c', '1', '-W', '1', gateway], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        ping_output, ping_error = ping_process.communicate()

        # Decode the output to a string and print it
        # print(ping_output.decode())

        # Check if the ping process was successful
        if ping_process.returncode == 0:
            return True  # Gateway is reachable
        else:
            return False  # Gateway is unreachable

    except subprocess.CalledProcessError:
        return False  # Gateway is unreachable

def switch_gateway(gateway):
    print(f"Switching default FOG gateway to {gateway}...")
    os.system(f"ip route del default")
    os.system(f"ip route add default via {gateway}")
    print(f"Default FOG gateway switched to {gateway}.")

def scan_available_gateways(gateways):
    while True:
        available_gateway = None
        for gateway in gateways:
            if check_gateway(gateway):
                available_gateway = gateway
                break

        if available_gateway:
            return available_gateway

        print("All gateways are currently unavailable. Starting the scan again in 2 seconds...")
        time.sleep(2)  # Wait for 2 seconds before starting the scan again

# Check drone role. Here is also a dummy for adding other drone roles to the array
drone_type = os.environ.get('DRONE_TYPE')
if drone_type in ['recon']:
    # Extract the gateway range from the 'MESH_FOG_GW_LIST' environment variable
    gw_list_value = os.environ.get('MESH_FOG_GW_LIST')
    match = re.match(r'(\d+\.\d+\.\d+\.\d+)-(\d+)', gw_list_value)
    
    if match:
        start_ip = match.group(1)
        end_octet = match.group(2)
        
        # Create the full IP address for the end_ip
        end_ip = f"{start_ip.rsplit('.', 1)[0]}.{end_octet}"

        # Create the gateways list based on the extracted range
        gateways = [f"{start_ip.rsplit('.', 1)[0]}.{i}" for i in range(int(start_ip.rsplit('.', 1)[1]), int(end_octet) + 1)]
        print ('gateway list:', start_ip, '-', end_ip)
    else:
        print("Invalid gateway range format in gw_list. Please check the environment variable.")
        exit(1)


    # Set the default gateway to the first entry
    default_gateway = gateways[0]

    # Variable to track the downtime of the current gateway
    current_gateway_down_time = 0

    # Variable to track the previous gateway status
    prev_gateway_status = False

    # Check the availability of the first gateway before setting it as default
    if not check_gateway(default_gateway):
        available_gateway = scan_available_gateways(gateways)
        if available_gateway:
            switch_gateway(available_gateway)
            default_gateway = available_gateway

    # Loop continuously to monitor the gateway
    while True:
        current_gateway_status = check_gateway(default_gateway)

        if current_gateway_status:
            # Reset the downtime counter if the gateway is reachable
            current_gateway_down_time = 0
        else:
            current_gateway_down_time += 1

            # Start available gateways check procedure if the downtime is 10 seconds or more
            if current_gateway_down_time >= 10:
                # Wait for 2 seconds before switching to the next gateway
                time.sleep(2)

                available_gateway = scan_available_gateways(gateways)
                if available_gateway:
                    switch_gateway(available_gateway)
                    default_gateway = available_gateway
                    current_gateway_down_time = 0  # Reset downtime counter

        # Print debug message when gateway status changes
        if prev_gateway_status != current_gateway_status:
            if current_gateway_status:
                print(f"Gateway ({default_gateway}) is now reachable.")
            else:
                print(f"Gateway ({default_gateway}) is now unreachable.")

        prev_gateway_status = current_gateway_status

        # Wait for 1 second before checking the current gateway again
        time.sleep(1)
else:
    print("This drone type doen not support FOG gateway switching")

'''
# !!! Old code behaves strange because of unpredictable behaviour of BATMAN distributed ARP tables 
#This script finds the best gateway for the Default Mesh network based on 'batctl gwl' TQ parameter

import subprocess
import re
import sys
import syslog
import psutil
import time

INTERFACE_READY_RETRY_COUNT = 120
TABLE_READ_RETRY_COUNT = 60
FETCH_IP_RETRY_COUNT = 10
INTERFACE_NAME = "bat0"

# Define function for printing output to stderr stream
def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

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
            eprint("Interface [{:s}] is up.".format(interface))
            break
        eprint("Interface [{:s}] is down.".format(interface))
        eprint(wait_message.format(attempt, INTERFACE_READY_RETRY_COUNT))
        time.sleep(1)
    else:
        eprint("Waited for network interface, but it was not available.")
        raise IOError

    for attempt in range(attempt + 1, INTERFACE_READY_RETRY_COUNT + 1):
        try:
            # Check that the link carrier has been acquired.
            with open("/sys/class/net/{:s}/carrier".format(interface), "r") as carrier_file:
                dat = carrier_file.readline().strip()
            if dat == "1":
                eprint("Got carrier in interface [{:s}].".format(interface))
                break
            else:
                eprint("No carrier in interface [{:s}].".format(interface))
                eprint(wait_message.format(attempt, INTERFACE_READY_RETRY_COUNT))
                time.sleep(1)
        except FileNotFoundError:
            eprint("Carrier file does not exist (yet).")
            eprint(wait_message.format(attempt, INTERFACE_READY_RETRY_COUNT))
            time.sleep(1)
    else:
        eprint("Waited for network interface, but it was not available.")
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
        eprint(batctl_tg_err)
        syslog.syslog(batctl_tg_err)
    
    batctl_tg_list=[]
    for i in range(2, len(batctl_tg)-1):
        a=re.findall(parse_mac, batctl_tg[i])
        batctl_tg_list.append(a)

    if len(batctl_tg_list):
        break
    else:
        eprint("BATMAN TransGlobal table empty")
        syslog.syslog("BATMAN TransGlobal table empty")
        time.sleep(1)
else:
    eprint("Waited for BATMAN TransGlobal table, but it was not available.")
    raise IOError

for _ in range(TABLE_READ_RETRY_COUNT):
    # Parse '# batctl dc' output
    try:
        batctl_dc=str(subprocess.Popen(['batctl', 'dc'], stdout=subprocess.PIPE).communicate()[0])
        batctl_dc_err=str(subprocess.Popen(['batctl', 'dc'], stdout=subprocess.PIPE).communicate()[1])
        batctl_dc=batctl_dc.split("\n")
        batctl_dc=batctl_dc[0].split('\\')
    except:
        eprint(batctl_dc_err)
        syslog.syslog(batctl_dc_err)

    batctl_dc_list=[]
    for i in range(3, len(batctl_dc)-1):
        a=re.findall(parse_mac, batctl_dc[i])
        b=re.findall(parse_ip, batctl_dc[i])
        batctl_dc_list.append([b[0], a[0]])

    if len(batctl_dc_list):
        break
    else:
        eprint("BATMAN Distributed ARP Table empty")
        syslog.syslog("BATMAN Distributed ARP Table empty")
        time.sleep(1)
else:
    eprint("Waited for BATMAN ARP table, but it was not available.")
    raise IOError

for _ in range(TABLE_READ_RETRY_COUNT):
    # Parse '# batctl gwl' output
    try:
        batctl_gwl=str(subprocess.Popen(['batctl', 'gwl'], stdout=subprocess.PIPE).communicate()[0])
        batctl_gwl_err=str(subprocess.Popen(['batctl', 'gwl'], stdout=subprocess.PIPE).communicate()[1])
        batctl_gwl=batctl_gwl.split("\n")
        batctl_gwl=batctl_gwl[0].split('\\')
    except:
        eprint(batctl_gwl_err)
        syslog.syslog(batctl_tg_err)

    batctl_gwl_list=[]
    for i in range(2, len(batctl_gwl)-1):
        a=re.findall(parse_mac, batctl_gwl[i])
        b=re.findall(parse_tq, batctl_gwl[i])
        if not len(a) or not len(b):
            eprint("Invalid parameters.")
            syslog.syslog("Invalid parameters.")
            time.sleep(1)
            continue
        batctl_gwl_list.append([a[0], b[0]])

    if len(batctl_gwl_list):
        break
    else:
        eprint("BATMAN Gateway List empty")
        syslog.syslog("BATMAN Gateway List empty")
        time.sleep(1)
else:
    eprint("Waited for BATMAN Gateway List, but it was not available.")
    raise IOError

for _ in range(FETCH_IP_RETRY_COUNT):
    ip = find_best_gateway_ip(batctl_gwl_list, batctl_dc_list, batctl_tg_list)
    if ip is not None:
        break
    time.sleep(1)
else:
    eprint("Waited for best gateway, but it was not available.")
    raise IOError

print(ip)
'''