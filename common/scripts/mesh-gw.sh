#!/bin/bash

# function help
# {
#     echo
#     echo "Usage: sudo ./mesh-gw.sh <interface>"
#     echo "Parameters:"
#     echo "	<interface>"
#     echo
#     echo "example:"
#     echo "sudo ./mesh-gw.sh wlan0"
#     exit
# }

# 1
# <interface>

batctl gw_mode server 500mbit/500mbit
# TODO: BMX7 support
# bmx7 -c tunOut -inet4
# bmx7 -c tunIn inet4 /n 0.0.0.0/0
dhclient $1 -v

# Forward traffic from wlx to bat0 and vice versa
sysctl -w net.ipv4.ip_forward=1
iptables -t nat -A POSTROUTING -o $1 -j MASQUERADE
iptables -A FORWARD -i $1 -o bat0 -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT
iptables -A FORWARD -i bat0 -o $1 -j ACCEPT
