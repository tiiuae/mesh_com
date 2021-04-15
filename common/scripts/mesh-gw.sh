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
