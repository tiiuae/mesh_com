#!/bin/bash

batctl gw_mode server 500mbit/500mbit
# TODO: BMX7 support
# bmx7 -c tunOut -inet4
# bmx7 -c tunIn inet4 /n 0.0.0.0/0
dhclient wlan0 -v
