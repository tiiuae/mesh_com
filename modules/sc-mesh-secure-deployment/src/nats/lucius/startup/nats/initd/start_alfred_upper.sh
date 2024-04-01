#!/bin/sh

ip link del bat1 2>/dev/null
ip link add name bat1 type batadv
batctl meshif bat1 interface create routing_algo BATMAN_V
ip link set dev bat1 master br-lan
ip link set dev bat1 up

/usr/bin/alfred -b bat1 -i bat0 -m -u /var/run/alfred_upper.sock