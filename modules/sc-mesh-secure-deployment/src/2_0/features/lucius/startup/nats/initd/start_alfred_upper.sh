#!/bin/sh

echo "wait for bat1 to come up"

tail -F /sys/class/net/bat1/address | awk '{exit}'
ip link set dev bat1 up

/usr/bin/alfred -b bat1 -i bat0 -m -u /var/run/alfred_upper.sock
