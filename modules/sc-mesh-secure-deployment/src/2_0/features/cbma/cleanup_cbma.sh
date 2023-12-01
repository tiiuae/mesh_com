#!/bin/bash
# Deletes macsec links, batman and bridges created within cbma
ip macsec show | grep ': protect on validate' | awk -F: '{print $2}' | awk '{print $1}' | xargs -I {} ip link delete {}
ifconfig bat0 down
ifconfig bat1 down
batctl meshif bat0 interface destroy
batctl meshif bat1 interface destroy
ip link set br-upper down
brctl delbr br-upper