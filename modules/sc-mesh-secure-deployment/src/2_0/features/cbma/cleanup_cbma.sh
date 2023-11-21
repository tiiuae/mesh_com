#!/bin/bash
# Deletes macsec links, batman and bridges created within cbma
ebtables -t nat -L OUTPUT | sed -n '/OUTPUT/,/^$/{/^--/p}' | xargs ebtables -t nat -D OUTPUT

ip macsec show | grep ': protect on validate' | awk -F: '{print $2}' | awk '{print $1}' | xargs -I {} ip link delete {}

ifconfig bat0 down
ifconfig bat1 down

batctl meshif bat0 interface destroy
batctl meshif bat1 interface destroy

ebtables --delete FORWARD --logical-in br-upper --jump ACCEPT
ebtables --delete FORWARD --logical-out br-upper --jump ACCEPT

ip link set br-upper down
brctl delbr br-upper
