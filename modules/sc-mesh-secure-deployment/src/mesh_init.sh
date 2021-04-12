#!/bin/bash

function help
{
    echo
    echo "Usage: sudo ./mesh_init.sh <ip> <AP MAC> <WEP key> <essid> <freq> <interface>"
    echo
    echo "Parameters:"
    echo "	<ip>"
    echo "	<AP MAC>"
    echo "	<WEP key>"
    echo "	<essid>"
    echo "	<freq>"
    echo
    echo "example:"
    echo "sudo ./mesh_init2.sh 192.168.1.2 00:11:22:33:44:55 1234567890 mymesh 5180 wlan1 "
    exit
}

if [[ -z "$1" || -z "$2" || -z "$3" || -z "$4" || -z "$5" ]]
  then
    echo "No arguments supplied"
    help
fi

echo "Killing wpa_supplicant..."
killall wpa_supplicant 2>/dev/null
nmcli network off



modprobe batman-adv

echo "$6 down.."
iw dev $6 del
#iw phy phy0 interface add $6 type ibss
iw phy phy1 interface add $6 type ibss


echo "$6 create adhoc.."
ifconfig $6 mtu 1560

cat <<EOF >/var/run/wpa_supplicant-adhoc.conf
ctrl_interface=DIR=/var/run/wpa_supplicant
# use 'ap_scan=2' on all devices connected to the network
# this is unnecessary if you only want the network to be created when no other networks..
ap_scan=1
p2p_disabled=1
network={
    ssid="$4"
    bssid=$2
    mode=1
    frequency=$5
    wep_key0=$3
    wep_tx_keyidx=0
    key_mgmt=NONE
}
EOF

echo "$6 up.."
ip link set $6 up
#wpa_supplicant -B -i $6 -c /var/run/wpa_supplicant-adhoc.conf -D nl80211 -C /var/run/wpa_supplicant/

batctl if add $6

echo "$6 bat0 up.."
ifconfig bat0 up
ifconfig bat0 $1/16
echo
ifconfig bat0
echo
echo "network-manager.service status:"
systemctl is-active network-manager.service 2>/dev/null
echo "networking status:"
nmcli networking 2>/dev/null || echo "inactive"

while :
do
wpa_supplicant -i $6 -c /var/run/wpa_supplicant-adhoc.conf -D nl80211 -C /var/run/wpa_supplicant/
sleep 10
done
