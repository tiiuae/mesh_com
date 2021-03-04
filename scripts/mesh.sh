#!/bin/bash

function help
{
    echo
    echo "Usage: sudo ./mesh_init.sh <mode> <ip> <mask> <AP MAC> <key> <essid> <freq> <txpower> <country>"
    echo
    echo "example:"
    echo "sudo mesh.sh mesh 192.168.1.2 255.255.255.0 00:11:22:33:44:55 1234567890 mymesh2 5220 30 fi"
    echo "sudo mesh.sh ap"
    exit
}

echo "Solving wifi device name.."
wifidev=$(iw dev | awk '$1=="Interface"{print $2}')
echo "Found: $wifidev"

# 1      2    3      4        5     6       7      8         9   
# <mode> <ip> <mask> <AP MAC> <key> <essid> <freq> <txpower> <country>

case "$1" in

mesh)
      if [[ -z "$1" || -z "$2" || -z "$3" || -z "$4" || -z "$5" || -z "$6" ]]
        then
          echo "check argumets..."
        help
      fi

cat <<EOF >/var/run/wpa_supplicant-adhoc.conf
ctrl_interface=DIR=/var/run/wpa_supplicant
# use 'ap_scan=2' on all devices connected to the network
# this is unnecessary if you only want the network to be created when no other networks..
ap_scan=1
country=$9
p2p_disabled=1
network={
    ssid="$6"
    bssid=$4
    mode=1
    frequency=$7
    wep_key0=$5
    wep_tx_keyidx=0
    key_mgmt=NONE
}
EOF

      echo "Killing wpa_supplicant..."
      killall wpa_supplicant 2>/dev/null
      killall alfred 2>/dev/null
      killall batadv-vis 2>/dev/null
      rm -f /var/run/alfred.sock


      echo "unmanage wifi interface.."
      nmcli dev set $wifidev managed no
      modprobe batman-adv

      echo "$wifidev down.."
      iw dev $wifidev del
      iw phy phy0 interface add $wifidev type ibss

      echo "$wifidev create adhoc.."
      ifconfig $wifidev mtu 1560

      echo "$wifidev up.."
      ip link set $wifidev up
      batctl if add $wifidev

      echo "bat0 up.."
      ifconfig bat0 up
      echo "bat0 ip address.."
      ifconfig bat0 $2 netmask $3
      echo
      ifconfig bat0

      # Alfred is not able to create socket without this delay
      sleep 3

      # for visualisation
      (alfred -i bat0 -m)&
      echo "started alfred"
      (batadv-vis -i bat0 -s)&
      echo "started batadv-vis"

      wpa_supplicant -i $wifidev -c /var/run/wpa_supplicant-adhoc.conf -D nl80211 -C /var/run/wpa_supplicant/ -B
      ;;
ap)
      if [[ -z "$1" ]]
        then
          echo "check arguments..."
        help
      fi

      # remove all saved wifi networks
      nmcli --pretty --fields UUID,TYPE con show | grep wifi | cut -b -37 | while read line; do nmcli con delete uuid  $line; done

      nmcli dev set $wifidev managed yes
      nmcli radio wifi on

      # find ap parameters from enclave
      if [ -z "$DRONE_DEVICE_ID" ]
      then
            echo "DRONE_DEVICE_ID not available"
            nmcli device wifi hotspot con-name debug-hotspot ssid debug_hotspot band a channel 36 password 1234567890
      else
            nmcli device wifi hotspot con-name debug-hotspot ssid $DRONE_DEVICE_ID band a channel 36 password 1234567890
      fi

      ;;
*)
      help
      ;;
esac

