#!/bin/bash

function help
{
    echo
    echo "Usage: sudo ./mesh-ibss.sh <mode> <ip> <mask> <AP MAC> <key> <essid> <freq> <txpower> <country> <interface> <phyname>"
    echo "Parameters:"
    echo "	<mode>"
    echo "	<ip>"
    echo "	<mask>"
    echo "	<AP MAC>"
    echo "	<WEP key>"
    echo "	<essid>"
    echo "	<freq>"
    echo "	<txpower>"
    echo "	<country>"
    echo "	<interface>" - optional
    echo "	<phyname>"   - optional
    echo
    echo "example:"
    echo "sudo ./mesh-ibss.sh mesh 192.168.1.2 255.255.255.0 00:11:22:33:44:55 1234567890 mymesh 5220 30 fi wlan1 phy1"
    echo "sudo mesh.sh ap"
    exit
}

find_mesh_wifi_device()
{
  # arguments:
  # $1 = wifi device vendor
  # $2 = wifi device id list

  # return values: retval_phy, retval_name as global

  echo "$1 $2"
  echo "Find WIFI card deviceVendor=$1 deviceID=$2"
  echo
  phynames=$(ls /sys/class/ieee80211/)

  for device in $2; do
    echo "$device"
    for phy in $phynames; do
      device_id="$(cat /sys/bus/pci/devices/*/ieee80211/"$phy"/device/device 2>/dev/null)"
      device_vendor="$(cat /sys/bus/pci/devices/*/ieee80211/"$phy"/device/vendor 2>/dev/null)"
      if [ "$device_id" = "$device" -a "$device_vendor" = "$1" ]; then
        retval_phy=$phy
        retval_name=$(ls /sys/class/ieee80211/"$phy"/device/net/)
        break 2
      else
        retval_phy=""
        retval_name=""
      fi
    done
  done
}


# 1      2    3      4        5     6       7      8         9         10
# <mode> <ip> <mask> <AP MAC> <key> <essid> <freq> <txpower> <country> <interface>

echo "Solving wifi device name.."
if [[ -z "${10}" ]]; then
  rfkill unblock all
  # multiple wifi options --> can be detected as follows:
  # manufacturer 0x168c = Qualcomm
  # devices = 0x0034 0x003c 9462/988x
  #           0x003e        6174
  find_mesh_wifi_device 0x168c "0x003e 0x0034 0x003c"

  if [ "$retval_phy" != "" ]; then
      phyname=$retval_phy
      wifidev=$retval_name
  else
      echo "ERROR! Can't find correct wifi device!"
      exit 1
  fi
else
  wifidev=${10}
  phyname=${11}
fi
echo "Found: $wifidev $phyname"

case "$1" in

mesh)

echo "sudo mesh $1 $2 $3 $4 $5 $6 $7 $8 $9 ${10} ${11}"
      if [[ -z "$1" || -z "$2" || -z "$3" || -z "$4" || -z "$5" || -z "$6" ]]
        then
          echo "check arguments..."
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
      # FIXME: If there is another Wi-Fi module being used as an AP for a GW,
      # this kills that process. We need a better way of handling this. For now
      # we can just not kill wpa_supplicant when we are loading the mesh_com_tb
      # module.
      if [[ -z "${10}" ]]; then
        pkill -f "/var/run/wpa_supplicant-" 2>/dev/null
        rm -fr /var/run/wpa_supplicant/"$wifidev"
      fi
      killall alfred 2>/dev/null
      killall batadv-vis 2>/dev/null
      rm -f /var/run/alfred.sock

      modprobe batman-adv

      echo "$wifidev down.."
      iw dev "$wifidev" del
      iw phy "$phyname" interface add "$wifidev" type ibss

      echo "$wifidev create adhoc.."
      ifconfig "$wifidev" mtu 1560

      echo "$wifidev up.."
      ip link set "$wifidev" up
      batctl if add "$wifidev"

      echo "bat0 up.."
      ifconfig bat0 up
      echo "bat0 ip address.."
      ifconfig bat0 "$2" netmask "$3"
      echo
      ifconfig bat0

      sleep 3

      # for visualisation
      (alfred -i bat0 -m)&
      echo "started alfred"
      (batadv-vis -i bat0 -s)&
      echo "started batadv-vis"

      # FIXME: Like the comment above - we need to figure out how to handle
      # multiple Wi-Fi interfaces better. For some reason the background setting
      # ensures wpa_supplicant doesn't start when this script is run as a process.
      # This is likely due to the interface not being up in time, and will
      # require some fiddling with the systemd startup order.
      if [[ -z "${10}" ]]; then
        wpa_supplicant -i "$wifidev" -c /var/run/wpa_supplicant-adhoc.conf -D nl80211 -C /var/run/wpa_supplicant/ -B -f /tmp/wpa_supplicant_ibss.log
      else
        wpa_supplicant -i "$wifidev" -c /var/run/wpa_supplicant-adhoc.conf -D nl80211 -C /var/run/wpa_supplicant/ -f /tmp/wpa_supplicant_ibss.log
      fi
      ;;

ap)
      if [[ -z "$1" ]]
        then
          echo "check arguments..."
        help
      fi

      # find ap parameters from enclave
      if [ -z "$DRONE_DEVICE_ID" ]
      then
        echo "DRONE_DEVICE_ID not available"

cat <<EOF >/var/run/wpa_supplicant-ap.conf
ctrl_interface=DIR=/var/run/wpa_supplicant
ap_scan=1
p2p_disabled=1
network={
    ssid="debug-hotspot"
    mode=2
    frequency=5220
    key_mgmt=WPA-PSK
    psk="1234567890"
    pairwise=CCMP
    group=CCMP
    proto=RSN
}
EOF
      else
        echo "DRONE_DEVICE_ID available"
cat <<EOF >/var/run/wpa_supplicant-ap.conf
ctrl_interface=DIR=/var/run/wpa_supplicant
ap_scan=1
p2p_disabled=1
network={
    ssid="$DRONE_DEVICE_ID"
    mode=2
    frequency=5220
    key_mgmt=WPA-PSK
    psk="1234567890"
    pairwise=CCMP
    group=CCMP
    proto=RSN
}
EOF
      fi

      echo "Killing wpa_supplicant..."
      pkill -f "/var/run/wpa_supplicant-" 2>/dev/null
      rm -fr /var/run/wpa_supplicant/"$wifidev"

      echo "create $wifidev"
      iw dev "$wifidev" del
      iw phy "$phyname" interface add "$wifidev" type managed

      echo "$wifidev up.."
      ip link set "$wifidev" up
      batctl if add "$wifidev"

      echo "set ip address.."
	    # set AP ipaddr
	    ifconfig "$wifidev" 192.168.1.1 netmask 255.255.255.0

	    # set bat0 ipaddr
      if [ -z "$DRONE_DEVICE_ID" ]
        then
          # DRONE_DEVICE_ID not available set default
          ifconfig bat0 192.168.1.1 netmask 255.255.255.0
        else
          declare -i ip=10#$(echo "$DRONE_DEVICE_ID" | tr -d -c 0-9)
          ifconfig bat0 192.168.1."$ip" netmask 255.255.255.0
      fi

      echo "bat0 up.."
      ifconfig bat0 up
      echo
      ifconfig bat0

      route del -net 192.168.1.0 netmask 255.255.255.0 dev bat0
      route add -net 192.168.1.0 netmask 255.255.255.0 dev bat0 metric 1

      #TODO
      # dhserver

      wpa_supplicant -B -i "$wifidev" -c /var/run/wpa_supplicant-ap.conf -D nl80211 -C /var/run/wpa_supplicant/ -f /tmp/wpa_supplicant_ap.log

      ;;


off)
      # service off
      pkill -f "/var/run/wpa_supplicant-" 2>/dev/null
      rm -fr /var/run/wpa_supplicant/"$wifidev"
      killall alfred 2>/dev/null
      killall batadv-vis 2>/dev/null
      rm -f /var/run/alfred.sock 2>/dev/null
      ;;
*)
      help
      ;;
esac
