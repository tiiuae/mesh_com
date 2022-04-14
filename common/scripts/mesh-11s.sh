#!/bin/bash

function help
{
    echo
    echo "Usage: sudo $0 <mode> <ip> <mask> <AP MAC> <key> <essid> <freq> <txpower> <country> [<interface>] [<phyname>] [<routing_algo>]"
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
    echo "	<routing_algo>" - optional
    echo
    echo "example:"
    echo "sudo $0 mesh 192.168.1.2 255.255.255.0 00:11:22:33:44:55 1234567890 mymesh2 5220 30 fi wlan1 phy1"
    echo "sudo $0 ap"
    exit
}

calculate_wifi_channel()
{
    # arguments:
    # $1 = wifi frequency

    # return values: retval_band, retval_channel as global

    # Set 2.4/5GHz frequency band and channel
    if [ "$1" -ge  "5160" ] && [ "$1" -le  "5885" ]; then
        retval_band="a"
        retval_channel=$((("$1"-5000)/5))
    elif [ "$1" -ge  "2412" ] && [ "$1" -le  "2472" ]; then
        retval_band="g"
        retval_channel=$((("$1"-2407)/5))
    else
        echo "ERROR! frequency out of range!"
        exit 1
    fi
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
      if [ "$device_id" = "$device" ] && [ "$device_vendor" = "$1" ]; then
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
  # devices = 0x0034 0x003c 9462/988x  11s
  #           0x003e        6174       adhoc
  #           0x0033        9590       doodle
  find_mesh_wifi_device 0x168c "0x0034 0x003c 0x003e 0x0033"

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

if [[ -z "${12}" ]]; then
    routing_algo="batman-adv"
else
    routing_algo=${12}
fi

case "$1" in

mesh)

echo "sudo mesh $1 $2 $3 $4 $5 $6 $7 $8 $9 ${10} ${11}"
      if [[ -z "$1" || -z "$2" || -z "$3" || -z "$4" || -z "$5" || -z "$6" ]]
        then
          echo "check arguments..."
        help
      fi

cat <<EOF >/var/run/wpa_supplicant-11s.conf
ctrl_interface=DIR=/var/run/wpa_supplicant
# use 'ap_scan=2' on all devices connected to the network
# this is unnecessary if you only want the network to be created when no other networks..
ap_scan=1
country=$9
p2p_disabled=1
mesh_max_inactivity=50
network={
    ssid="$6"
    bssid=$4
    mode=5
    frequency=$7
    psk="$5"
    key_mgmt=SAE
    ieee80211w=2
    mesh_fwding=0
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

      if [ "$routing_algo" == "batman-adv" ]; then
        pkill alfred 2>/dev/null
        pkill batadv-vis 2>/dev/null
        rm -f /var/run/alfred.sock

        #Check if batman_adv is built-in module
        modname=$(ls /sys/module | grep batman_adv)
        if [[ -z $modname ]]; then
          modprobe batman-adv
        fi
      elif [ "$routing_algo" == "olsr" ]; then
         pkill olsrd 2>/dev/null
      
      pkill hostapd 2>/dev/null
      pkill transmit 2>/dev/null

      #Check if batman_adv is built-in module
      modname=$(ls /sys/module | grep batman_adv)
      if [[ -z $modname ]]; then
        modprobe batman-adv
      fi

      echo "$wifidev down.."
      iw dev "$wifidev" del
      iw phy "$phyname" interface add "$wifidev" type mp
  
      echo "Longer range tweak.."
      iw phy "$phyname" set distance 1000

      echo "$wifidev create 11s.."
      ifconfig "$wifidev" mtu 1560

      echo "$wifidev up.."
      ip link set "$wifidev" up

      if [ "$routing_algo" == "batman-adv" ]; then
        batctl if add "$wifidev"
        echo "bat0 up.."
        ifconfig bat0 up
        echo "bat0 ip address.."
        ifconfig bat0 "$2" netmask "$3"
        echo "bat0 mtu size"
        ifconfig bat0 mtu 1460
        echo
        ifconfig bat0

        sleep 3

        # for visualisation
        (alfred -i bat0 -m)&
        echo "started alfred"
        (batadv-vis -i bat0 -s)&
        echo "started batadv-vis"
     elif [ "$routing_algo" == "olsr" ]; then
        ifconfig "$wifidev" $2 netmask "$3"
        # Enable debug level as necessary
        (olsrd -i "$wifidev" -d 0)&
     fi

      # Radio parameters
      iw dev "$wifidev" set txpower limit "$8"00

      # FIXME: Like the comment above - we need to figure out how to handle
      # multiple Wi-Fi interfaces better. For some reason the background setting
      # ensures wpa_supplicant doesn't start when this script is run as a process.
      # This is likely due to the interface not being up in time, and will
      # require some fiddling with the systemd startup order.
      if [[ -z "${10}" ]]; then
        wpa_supplicant -i "$wifidev" -c /var/run/wpa_supplicant-11s.conf -D nl80211 -C /var/run/wpa_supplicant/ -B -f /tmp/wpa_supplicant_11s.log
      else
        wpa_supplicant -i "$wifidev" -c /var/run/wpa_supplicant-11s.conf -D nl80211 -C /var/run/wpa_supplicant/ -f /tmp/wpa_supplicant_11s.log
      fi

      ifname_ap="$wifidev-1"
      pcie_radio_mac="$(ip -brief link | grep "$wifidev" | awk '{print $3; exit}')"
      iw dev "$wifidev" interface add "$ifname_ap" type managed addr "00:01:$(echo "$pcie_radio_mac" | cut -b 7-17)"


      # Set frequency band and channel from given frequency
      calculate_wifi_channel "$7"


cat <<EOF >/var/run/hostapd.conf
country_code=FI
interface=$ifname_ap
ssid=tii_dri_drone
hw_mode=$retval_band
channel=$retval_channel
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=thisisaverylongpassword
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
ctrl_interface=/var/run/hostapd
# beacon interval needs to match mesh-point
# beacon_int=200 # Does this work? Doesn't seem to have any effect?

#This is an empty information element. dd indicates hex. 01 is the length of the data and 00 the actual data:
vendor_elements=dd0100

#This information element has one fixed Location message:
#vendor_elements=dd1EFA0BBC0D00102038000058D6DF1D9055A308820DC10ACF072803D20F0100
EOF

      /opt/ros/galactic/share/bin/hostapd -B /var/run/hostapd.conf
      sleep 2
      /opt/ros/galactic/share/bin/transmit b p &
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
      if [ "$routing_algo" == "batman-adv" ]; then
        pkill alfred 2>/dev/null
        pkill batadv-vis 2>/dev/null
        rm -f /var/run/alfred.sock 2>/dev/null
      elif [ "$routing_algo" == "olsr" ]; then
        pkill olsrd 2>/dev/null
      fi
      ;;
*)
      help
      ;;
esac

