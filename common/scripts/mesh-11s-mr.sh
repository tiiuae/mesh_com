#!/bin/bash

function help
{
  echo
  cat << EOF
Usage: sudo ./mesh_11s-mr.sh <mode> <ip> <mask> <AP MAC> <key> <essid> <freq> <txpower> <country> [<interface>] [<phyname>]"
Parameters:
  <mode>
  <ip>
  <mask>
  <AP MAC>
  <key>
  <essid>
  <freq>
  <txpower>
  <country>
  <interface>  - optional
  <phyname>    - optional

example:
 For One Radio:
   sudo ./mesh-11s-mr.sh mesh 192.168.1.2 255.255.255.0 00:11:22:33:44:55 1234567890 mymesh2 5220 30 fi wlan1 phy1
 Automatic detection:
   sudo ./mesh-11s-mr.sh mesh 192.168.1.2 255.255.255.0 00:11:22:33:44:55 1234567890 mymesh2 5220 30 fi
 Create simple AP for debug purposes:
   sudo ./mesh-11s-mr.sh ap
 Turn all radios off:
   sudo ./mesh-11s-mr.sh off
EOF
  exit
}

find_mesh_wifi_device()
{
  # arguments:
  # $1 = bus (usb, pci, sdio..)
  # $2 = wifi device vendor
  # $3 = wifi device id list

  # return values: device_list
  #                format example = "phy0,wlp1s0 phy1,wlp2s0 phy2,wlp3s0"

  device_list=""

  echo "## Searching WIFI card: bus=$1 deviceVendor=$2 deviceID=$3"

  case "$1" in
    pci)
      # tested only with Qualcomm cards
      phynames=$(ls /sys/class/ieee80211/)
      for device in $3; do
        for phy in $phynames; do
          device_id="$(cat /sys/bus/pci/devices/*/ieee80211/"$phy"/device/device 2>/dev/null)"
          device_vendor="$(cat /sys/bus/pci/devices/*/ieee80211/"$phy"/device/vendor 2>/dev/null)"
          if [ "$device_id" = "$device" ] && [ "$device_vendor" = "$2" ]; then
            retval_phy=$phy
            retval_name=$(ls /sys/class/ieee80211/"$phy"/device/net/)
            # add "phy,name" pair to device_list (space as separator for pairs)
            device_list="$device_list$retval_phy,$retval_name "
          fi
        done
      done
      ;;
    usb)
      device_list=""
      ;;
    sdio)
      device_list=""
      ;;
  esac
}

create_meshpoint()
{
  # parameters:
  # 1      2    3      4        5     6       7      8         9         10          11
  # <mode> <ip> <mask> <AP MAC> <key> <essid> <freq> <txpower> <country> <interface> <phy>

  echo "create_meshpoint $1 $2 $3 $4 $5 $6 $7 $8 $9 ${10} ${11}"
  if [[ -z "$1" || -z "$2" || -z "$3" || -z "$4" || -z "$5" || -z "$6" ]]
  then
    echo "check arguments..."
    help
  fi

cat <<EOF >/var/run/wpa_supplicant-11s_"${10}".conf
ctrl_interface=DIR=/var/run/wpa_supplicant
# use 'ap_scan=2' on all devices connected to the network
# this is unnecessary if you only want the network to be created when no other networks..
ap_scan=1
country=$9
p2p_disabled=1
#autoscan=periodic:10
#bgscan="simple:30:-45:300"
network={
    ssid="$6"
    bssid=$4
    mode=5
    frequency=$7
    psk="$5"
    key_mgmt=SAE
    mesh_fwding=0
    ieee80211w=2
}
EOF

  echo "Killing wpa_supplicant for ${10}..."
  pkill -f "/var/run/wpa_supplicant-11s_${10}" 2>/dev/null
  rm -fr /var/run/wpa_supplicant/"${10}"


  killall alfred 2>/dev/null
  killall batadv-vis 2>/dev/null
  rm -f /var/run/alfred.sock

  modprobe batman-adv

  echo "$wifidev down.."
  iw dev "${10}" del
  iw phy "${11}" interface add "${10}" type mp

  echo "${10} create 11s.."
  ifconfig "${10}" mtu 1560

  echo "${10} up.."
  ip link set "${10}" up
  batctl if add "${10}"

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

  wpa_supplicant -i "${10}" -c /var/run/wpa_supplicant-11s_"${10}".conf -D nl80211 -C /var/run/wpa_supplicant/ -B
}

create_ap()
{
  # parameters:
  # 1      2    3      4        5     6       7      8         9         10          11
  # <mode> <ip> <mask> <AP MAC> <key> <essid> <freq> <txpower> <country> <interface> <phy>

  if [[ -z "$1" ]]
  then
    echo "check arguments..."
    help
  fi

  # find ap parameters from enclave
  if [ -z "$DRONE_DEVICE_ID" ]
  then
    echo "DRONE_DEVICE_ID not available"

cat <<EOF >/var/run/wpa_supplicant-ap_"${10}".conf
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

cat <<EOF >/var/run/wpa_supplicant-ap_"${10}".conf
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

  echo "Killing wpa_supplicant ${10}..."
  pkill -f "/var/run/wpa_supplicant-*_${10}" 2>/dev/null
  rm -fr /var/run/wpa_supplicant/"${10}"

  echo "create $wifidev"
  iw dev "${10}" del
  iw phy "${11}" interface add "${10}" type managed

  echo "$wifidev up.."
  ip link set "${10}" up
  batctl if add "${10}"

  echo "set ip address.."
  # set AP ipaddr
  ifconfig "${10}" 192.168.1.1 netmask 255.255.255.0

  # set bat0 ipaddr
  if [ -z "$DRONE_DEVICE_ID" ]
  then
    # DRONE_DEVICE_ID not available set default
    ifconfig bat0 192.168.1.1 netmask 255.255.255.0
  else
    declare -i ip
    ip=10#$(echo "$DRONE_DEVICE_ID" | tr -d -c 0-9)
    ifconfig bat0 192.168.1."$ip" netmask 255.255.255.0
  fi

  echo "bat0 up.."
  ifconfig bat0 up
  echo
  ifconfig bat0

  route del -net 192.168.1.0 netmask 255.255.255.0 dev bat0
  route add -net 192.168.1.0 netmask 255.255.255.0 dev bat0 metric 1

  # TODO DHCP server?
  # dhserver

  wpa_supplicant -B -i "${10}" -c /var/run/wpa_supplicant-ap_"${10}".conf -D nl80211 -C /var/run/wpa_supplicant/
}

off()
{
  # parameters:
  # 1      2    3      4        5     6       7      8         9         10          11
  # <mode> <ip> <mask> <AP MAC> <key> <essid> <freq> <txpower> <country> <interface> <phy>

  # kills all mesh script started wpa_supplicant processes (ap/mesh)
  pkill -f "/var/run/wpa_supplicant-" 2>/dev/null
  rm -fr /var/run/wpa_supplicant/"${10}"
  killall alfred 2>/dev/null
  killall batadv-vis 2>/dev/null
  rm -f /var/run/alfred.sock 2>/dev/null
}

init_device()
{
  # parameters:
  # 1      2    3      4        5     6       7      8         9         10          11
  # <mode> <ip> <mask> <AP MAC> <key> <essid> <freq> <txpower> <country> <interface> <phy>

  echo "- init_device $1"
  echo

  case "$1" in
    mesh)
      create_meshpoint "$@"
      ;;
    ap)
      create_ap "$@"
      ;;
    off)
      # stop all processes which were started
      off "$@"
      ;;
    *)
      help
      ;;
  esac
}

### MAIN ###
main ()
{
  # parameters
  # 1      2    3      4        5     6       7      8         9         10           11
  # <mode> <ip> <mask> <AP MAC> <key> <essid> <freq> <txpower> <country> <interface>  <phy>

  if [[ -z "$1" ]]; then
    help
    exit
  fi

  echo "Solving wifi device name and phy name.."
  echo
  #----------------

  count=0

  rfkill unblock all

  if [[ -z "${10}" ]]; then
    # multiple wifi options --> can be detected as follows:
    # manufacturer 0x168c = Qualcomm
    # devices = 0x0034 0x003c 9462/988x  11s
    #           0x003e        6174       adhoc
    list="0x0034 0x003c 0x003e"
    for dev in $list; do
      find_mesh_wifi_device "pci" 0x168c "$dev"

      if [ "$device_list" != "" ]; then
        for pair in $device_list; do
          phyname=$(echo "$pair" | cut -f1 -d ',')
          wifidev=$(echo "$pair" | cut -f2 -d ',')

          echo "  --------------------------------------------------------"

          count=$((count+1))
          echo "- #$count Found: $wifidev $phyname"

          if [ "$count" -gt 1 ]; then
            # ACS - Automatic Channel Selection, not yet working for wpa/ap
            # TODO Hardcoded frequency used
            init_device "$1" "$2" "$3" "$4" "$5" "$6" "2412" "$8" "$9" "$wifidev" "$phyname"
          else
            init_device "$1" "$2" "$3" "$4" "$5" "$6" "$7" "$8" "$9" "$wifidev" "$phyname"
          fi

          if [ "$1" = "ap" ]; then
            # only one AP is initialised
            exit 0
          fi
        done
      else
        echo "- Not Found: 0x168c $dev -  (not installed)"
      fi
      echo
    done
  else
    wifidev=${10}
    phyname=${11}
    init_device "$1" "$2" "$3" "$4" "$5" "$6" "$7" "$8" "$9" "${10}" "${11}"
  fi
}

main "$@"

