#!/bin/bash

#######################################
# Init
# Globals:
#  result
#  device_list
# Arguments:
#  $1 = ipaddress
#  $2 = encryption
#  $3 = frequency
#  $4 = country
#  $5 = Batman orig_interval
#  $6 = Batman routing algo
#######################################
_init() {
  _deinit
  echo "$0, init called" | print_log

  # detect_wifi
  # multiple wifi options --> can be detected as follows:
  # manufacturer 0x168c = Qualcomm
  # devices = 0x0034 0x003c 9462/988x  11s
  #           0x003e        6174       adhoc
  #           0x0033        Doodle
  find_wifi_device "pci" 0x168c "0x0034 0x003c 0x003e 0x0033"
  phyname=${device_list[0]}  # only first pci device is used here TODO
  wifidev="wlp1s0"
  if ! iw phy $phyname interface add $wifidev type mp; then
    result=$FAIL
    return
  fi

  # setup iperf3 server
  iperf3 -s -D -p "$iperf3_port" --forceflush

  ifconfig "$wifidev" mtu 1560
  ip link set "$wifidev" up

  batctl if add "$wifidev"
  set_batman_routing_algo "$6"
  batctl bat0 interface destroy  # needed for possible algo selection
  batctl if add "$wifidev"
  set_batman_orig_interval "$5"
  ifconfig bat0 up

  # Create static mac addr for Batman if
  mac_eth0="$(ip -brief link | grep "$wifidev" | awk '{print $3; exit}')"
  mac_bat0="00:00:$(echo "$mac_eth0" | cut -b 7-17)"
  ifconfig bat0 hw ether "$mac_bat0"
  ifconfig bat0 "$1" netmask 255.255.255.0
  ifconfig bat0 mtu 1460

  conf_filename="./tmp/wpa_supplicant_11s_$3_$2.conf"
  log_filename="./logs/wpa_supplicant_11s_$3_$2.log"

  # Create wpa_supplicant.conf here
  create_wpa_supplicant_config "$conf_filename" "$3" "$2" "$4"

  # start wpa_supplicant
  wpa_supplicant -Dnl80211 -B -i"$wifidev" -C /var/run/wpa_supplicant/ -c "$conf_filename" -f "$log_filename"
  sleep 5
  iw dev "$wifidev" set mesh_param mesh_fwding 0
  iw dev "$wifidev" set mesh_param mesh_ttl 1

  sleep 5
  wifi_type=$(iw dev "$wifidev" info | grep type)
  echo "$wifi_type mode" | print_log
  case $wifi_type in
    *"managed"*)
    result=$FAIL
    ;;
    *"mesh"*)
    result=$PASS
    ;;
    *)
    result=$FAIL
    return
    ;;
  esac
}

#######################################
# Init VLAN
# Globals:
#  result
#  device_list
# Arguments:
#  $1 = ipaddress
#  $2 = encryption
#  $3 = frequency
#  $4 = country
#  $5 = Batman orig_interval
#  $6 = Batman routing algo
#######################################
_init_vlan() {
  _deinit
  echo "$0, init called" | print_log
  # detect_wifi
  # multiple wifi options --> can be detected as follows:
  # manufacturer 0x168c = Qualcomm
  # devices = 0x0034 0x003c 9462/988x  11s
  #           0x003e        6174       adhoc
  #           0x0033        Doodle
  find_wifi_device "pci" 0x168c "0x0034 0x003c 0x003e 0x0033"
  phyname=${device_list[0]}  # only first pci device is used here
  wifidev="wlp1s0"
  if ! iw phy $phyname interface add $wifidev type mp; then
    result=$FAIL
    return
  fi

  # setup iperf3 server
  iperf3 -s -D -p "$iperf3_port" --forceflush

  ifconfig "$wifidev" mtu 1560
  ip link set "$wifidev" up

  batctl if add "$wifidev"
  set_batman_routing_algo "$6"
  batctl bat0 interface destroy  # needed for possible algo selection
  batctl if add "$wifidev"
  set_batman_orig_interval "$5"
  ifconfig bat0 up
  ifconfig bat0 mtu 1460

  # Default forward policy config
  iptables -P FORWARD ACCEPT

  conf_filename="./tmp/wpa_supplicant_11s_$3_$2.conf"
  log_filename="./logs/wpa_supplicant_11s_$3_$2.log"

  # Create wpa_supplicant.conf here
  create_wpa_supplicant_config "$conf_filename" "$3" "$2" "$4"

  # start wpa_supplicant
  wpa_supplicant -Dnl80211 -B -i"$wifidev" -C /var/run/wpa_supplicant/ -c "$conf_filename" -f "$log_filename"
  sleep 5
  iw dev "$wifidev" set mesh_param mesh_fwding 0
  iw dev "$wifidev" set mesh_param mesh_ttl 1

  # Create static mac addr for Batman if
  mac_eth0="$(ip -brief link | grep "$wifidev" | awk '{print $3; exit}')"
  mac_bat0="00:00:$(echo "$mac_eth0" | cut -b 7-17)"
  ifconfig bat0 hw ether "$mac_bat0"

  # Create bridge br-lan
  brctl addbr br-lan

  # Batman Mesh virtual network chain
  vbat="$(echo "$1" | rev | cut -d "." -f1 | rev)"
  ip link add link bat0 name bat0."$vbat" type vlan id "$vbat"
  ip link add link bat0 name bat0.$(("$vbat"+1)) type vlan id $(("$vbat"+1))

  # Virtual interface up
  ifconfig bat0."$vbat" up
  ifconfig bat0.$(("$vbat"+1)) up
  brctl addif br-lan bat0."$vbat"
  brctl addif br-lan bat0.$(("$vbat"+1))
  ifconfig br-lan "$1" netmask 255.255.255.0
  ifconfig br-lan up
  echo
  ifconfig br-lan

  # Disable batman bridge loop avoidance
  batctl meshif bat0 bridge_loop_avoidance 0
  echo "Batman chain bat0.$vbat bat0.$(("$vbat"+1)) done."

  sleep 5
  wifi_type=$(iw dev "$wifidev" info | grep type)
  echo "$wifi_type mode" | print_log
  case $wifi_type in
    *"managed"*)
    result=$FAIL
    ;;
    *"mesh"*)
    result=$PASS
    ;;
    *)
    result=$FAIL
    return
    ;;
  esac
}