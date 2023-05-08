#!/bin/bash

is_interface() {
  # arguments:
  # $1 = interface name
	[[ -z "$1" ]] && return 1
	[[ -d "/sys/class/net/${1}" ]]
}

wait_for_intf() {
  # arguments:
  # $1 = interface name
	local TIMEOUT_MS=30000
	while [[ 0 -ne $TIMEOUT_MS ]]; do
		if is_interface "$1"; then
			return
		fi
		sleep 0.1
		TIMEOUT_MS=$((TIMEOUT_MS-100))
	done
}

calculate_wifi_channel() {
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

find_ethernet_port() {
  # for HW detection
  COMMS_PCB_VERSION_FILE="/opt/hardware/comms_pcb_version"
  if [ -f "$COMMS_PCB_VERSION_FILE" ]; then
    # shellcheck disable=SC1090
    source "$COMMS_PCB_VERSION_FILE"
    # has PCB version 0
    if (( $(echo "$COMMS_PCB_VERSION == 0" |bc -l) )); then
      eth_port="eth1"
    # PCB version is 0.5
    elif (( $(echo "$COMMS_PCB_VERSION == 0.5" |bc -l) )); then
      eth_port="eth0"
    # PCB version starts from 1
    elif (( $(echo "$COMMS_PCB_VERSION >= 1" |bc -l) )); then
      eth_port="lan1"
    else
      eth_port="eth1"
    fi
  fi
}

mode_execute() {
  # parameters:
  # $1 = mode
  case "$mode" in
  "mesh")

      cat <<EOF >/var/run/wpa_supplicant-11s_"$wifidev".conf
ctrl_interface=DIR=/var/run/wpa_supplicant
# use 'ap_scan=2' on all devices connected to the network
# this is unnecessary if you only want the network to be created when no other networks..
ap_scan=1
country=$cc
p2p_disabled=1
mesh_max_inactivity=50
network={
    ssid="$ssid"
    bssid=$mac
    mode=5
    frequency=$freq
    psk="$psk"
    key_mgmt=SAE
    ieee80211w=2
    beacon_int=1000
    mesh_fwding=0
}
EOF

      echo "Killing wpa_supplicant..."

      if [ "$routing_algo" == "batman-adv" ]; then
        killall alfred 2>/dev/null
        killall batadv-vis 2>/dev/null
        rm -f /var/run/alfred.sock

        #Check if batman_adv is built-in module
        if [[ -d "/sys/module/batman_adv" ]]; then
          modprobe batman-adv
        fi
      elif [ "$routing_algo" == "olsr" ]; then
         killall olsrd 2>/dev/null
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
        ifconfig bat0 "$ipaddr" netmask "$nmask"
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
        ifconfig "$wifidev" "$ipaddr" netmask "$nmask"
        # Enable debug level as necessary
        (olsrd -i "$wifidev" -d 0)&
      fi

      # Radio parameters
      iw dev "$wifidev" set txpower limit "$txpwr"00

      brctl addif br-lan bat0 $eth_port
      ifconfig br-lan "$br_lan_ip" netmask "255.255.255.0"
      ifconfig br-lan up
      echo
      ifconfig br-lan
      # Add forwarding rules from AP to bat0 interface
      iptables -P FORWARD ACCEPT
      route del -net 192.168.1.0 gw 0.0.0.0 netmask 255.255.255.0 dev br-lan
      route add -net 192.168.1.0 gw "$br_lan_ip" netmask 255.255.255.0 dev br-lan
      iptables -A FORWARD --in-interface bat0 -j ACCEPT
      iptables --table nat -A POSTROUTING --out-interface "$br_lan_ip" -j MASQUERADE

      wpa_supplicant -i "$wifidev" -c /var/run/wpa_supplicant-11s_"$wifidev".conf -D nl80211 -C /var/run/wpa_supplicant/ -f /tmp/wpa_supplicant_11s.log
      ;;
  "ap+mesh_mcc")
      wait_for_intf "br-lan"
      ifname_ap="$(ifconfig -a | grep -m 1 "wlan*" | awk -F':' '{ print $1 }')"
      ap_if_mac="$(ip -brief link | grep "$ifname_ap" | awk '{print $3; exit}')"
      ssid="comms_sleeve#$(echo "$ap_if_mac" | cut -b 13-14,16-17)"
      # Set frequency band and channel from given frequency
      calculate_wifi_channel "$freq_mcc"
      ifconfig "$ifname_ap" up

      # AP hostapd config
      cat <<EOF >/var/run/hostapd-"$ifname_ap".conf
country_code=$cc
interface=$ifname_ap
ssid=$ssid
hw_mode=$retval_band
channel=$retval_channel
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=$psk
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
beacon_int=1000
ctrl_interface=/var/run/hostapd
EOF

      # Bridge AP and Mesh
      if [ "$routing_algo" = "olsr" ]; then
            brctl addif br-lan "$wifidev" "$ifname_ap"
            iptables -A FORWARD --in-interface "$wifidev" -j ACCEPT
            killall olsrd 2>/dev/null
            (olsrd -i br-lan -d 0)&
      else
            ##batman-adv###
            ifconfig br-lan down
            brctl addif br-lan "$ifname_ap"
            ifconfig br-lan down
            iptables -A FORWARD --in-interface bat0 -j ACCEPT
      fi
      ifconfig br-lan "$br_lan_ip" netmask "255.255.255.0"
      ifconfig br-lan up
      echo
      ifconfig br-lan
      # Add forwading rules from AP to bat0 interface
      iptables -P FORWARD ACCEPT
      route del -net 192.168.1.0 gw 0.0.0.0 netmask 255.255.255.0 dev br-lan
      route add -net 192.168.1.0 gw "$br_lan_ip" netmask 255.255.255.0 dev br-lan
      iptables --table nat -A POSTROUTING --out-interface "$ifname_ap" -j MASQUERADE

      # Start AP
      /usr/sbin/hostapd -B /var/run/hostapd-"$ifname_ap".conf -f /tmp/hostapd.log
      ;;
  "ap+mesh_scc")
      wait_for_intf "br-lan"
      # chanbw config
      mount -t debugfs none /sys/kernel/debug
      if [ -f "/sys/kernel/debug/ieee80211/phy0/ath9k/chanbw" ]; then
          echo 20 > /sys/kernel/debug/ieee80211/phy0/ath9k/chanbw
      fi

      # RPi activity led config
      echo "phy0tx" > /sys/class/leds/led0/trigger

      #Create static mac addr for Batman if
      eth0_mac="$(ip -brief link | grep eth0 | awk '{print $3; exit}')"
      batif_mac="00:00:$(echo "$eth0_mac" | cut -b 7-17)"
      ifconfig bat0 hw ether "$batif_mac"

      # Radio parameters
      iw dev "$wifidev" set txpower limit "$txpwr"00

      pcie_radio_mac="$(ip -brief link | grep "$wifidev" | awk '{print $3; exit}')"
      ssid="p2p#$(echo "$pcie_radio_mac" | cut -b 13-14,16-17)"

      ifname_ap="$wifidev-1"
      iw dev "$wifidev" interface add "$ifname_ap" type managed addr "00:01:$(echo "$pcie_radio_mac" | cut -b 7-17)"

      # Set frequency band and channel from given frequency
      calculate_wifi_channel "$freq"

      # AP hostapd config
      cat <<EOF >/var/run/hostapd-"$ifname_ap".conf
ctrl_interface=/var/run/hostapd
interface=$ifname_ap
hw_mode=$retval_band
channel=$retval_channel

ieee80211h=1
ieee80211d=1
country_code=$cc

ssid=$ssid
auth_algs=1
wpa=2
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
wpa_passphrase=$psk

wmm_enabled=1
beacon_int=1000
ctrl_interface=/var/run/hostapd

### IEEE 802.11n
ieee80211n=1
#ht_capab=[HT40+][LDPC][SHORT-GI-20][SHORT-GI-40][TX-STBC][RX-STBC1][DSSS_CCK-40]

### IEEE 802.11ac
#ieee80211ac=1
#vht_capab=[MAX-MPDU-11454][RXLDPC][SHORT-GI-80][TX-STBC-2BY1][RX-STBC-1]
EOF
      ifconfig br-lan down
      brctl addif br-lan "$ifname_ap"
      ifconfig br-lan "$ipaddr" netmask "$nmask"
      ifconfig br-lan up
      echo
      ifconfig br-lan
      iptables -P FORWARD ACCEPT
      ip addr flush dev bat0

      # Start AP
      /usr/sbin/hostapd -B /var/run/hostapd-"$ifname_ap".conf -f /tmp/hostapd.log
      ;;
  off)
      # service off
      pkill -f "/var/run/wpa_supplicant-" 2>/dev/null
      rm -fr /var/run/wpa_supplicant/"$wifidev"
      killall hostapd 2>/dev/null
      if [ "$routing_algo" == "batman-adv" ]; then
        killall alfred 2>/dev/null
        killall batadv-vis 2>/dev/null
        rm -f /var/run/alfred.sock 2>/dev/null
      elif [ "$routing_algo" == "olsr" ]; then
        killall olsrd 2>/dev/null
      fi
      ;;
  *)
      exit 1
      ;;
  esac
}

### MAIN ######################################################################
main () {
  source /opt/mesh-helper.sh
  # sources mesh configuration
  source_configuration
  # set bridge ip, sets br_lan_ip
  generate_br_lan_ip

  # local find eth port
  find_ethernet_port
  echo $eth_port

  if [ "$1" == "mesh" ] || "$1" == "off"; then
    mode=$1
  else
    # shellcheck disable=SC2153
    mode="$MODE"
  fi
  ssid=$ESSID
  ipaddr=$IP
  nmask=$MASK
  mac=$MAC
  # shellcheck disable=SC2153
  freq=$FREQ
  # shellcheck disable=SC2153
  freq_mcc=$FREQ_MCC
  cc=$COUNTRY
  psk=$KEY
  txpwr=$TXPOWER
  algo=$ROUTING
  wifidev=$MESH_VIF
  phyname=$PHY

  echo "Used: $wifidev $phyname"

  if [[ -z "$algo" ]]; then
      routing_algo="batman-adv"
  else
      routing_algo=$algo
  fi

  brctl addbr br-lan 2>/dev/null

  mode_execute "$mode"
}

main "$@"