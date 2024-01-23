#!/bin/bash
source ./qos_olsrd_conf.sh
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

mesh_kill() {
  # $1 is process string to kill

  # shellcheck disable=SC2009
  # pgrep is not available
  pid=$(ps aux | grep "$1" | awk '{print $1}')

  # check pid is numeric value
  if [ "$pid" -eq "$pid" ] 2>/dev/null; then
    echo "killing $pid"
    kill -9 "$pid"
  fi
}

add_network_intf_to_bridge() {
  _bridge_name=$1
  _interfaces=$2

  # detect devices where managed switch is available
  _lan1=0
  if ip link show lan1 &> /dev/null; then
    _lan1=1
  fi
  # Loop through the interface names and add them to the bridge if available
  for _interface in $_interfaces; do
    # Check if the interface exists
    if ip link show dev "$_interface" > /dev/null 2>&1; then
      echo "Adding $_interface to $_bridge_name"
      brctl addif "$_bridge_name" "$_interface" 2>/dev/null
    else
      echo "Interface $_interface not found. Skipping."
    fi
  done

  if [ "$_lan1" -eq 1 ]; then
    # Add lan1 to bridge
    brctl delif "$_bridge_name" eth0 2>/dev/null
  fi
}

fix_iface_mac_addresses() {
      # handles the case when the mac address of the batman interface the same as the eth0.
      # For batman interface, the mac address is set to 00:0Y:XX:XX:XX:XX by utilizing eth0 mac address.
      # from 00:0Y:XX:XX:XX:XX Y is the batman interface index

      # batman interface is bat0, take the last char for the numbering
      batman_iface_index="$(echo "$batman_iface" | cut -b 4)"

      #Create static mac addr for Batman if
      eth0_mac="$(ip -brief link | grep eth0 | awk '{print $3; exit}')"
      batif_mac="00:0${batman_iface_index}:$(echo "$eth0_mac" | cut -b 7-17)"
      ifconfig "$batman_iface" down
      ifconfig "$batman_iface" hw ether "$batif_mac"
      ifconfig "$batman_iface" up
      if [[ -n $bridge_name ]]; then
        ifconfig "$bridge_name" down
        ifconfig "$bridge_name" hw ether "$eth0_mac"
        ifconfig "$bridge_name" up
      fi
}

calculate_network_address() {
  # with ip 192.168.1.2 and mask 255.255.255.0 --> network=192.168.1.0
  _IP_ADDRESS=$1
  _NETMASK=$2

  # Split the IP and MASK into octets
  IFS=. read -r -a ip_octets <<< "$_IP_ADDRESS"
  IFS=. read -r -a mask_octets <<< "$_NETMASK"

  # Calculate the network address
  network=""
  for ((i=0; i<4; i++)); do
    network_octet=$(( 10#${ip_octets[i]} & 10#${mask_octets[i]} ))
    network="${network}${network_octet}"
    if [ "$i" -lt 3 ]; then
      network="${network}."
    fi
  done
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

find_phy_interface() {
  # Argument:
  # $1 = Wi-Fi interface name

  # Return value: retval_phy as global

  echo "Find PHY interface for Wi-Fi interface: $1"
  echo
  phy_names=$(ls /sys/class/ieee80211/)

  for phy in $phy_names; do
    if [ -d "/sys/class/ieee80211/$phy/device/net/$1" ]; then
      retval_phy=$phy
      break
    else
      retval_phy=""
    fi
  done
}

slaac() {
  _slaac_interfaces=$1
  echo "SLAAC for '$_slaac_interfaces'"
  for _slaac_interface in $_slaac_interfaces; do
    echo "  SLAAC for '$_slaac_interface'"
    /opt/slaac.sh $_slaac_interface
    echo "  SLAAC for '$_slaac_interface' DONE"
  done
  echo "SLAAC for '$_slaac_interfaces' DONE"
}

mode_execute() {
  # parameters:
  # $1 = mode
  case "$mode" in
    "halow")
      MAX_RETRIES=5
      RETRY_COUNT=0
      cat <<EOF >/var/run/wpa_supplicant-11s_"$INDEX".conf
ctrl_interface=DIR=/var/run/wpa_supplicant_"$INDEX"
ap_scan=1
country=$cc
mesh_max_inactivity=50
p2p_disabled=1
network={
        ssid="$ssid"
        bssid=$mac
        mode=5
        ieee80211w=2
        frequency=$freq
        key_mgmt=SAE
        psk="$psk"
        mesh_fwding=0
}
EOF

      if [ "$routing_algo" == "batman-adv" ]; then
        echo "batman-adv"
      elif [ "$routing_algo" == "olsr" ]; then
        mesh_kill "[o]lsrd -i $wifidev -d 0"
      fi

      modprobe mac80211

    command=
      # read halow cfg written by init_halow.sh if found
      if [ -f /var/run/halow_init.conf ]; then
        source /var/run/halow_init.conf
        if [ $SPI_BUS_NO ]; then
          command="insmod /lib/modules/$KERNEL_VERSION/kernel/drivers/staging/nrc/nrc.ko fw_name=nrc7292_cspi.bin bd_name=nrc7292_bd.dat hifspeed=${DRV_SPI_SPEED} power_save=${DRV_POWER_SAVE} sw_enc=${DRV_SW_ENC} spi_gpio_irq=${DRV_GPIO_IRQ} spi_bus_num=${SPI_BUS_NO}"
          if [ $DRV_POLLING_INTERVAL ]; then
            command="${command}  spi_polling_interval=${DRV_POLLING_INTERVAL}"
          fi
        else
          echo "ERROR: HaLoW radio not found."
          logger "ERROR: HaLoW radio not found."
          return 1
        fi
      # otherwise use the old detection code.
      else
        # Radio spi bus number
        BUSNO=$(for i in /sys/class/spi_master/*/; do
        if [ -e "$i/device" ] && [ "$(basename "$(readlink "$i/device")")" == "spi-ft232h.0" ]; then
          echo "${i//[^0-9]/}"
        fi
        done)

        # /usr/local/bin/init_halow.sh will take care of the firmware copy to /lib/firmware
        if [ "$BUSNO" -gt 0 ]; then
            command="insmod /lib/modules/$KERNEL_VERSION/kernel/drivers/staging/nrc/nrc.ko fw_name=nrc7292_cspi.bin bd_name=nrc7292_bd.dat spi_bus_num=$BUSNO spi_polling_interval=3 hifspeed=20000000 spi_gpio_irq=-1 power_save=0 sw_enc=0"
        else
            command="insmod /lib/modules/$KERNEL_VERSION/kernel/drivers/staging/nrc/nrc.ko fw_name=nrc7292_cspi.bin bd_name=nrc7292_bd.dat hifspeed=20000000 power_save=0 sw_enc=1 spi_bus_num=0 spi_gpio_irq=5"
        fi
      fi

      # load the kernel driver.

      # remove nrc module before init
      rmmod nrc.ko

      # set initial country code
      logger "country code set($wifidev): $cc"
      iw reg set "$cc"

      while [ $RETRY_COUNT -lt $MAX_RETRIES ]; do
        logger "nrc module load retry count: $RETRY_COUNT"
        $command
        sleep 2
        # Bring up halow interface
        echo "$wifidev up.."
        ifconfig "$wifidev" up
        if [ $? -eq 255 ]; then
          echo "Failed to init HaLow. (Attempt $((RETRY_COUNT + 1)))"
          rmmod nrc.ko
          RETRY_COUNT=$((RETRY_COUNT + 1))
        else
          break
        fi
      done

      echo "$wifidev create 11s.."
      ifconfig "$wifidev" mtu 1460

      sleep 1

      if [ "$routing_algo" == "batman-adv" ]; then
        batctl meshif $batman_iface if add "$wifidev"
        echo "$batman_iface up.."
        ifconfig "$batman_iface" up
        echo "$batman_iface ip address.."
        ifconfig "$batman_iface" "$ipaddr" netmask "$nmask"
        echo "$batman_iface mtu size"
        ifconfig "$batman_iface" mtu 1460
        echo
        ifconfig "$batman_iface"
        if [[ -n $bridge_name ]]; then
            add_network_intf_to_bridge "$bridge_name" "$bridge_interfaces"
            ifconfig "$bridge_name" "$bridge_ip" netmask "$nmask"
            ifconfig "$bridge_name" up
            echo
            ifconfig "$bridge_name"
            # Add forwarding rules from AP to "$batman_iface" interface
            iptables -P FORWARD ACCEPT
            route del -net "$network" gw 0.0.0.0 netmask "$nmask" dev "$bridge_name"
            route add -net "$network" gw "$bridge_ip" netmask "$nmask" dev "$bridge_name"
            iptables -A FORWARD --in-interface "$_mesh_vif" -j ACCEPT
            iptables --table nat -A POSTROUTING --out-interface "$bridge_ip" -j MASQUERADE
            sleep 5
        fi
        fix_iface_mac_addresses

      elif [ "$routing_algo" == "olsr" ]; then
        ifconfig "$wifidev" "$ipaddr" netmask "$nmask"
        # Enable debug level as necessary
        (olsrd -i "$wifidev" -d 0)&
      fi

      #SLAAC immediately after basic setup
      slaac "$slaac"

      if [ "$routing_algo" == "batman-adv" ]; then
        sleep 3
        # for visualisation
        if ps aux | grep -q "[b]atadv-vis -i $batman_iface -s"; then
          echo "batadv-vis is already running."
        else
          (batadv-vis -i "$batman_iface" -s) &
          echo "batadv-vis started."
        fi
      fi

      # Radio parameters
      echo "set radio parameters"
      # /usr/local/bin/cli_app set txpwr fixed 23

      # Batman parameters
      batctl "$wifidev" hop_penalty 30

      echo "Longer range tweak.."
      #if [ "$priority" == "long_range" ]; then
        # /usr/local/bin/cli_app ...
      #elif [ "$priority" == "low_latency" ]; then
        # /usr/local/bin/cli_app ...
      #else
        # /usr/local/bin/cli_app ...
      #fi

      wpa_supplicant -i "$wifidev" -c /var/run/wpa_supplicant-11s_"$INDEX".conf -D nl80211 -C /var/run/wpa_supplicant_"$INDEX"/ -f /tmp/wpa_supplicant_11s_"$INDEX".log
      ;;
  "mesh")
      if [ "$priority" ==  "high_throughput" ]; then
    	ht=0
      else
	ht=1
      fi
      cat <<EOF >/var/run/wpa_supplicant-11s_"$INDEX".conf
ctrl_interface=DIR=/var/run/wpa_supplicant_$INDEX
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
    # 11b rates dropped (for better performance)
    mesh_basic_rates=60 90 120 180 240 360 480 540
    disable_vht=$ht
    disable_ht40=$ht
}
EOF

      if [ "$routing_algo" == "batman-adv" ]; then
        echo "batman-adv"
        if [[ -d "/sys/module/batman_adv" ]]; then
          modprobe batman-adv
        fi
      elif [ "$routing_algo" == "olsr" ]; then
        #mesh_kill "[o]lsrd -i $wifidev -d 0"
        stop_olsrd "$id0_MESH_VIF"
      fi

      echo "$wifidev down.."
      iw dev "$wifidev" del
      iw phy "$phyname" interface add "$wifidev" type mp
        
      if [ "$priority" == "long_range" ]; then
        echo "Long range config"
        mount -t debugfs none /sys/kernel/debug
        iw phy "$phyname" set distance 8000
        echo 10 > /sys/kernel/debug/ieee80211/"$phyname"/ath9k/chanbw
        echo 7 > /sys/kernel/debug/ieee80211/"$phyname"/ath9k/tii_mask
        echo 4289724416 > /sys/kernel/debug/ieee80211/"$phyname"/rc/tii_rc
      elif [ "$priority" == "high_throughput" ]; then
        echo "High throughput config"
        iw phy "$phyname" set distance 0
      fi

      echo "$wifidev create 11s.."
      ifconfig "$wifidev" mtu 1560

      echo "$wifidev up.."
      ip link set "$wifidev" up

      if [ "$routing_algo" == "batman-adv" ]; then
        batctl meshif $batman_iface if add "$wifidev"
        echo "$batman_iface up.."
        ifconfig "$batman_iface" up
        echo "$batman_iface ip address.."
        ifconfig "$batman_iface" "$ipaddr" netmask "$nmask"
        echo "$batman_iface mtu size"
        ifconfig "$batman_iface" mtu 1460
        echo
        ifconfig "$batman_iface"
        if [[ -n $bridge_name ]]; then
            add_network_intf_to_bridge "$bridge_name" "$bridge_interfaces"
            ifconfig "$bridge_name" "$bridge_ip" netmask "$nmask"
            ifconfig "$bridge_name" up
            echo
            ifconfig "$bridge_name"
            # Add forwarding rules from AP to "$batman_iface" interface
            iptables -P FORWARD ACCEPT
            route del -net "$network" gw 0.0.0.0 netmask "$nmask" dev "$bridge_name"
            route add -net "$network" gw "$bridge_ip" netmask "$nmask" dev "$bridge_name"
            iptables -A FORWARD --in-interface "$_mesh_vif" -j ACCEPT
            iptables --table nat -A POSTROUTING --out-interface "$bridge_ip" -j MASQUERADE
            sleep 5
        fi
        fix_iface_mac_addresses
      elif [ "$routing_algo" == "olsr" ]; then
        ifconfig "$wifidev" "$ipaddr" netmask "$nmask"
        # Enable debug level as necessary
        #(olsrd -i "$wifidev" -d 0)&
        start_olsrd "$id0_MESH_VIF"
      fi

      #SLAAC immediately after basic setup
      slaac "$slaac"

      # Radio parameters
      iw dev "$wifidev" set txpower limit "$txpwr"00

      if [ "$routing_algo" == "batman-adv" ]; then
        sleep 3
        # for visualisation
        if ps aux | grep -q "[b]atadv-vis -i $batman_iface -s"; then
          echo "batadv-vis is already running."
        else
          (batadv-vis -i "$batman_iface" -s) &
          echo "batadv-vis started."
        fi
      fi
      wpa_supplicant -dd -i "$wifidev" -c /var/run/wpa_supplicant-11s_"$INDEX".conf -D nl80211 -C /var/run/wpa_supplicant_"$INDEX"/ -f /tmp/wpa_supplicant_11s_"$INDEX".log
      ;;
  "ap+mesh_mcc")
      wait_for_intf "$bridge_name"
      ifname_ap="$(ifconfig -a | grep -m 1 "wlan*" | awk -F':' '{ print $1 }')"
      ap_if_mac="$(ip -brief link | grep "$ifname_ap" | awk '{print $3; exit}')"
      ssid="comms_sleeve#$(echo "$ap_if_mac" | cut -b 13-14,16-17)"
      # Set frequency band and channel from given frequency
      calculate_wifi_channel "$freq_mcc"
      ifconfig "$ifname_ap" up

      # AP hostapd config
      cat <<EOF >/var/run/hostapd-"$INDEX".conf
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
            brctl addif "$bridge_name" "$wifidev" "$ifname_ap"
            iptables -A FORWARD --in-interface "$wifidev" -j ACCEPT
            mesh_kill "[o]lsrd -i $bridge_name -d 0"
            (olsrd -i "$bridge_name" -d 0)&
      else
            ##batman-adv###
            ifconfig "$bridge_name" down
            add_network_intf_to_bridge "$bridge_name" "$ifname_ap"
            ifconfig "$bridge_name" down
            iptables -A FORWARD --in-interface "$batman_iface" -j ACCEPT
      fi
      ifconfig "$bridge_name" "$bridge_ip" netmask "$nmask"
      ifconfig "$bridge_name" up
      echo
      ifconfig "$bridge_name"
      # Create static mac addr for Batman if and br-lan
      fix_iface_mac_addresses
      # Add forwading rules from AP to "$batman_iface" interface
      iptables -P FORWARD ACCEPT
      route del -net "$network" gw 0.0.0.0 netmask "$nmask" dev "$bridge_name"
      route add -net "$network" gw "$bridge_ip" netmask "$nmask" dev "$bridge_name"
      iptables --table nat -A POSTROUTING --out-interface "$ifname_ap" -j MASQUERADE

      #SLAAC immediately after basic setup
      slaac "$slaac"

      # Start AP
      /usr/sbin/hostapd -B /var/run/hostapd-"$INDEX".conf -f /tmp/hostapd_"$INDEX".log
      ;;
  "ap+mesh_scc")
      wait_for_intf "$bridge_name"
      # chanbw config
      mount -t debugfs none /sys/kernel/debug
      if [ -f "/sys/kernel/debug/ieee80211/phy0/ath9k/chanbw" ]; then
          echo 20 > /sys/kernel/debug/ieee80211/phy0/ath9k/chanbw
      fi

      # RPi activity led config
      echo "phy0tx" > /sys/class/leds/led0/trigger

      #Create static mac addr for Batman if and br-lan
      fix_iface_mac_addresses

      # Radio parameters
      iw dev "$wifidev" set txpower limit "$txpwr"00

      pcie_radio_mac="$(ip -brief link | grep "$wifidev" | awk '{print $3; exit}')"
      ssid="p2p#$(echo "$pcie_radio_mac" | cut -b 13-14,16-17)"

      ifname_ap="$wifidev-1"
      iw dev "$wifidev" interface add "$ifname_ap" type managed addr "00:01:$(echo "$pcie_radio_mac" | cut -b 7-17)"

      # Set frequency band and channel from given frequency
      calculate_wifi_channel "$freq"

      # AP hostapd config
      cat <<EOF >/var/run/hostapd-"$INDEX".conf
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
      ifconfig "$bridge_name" down
      add_network_intf_to_bridge "$bridge_name" "$ifname_ap"
      ifconfig "$bridge_name" "$ipaddr" netmask "$nmask"
      ifconfig "$bridge_name" up
      echo
      ifconfig "$bridge_name"
      iptables -P FORWARD ACCEPT
      ip addr flush dev "$batman_iface"

      #SLAAC immediately after basic setup
      slaac "$slaac"

      # Start AP
      /usr/sbin/hostapd -B /var/run/hostapd-"$INDEX".conf  -f /tmp/hostapd_"$INDEX".log
      ;;
  *)
      exit 1
      ;;
  esac
}

### MAIN ######################################################################
main () {
  # $1 = mode
  # $2 = index number

  source /opt/mesh-helper.sh

  echo "index=$2 mode=$1"
  INDEX=$2

  # sources mesh configuration
  source_configuration "0"
  source_configuration "${INDEX:2}"

  # linux kernel release
  KERNEL_VERSION=$(uname -r)

  # Modes: mesh, ap+mesh_scc, ap+mesh_mcc, halow
  # mesh 1.0 with NATS communication setup
  # Modes: mesh, ap+mesh_scc, ap+mesh_mcc, halow
  # example from mesh_default.conf:
  #  id0_MODE=mesh
  #  id0_IP=10.20.15.3
  #  id0_MASK=255.255.255.0
  #  id0_MAC=00:11:22:33:44:55
  #  id0_KEY=1234567890
  #  id0_ESSID=gold
  #  id0_FREQ=5805
  #  id0_TXPOWER=30
  #  id0_COUNTRY=AE
  #  id0_MESH_VIF=wlp1s0
  #  id0_PHY=phy0
  #  id0_BATMAN_IFACE=bat0
  #  id0_FREQ_MCC=2412
  #  id0_ROUTING=batman-adv
  #  id0_PRIORITY=long_range
  #  BRIDGE="br-mesh eth1 eth0 lan1"
  #  ROLE=drone
  find_ethernet_port
  # to get eth_port warning free
  eth_port=$eth_port

  # default mesh handling and power off radio
  if [ "$1" == "mesh" ]; then
    mode=$1
  else
    # shellcheck disable=SC2153
    _mode="${INDEX}_MODE"
    mode="${!_mode}"
  fi

  _ssid="${INDEX}_ESSID"
  ssid="${!_ssid}"

  _ipaddr="${INDEX}_IP"
  ipaddr="${!_ipaddr}"

  _nmask="${INDEX}_MASK"
  nmask="${!_nmask}"

  _mac="${INDEX}_MAC"
  mac="${!_mac}"

  _freq="${INDEX}_FREQ"
  freq="${!_freq}"

  _freq_mcc="${INDEX}_FREQ_MCC"
  freq_mcc="${!_freq_mcc}"

  _cc="${INDEX}_COUNTRY"
  cc="${!_cc}"

  _psk="${INDEX}_KEY"
  psk="${!_psk}"

  _txpwr="${INDEX}_TXPOWER"
  txpwr="${!_txpwr}"

  _algo="${INDEX}_ROUTING"
  algo="${!_algo}"

  _mesh_vif="${INDEX}_MESH_VIF"
  wifidev="${!_mesh_vif}"

  find_phy_interface "$wifidev"
  phyname=$retval_phy

  _priority="${INDEX}_PRIORITY"
  priority="${!_priority}"

  _batman_iface="${INDEX}_BATMAN_IFACE"
  batman_iface="${!_batman_iface}"

  _mptcp="${INDEX}_MPTCP"
  mptcp="${!_mptcp}"

  _bridge="${INDEX}_BRIDGE"
  bridge="${!_bridge}"

  _slaac="${INDEX}_SLAAC"
  slaac="${!_slaac}"
  echo "SLAAC ifaces: '$slaac'"

  # shellcheck disable=SC2153
  # shellcheck disable=SC2034
  # this is for the future use
  role=${ROLE}

  echo "Used: $wifidev $phyname"

  if [[ -z "$algo" ]]; then
      routing_algo="batman-adv"
  else
      routing_algo=$algo
  fi

  # e.g. BRIDGE=br-mesh eth0 eth1 lan1
  bridge_name=$(echo "$bridge" | cut -d' ' -f1)
  bridge_interfaces=$(echo "$bridge" | cut -d' ' -f2-)

  if brctl show "$bridge_name" &>/dev/null; then
    echo "Bridge $bridge_name already exists."
  else
    # Create the bridge if it doesn't exist
    brctl addbr "$bridge_name" 2>/dev/null
    echo "Bridge $bridge_name created."
  fi
  if [[ -n "$bridge_name" ]]; then
      generate_lan_bridge_ip
      # to get bridge_ip warning free
      bridge_ip=$bridge_ip
      calculate_network_address "$bridge_ip" "$nmask"
  fi
  if [ "$mptcp" == "enable" ]; then
    echo "MPTCP enabled"
    if ! [ -f /var/run/mptcp.conf ]; then
        echo "SUBFLOWS=-1" > /var/run/mptcp.conf
    fi
    if [ $(grep -ic "INTERFACE_${INDEX}" /var/run/mptcp.conf) -eq 1 ]; then
        source /var/run/mptcp.conf
        sed -i "/INTERFACE_${INDEX}/d" /var/run/mptcp.conf
    else
        source /var/run/mptcp.conf
        subflows=$((SUBFLOWS+1))
        sed_param=s/SUBFLOWS=.*/SUBFLOWS=${subflows}/
        sed -i "$sed_param" /var/run/mptcp.conf
    fi
    if [[ -n $bridge_name ]]; then
        if [ $(grep -ic "BRIDGE_IFACE" /var/run/mptcp.conf) -eq 1 ]; then
          sed -i "/BRIDGE_IFACE/d" /var/run/mptcp.conf
        fi
        echo "BRIDGE_IFACE=${bridge_name}" >> /var/run/mptcp.conf
        echo "INTERFACE_${INDEX}=${bridge_name}" >> /var/run/mptcp.conf
        source /var/run/mptcp.conf
    else
        echo "index="$INDEX
        echo "INTERFACE_${INDEX}=${batman_iface}" >> /var/run/mptcp.conf
    fi
  fi
  mode_execute "$mode"

}

main "$@"
