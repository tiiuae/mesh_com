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
        mode=5
        ieee80211w=2
        frequency=$freq
        freq_list=$freq
        scan_freq=$freq
        key_mgmt=SAE
        psk="$psk"
        mesh_fwding=0
}
EOF
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

      #SLAAC immediately after basic setup
      slaac "$slaac"

      # Radio parameters
      echo "set radio parameters"
      # /usr/local/bin/cli_app set txpwr fixed 23
      /usr/local/bin/cli_app set gi short
      /usr/local/bin/cli_app set support_ch_width 1
      /usr/local/bin/cli_app set mesh_rssi_threshold -105

      # Batman parameters
      if [ "$routing_algo" == "batman-adv" ]; then
        batctl "$wifidev" hop_penalty 30
      fi

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
    mode=5
    frequency=$freq
    freq_list=$freq
    scan_freq=$freq
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

      echo "$wifidev up.."
      ip link set "$wifidev" up

      #SLAAC immediately after basic setup
      slaac "$slaac"

      # Radio parameters
      iw dev "$wifidev" set txpower limit "$txpwr"00

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
bridge=br-lan
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
EOF

      #SLAAC immediately after basic setup
      slaac "$slaac"

      # Start AP
      /usr/sbin/hostapd /var/run/hostapd-"$INDEX".conf -f /tmp/hostapd_"$INDEX".log
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
interface=$ifname_ap
hw_mode=$retval_band
channel=$retval_channel

ieee80211h=1
ieee80211d=1
country_code=$cc

bridge=br-lan
ssid=$ssid
auth_algs=1
wpa=2
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
wpa_passphrase=$psk

wmm_enabled=1
beacon_int=1000

### IEEE 802.11n
ieee80211n=1
#ht_capab=[HT40+][LDPC][SHORT-GI-20][SHORT-GI-40][TX-STBC][RX-STBC1][DSSS_CCK-40]

### IEEE 802.11ac
#ieee80211ac=1
#vht_capab=[MAX-MPDU-11454][RXLDPC][SHORT-GI-80][TX-STBC-2BY1][RX-STBC-1]
EOF

      #SLAAC immediately after basic setup
      slaac "$slaac"

      # Start AP
      /usr/sbin/hostapd /var/run/hostapd-"$INDEX".conf  -f /tmp/hostapd_"$INDEX".log
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
  #  id0_KEY=1234567890
  #  id0_ESSID=gold
  #  id0_FREQ=5805
  #  id0_TXPOWER=30
  #  id0_COUNTRY=AE
  #  id0_MESH_VIF=wlp1s0
  #  id0_PHY=phy0
  #  id0_FREQ_MCC=2412
  #  id0_PRIORITY=long_range
  #  ROLE=drone

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

  _mesh_vif="${INDEX}_MESH_VIF"
  wifidev="${!_mesh_vif}"

  find_phy_interface "$wifidev"
  phyname=$retval_phy

  _priority="${INDEX}_PRIORITY"
  priority="${!_priority}"

  _mptcp="${INDEX}_MPTCP"
  mptcp="${!_mptcp}"

  _slaac="${INDEX}_SLAAC"
  slaac="${!_slaac}"
  echo "SLAAC ifaces: '$slaac'"

  # shellcheck disable=SC2153
  # shellcheck disable=SC2034
  # this is for the future use
  role=${ROLE}

  echo "Used: $wifidev $phyname"

  if [[ -z "$algo" ]]; then
      if [[ -z "$batman_iface" ]]; then
        routing_algo=""
      else
        routing_algo="batman-adv"  
      fi
  else
      routing_algo=$algo
  fi

  # todo this needs to be moved
  if [ "$mptcp" == "enable" ]; then
    echo "MPTCP enabled"
    if [ $(grep -ic "INTERFACE_${INDEX}" /var/run/mptcp.conf) -eq 1 ]; then
        source /var/run/mptcp.conf
        sed -i "/INTERFACE_${INDEX}/d" /var/run/mptcp.conf
	sed -i "/SLAAC_${INDEX}/d" /var/run/mptcp.conf
    fi
    echo "index="$INDEX
    echo "INTERFACE_${INDEX}=${wifidev}" >> /var/run/mptcp.conf
    echo "SLAAC_${INDEX}=${slaac}" >> /var/run/mptcp.conf
  fi
  mode_execute "$mode"

}

main "$@"
