#! /bin/bash -x

COMMS_PCB_VERSION_FILE="/opt/hardware/comms_pcb_version"

calculate_wifi_channel()
{
  # arguments:
  # $1 = wifi frequency
  # return values: retval_band, retval_channel as global
  # Set 2.4/5GHz frequency band and channel

  if [ "$1" -ge 5160 ] && [ "$1" -le 5885 ]; then
    retval_band="a"
    retval_channel=$((("$1"-5000)/5))
  elif [ "$1" -ge 2412 ] && [ "$1" -le 2472 ]; then
    retval_band="g"
    retval_channel=$((("$1"-2407)/5))
  else
    echo "ERROR! frequency out of range!"
    exit 1
  fi
}

create_ap_config()
{
  cat > ap.conf <<- EOF
    network={
      ssid="WirelessLab"
      psk="ssrcdemo"
    }
  EOF
}
#configurate the dhcpd file 
create_dhcpd_config()
{
  SUBNET="$1"

  cat > /etc/dhcp/dhcpd.conf <<- EOF
    default-lease-time 600;
    max-lease-time 7200;
    ddns-update-style none;
    authoritative;

    subnet $SUBNET.0 netmask 255.255.255.0 {
            range $SUBNET.100 $SUBNET.199;
            option routers $SUBNET.1;
    }
  EOF
  cp /dev/null /var/lib/dhcp/dhcpd.leases
}
#create a confiuration file for OlSRD whereas we add Hna4 and Hna6
create_olsrd_config()
{
  wifidev="$1"
  SUBNET="$2"
  IPV6_PREFIX="$3"

  cat > /etc/olsrd/olsrd.conf <<- EOF
  
  LinkQualityFishEye   0

  Interface "$wifidev"

  {

  }

  IpVersion               4

  LinkQualityFishEye      0

  LinkQualityAlgorithm "etx_ffeth_nl80211"

  # This is only here to be able to generate a

  # configuration file with the script

  LoadPlugin "/usr/lib/olsrd_jsoninfo.so.1.1"

  {

    PlParam "port"          "9090"

   PlParam "accept"        "0.0.0.0"

  }

  # load arprefresh plugin

  # - UDP packets on port 698

  LoadPlugin "/usr/lib/olsrd_arprefresh.so.0.1"  

  {

  }

  Hna4
  {
          $SUBNET.0 255.255.255.0
  }

  Hna6
  {
          $IPV6_PREFIX:0 64
  }
  EOF
}
#configuration of the radvd.conf to use a random ipv6
create_radvd_config()
{
  IPV6_PREFIX="$1"

  cat > /etc/radvd.conf <<- EOF
    interface br-lan
    {
            AdvSendAdvert on;
            prefix $IPV6_PREFIX:0/64 {
            };
    };
  EOF
}

if [ -f "/opt/mesh.conf" ]; then
  source /opt/mesh.conf
  mode=$CONCURRENCY
  ch=$MCC_CHANNEL
  ipaddr=$IP
  nmask=$MASK
  cc=$COUNTRY
  psk=$KEY
  txpwr=$TXPOWER
  algo=$ROUTING
  mesh_if=$MESH_VIF
else
  mode="mesh"
fi

if [ $mode == "provisioning" ]; then
  killall hostapd
  killall wpa_supplicant
  ifconfig br-lan down
  brctl delbr br-lan
  ifname_ap="$(ifconfig -a | grep wlan* | awk -F':' '{ print $1 }')"
  ifname_mp="$(ifconfig -a | grep wlp1* | awk -F':' '{ print $1 }')"
  ifconfig $ifname_ap down
  ifconfig $ifname_mp down
  ifconfig $ifname_ap up
  ifconfig $ifname_mp up
  exit 1
fi

#start mesh service if mesh provisoning is done
hw_platform=$( grep Model /proc/cpuinfo | awk '{print $5}' )
if [ $hw_platform == "Compute" ]; then
  if [ -f "/opt/S9011sMesh" ]; then
    #start Mesh service
    echo "starting 11s mesh service"
    /opt/S9011sMesh start
    sleep 2
  fi
else
  if [ -f "/opt/S90mesh" ]; then
    echo "starting ibss mesh service"
    /opt/S90mesh start
    sleep 2
  fi
fi

###Deciding IP address to be assigned to br-lan from WiFi MAC
mesh_if_mac="$(ip -brief link | grep "$mesh_if" | awk '{print $3; exit}')"
ip_random="$(echo "$mesh_if_mac" | cut -b 16-17)"
br_lan_ip="192.168.1."$((16#$ip_random))

if [ "$mode" = "sta+mesh" ]; then
  create_ap_config
  #Connect to default GW AP
  iface=$(ifconfig -a | grep wlan* | awk -F':' '{ print $1 }')
  wpa_supplicant -Dnl80211 -i$iface -c ap.conf -B
  sleep 3
  udhcpc -i $iface
elif [ "$mode" = "ap+mesh_mcc" ]; then
  # Create bridge br-lan
  brctl addbr br-lan
  ifname_ap="$(ifconfig -a | grep -E '^wlan' -m 1 | awk -F':' '{ print $1 }')"
  ap_if_mac="$(ip -brief link | grep "$ifname_ap" | awk '{print $3; exit}')"
  ssid="comms_sleeve#$(echo "$ap_if_mac" | cut -b 13-14,16-17)"
  # Set frequency band and channel from given frequency
  calculate_wifi_channel "$ch"
  ifconfig $ifname_ap up
  #generate a random IPV6
  IPV6_PREFIX=$( echo fd`dd if=/dev/urandom bs=7 count=1 status=none | xxd -p` | sed 's/\(....\)/\1:/g' )
  ip addr add $SUBNET.1/24 dev br-lan && ip addr add $IPV6_PREFIX:1/64 dev br-lan

  create_olsrd_config "wlp1s0" "$SUBNET" "$IPV6_PREFIX"
  # FIXME: launch olsrd

  create_dhcpd_config "$SUBNET"
  dhcpd -f br-lan

  create_radvd_config "$IPV6_PREFIX"
  # FIXME: launch radvd

  # AP hostapd config
  cat > /var/run/hostapd.conf <<- EOF
    country_code=AE
    interface=$ifname_ap
    ssid=$ssid
    hw_mode=g
    channel=7
    macaddr_acl=0
    auth_algs=1
    ignore_broadcast_ssid=0
    wpa=2
    wpa_passphrase=ssrcdemo
    wpa_key_mgmt=WPA-PSK
    wpa_pairwise=TKIP
    rsn_pairwise=CCMP
  EOF

  # Start AP
  /usr/sbin/hostapd -B /var/run/hostapd.conf -f /tmp/hostapd.log
  # Bridge AP and Mesh
  if [ "$algo" = "olsr" ]; then
    brctl addif br-lan "$mesh_if" "$ifname_ap"
    iptables -A FORWARD --in-interface $mesh_if -j ACCEPT
    killall olsrd 2>/dev/null  
    (olsrd -i wlp1s0 -d 0)&
  else
    ##batman-adv###
    brctl addif br-lan bat0 "$ifname_ap"
    iptables -A FORWARD --in-interface bat0 -j ACCEPT
  fi
  ifconfig br-lan $br_lan_ip netmask "255.255.255.0"
  ifconfig br-lan up
  echo
  ifconfig br-lan
  # Add forwading rules from AP to bat0 interface
  iptables -P FORWARD ACCEPT
  route del -net 192.168.1.0 gw 0.0.0.0 netmask 255.255.255.0 dev br-lan
  route add -net 192.168.1.0 gw $br_lan_ip netmask 255.255.255.0 dev br-lan
  iptables --table nat -A POSTROUTING --out-interface $ifname_ap -j MASQUERADE

  #setup minimalistic Mumble server configuration
  rm /etc/umurmur.conf
  cp /opt/mesh_com/modules/utils/docker/umurmur.conf /etc/umurmur.conf
  #Get GW IP
  if [ "$mode" = "sta+mesh" ]; then
    gw_ip=$(ifconfig $iface | grep "inet " | awk '{ print $2 }')
  elif [ "$mode" = "ap+mesh" ]; then
    gw_ip=$(ifconfig br-lan | grep "inet " | awk '{ print $2 }')
  fi
  echo "bindport = 64738;" >> /etc/umurmur.conf
  echo "bindaddr = "\"$gw_ip\"";" >> /etc/umurmur.conf
  sleep 10
  umurmurd

elif [ "$mode" == "ap+mesh_scc" ]; then
  sleep 2
  # chanbw config
  mount -t debugfs none /sys/kernel/debug
  if [ -f "/sys/kernel/debug/ieee80211/phy0/ath9k/chanbw" ]; then
    echo 20 > /sys/kernel/debug/ieee80211/phy0/ath9k/chanbw
  fi

  wifidev="$(ifconfig -a | grep -E '^wlp1' -m 1 | awk -F':' '{ print $1 }')"
  # Radio parameters
  iw dev "$wifidev" set txpower limit "$txpwr"00

  # RPi activity led config
  echo "phy0tx" > /sys/class/leds/led0/trigger

  #Create static mac addr for Batman if
  eth0_mac="$(ip -brief link | grep eth0 | awk '{print $3; exit}')"
  batif_mac="00:00:$(echo "$eth0_mac" | cut -b 7-17)"
  ifconfig bat0 hw ether "$batif_mac"

  brctl addbr br-lan
  # AP setup

  pcie_radio_mac="$(ip -brief link | grep "$wifidev" | awk '{print $3; exit}')"
  ssid="p2p#$(echo "$pcie_radio_mac" | cut -b 13-14,16-17)"

  ifname_ap="$wifidev-1"
  iw dev "$wifidev" interface add "$ifname_ap" type managed addr "00:01:$(echo "$pcie_radio_mac" | cut -b 7-17)"

  # Set frequency band and channel from given frequency
  calculate_wifi_channel $ch

  # AP hostapd config
  cat > /var/run/hostapd.conf <<- EOF
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

    ### IEEE 802.11n
    ieee80211n=0
    #ht_capab=[HT40+][LDPC][SHORT-GI-20][SHORT-GI-40][TX-STBC][RX-STBC1][DSSS_CCK-40]

    ### IEEE 802.11ac
    ieee80211ac=0
    #vht_capab=[MAX-MPDU-11454][RXLDPC][SHORT-GI-80][TX-STBC-2BY1][RX-STBC-1]
  EOF

  # Start AP
  /usr/sbin/hostapd -B /var/run/hostapd.conf -dd -f /tmp/hostapd_"$ifname_ap".log

  # Bridge AP and Mesh
  brctl addif br-lan bat0 "$ifname_ap"
  ifconfig "$wifidev" "$ipaddr" netmask "$nmask"
  ifconfig br-lan up
  echo
  ifconfig br-lan
  iptables -P FORWARD ACCEPT
  ip addr flush dev bat0
  echo "Mesh Point + AP done."

  IPV6_PREFIX=$( echo fd`dd if=/dev/urandom bs=7 count=1 status=none | xxd -p` | sed 's/\(....\)/\1:/g' )
  ip addr add $SUBNET.1/24 dev br-lan && ip addr add $IPV6_PREFIX:1/64 dev br-lan

  create_olsrd_config "$wifidev" "$SUBNET" "$IPV6_PREFIX"
  # FIXME: launch olsrd

  create_dhcpd_config "$SUBNET"
  dhcpd -f br-lan

  create_radvd_config "$IPV6_PREFIX"
  # FIXME: launch radvd

  echo 'Mesh Point + AP extra done'

else
   brctl addbr br-lan
  # Bridge ethernet and Mesh
  
  eth_port="eth1"
  if [ -f "$COMMS_PCB_VERSION_FILE" ]; then
    source "$COMMS_PCB_VERSION_FILE"
    # Sleeve 1.x has PCB version 0
    if (( $(echo "$COMMS_PCB_VERSION == 0" |bc -l) )); then
      eth_port="eth1"
    # CM1.5 PCB version is 0.5
    elif (( $(echo "$COMMS_PCB_VERSION == 0.5" |bc -l) )); then
      eth_port="eth0"
    # CM2.x PCB version starts from 1
    elif (( $(echo "$COMMS_PCB_VERSION >= 1" |bc -l) )); then
      eth_port="lan1"
    fi
  fi
  echo $eth_port

  brctl addif br-lan bat0 $eth_port
  echo $br_lan_ip
  ifconfig br-lan $br_lan_ip netmask "255.255.255.0"
  ifconfig br-lan up
  echo
  ifconfig br-lan
  # Add forwading rules from AP to bat0 interface
  iptables -P FORWARD ACCEPT
  route del -net 192.168.1.0 gw 0.0.0.0 netmask 255.255.255.0 dev br-lan
  route add -net 192.168.1.0 gw $br_lan_ip netmask 255.255.255.0 dev br-lan
  iptables -A FORWARD --in-interface bat0 -j ACCEPT
  iptables --table nat -A POSTROUTING --out-interface $br_lan_ip -j MASQUERADE
fi

#start gw manager
nohup python -u /opt/mesh_com/modules/sc-mesh-secure-deployment/src/gw/main.py

# WARNING: anything below the nohup above does not get executed for some reason

#start comms sleeve web server for companion phone
#nohup python -u /opt/mesh_com/modules/utils/docker/comms_sleeve_server.py -ip $br_lan_ip -ap_if $ifname_ap -mesh_if bat0
