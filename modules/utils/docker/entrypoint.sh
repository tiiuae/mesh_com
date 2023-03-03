#! /bin/bash

MESH_FOLDER="/opt/mesh_com"

# Define constants
MESH_VERSION="1.5"
MESH_FOLDER="/opt/mesh_com"
SC_MESH_FOLDER="$MESH_FOLDER/modules/sc-mesh-secure-deployment/src/1_5"
COMMON_FOLDER="$SC_MESH_FOLDER/common"
FEATURES_FOLDER="$SC_MESH_FOLDER/features/mutual/utils"
ROOT_CERT="$COMMON_FOLDER/test/root_cert.der"
MESH_COM_CONF="$COMMON_FOLDER/mesh_com_11s.conf"
AUTH_AP_CONF="$FEATURES_FOLDER/auth_ap.yaml"
DHCPD_LEASES="/var/lib/dhcp/dhcpd.leases"

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


install_packages()
{
  #install the python packages
  if [ -d "$MESH_FOLDER/modules/utils/package/python_packages" ]; then
     echo "Directory $MESH_FOLDER/modules/utils/package/python_packages exists."
  else
     tar -C $MESH_FOLDER/modules/utils/package/ -zxvf $MESH_FOLDER/modules/utils/package/python_packages.tar.gz
  fi

  #pip install --no-index --find-links /opt/mesh_com/modules/utils/package/python_packages -r /opt/mesh_com/modules/utils/package/python_packages/requirements.txt
  #pip install --no-index --find-links /opt/mesh_com/modules/utils/package/python_packages2 -r /opt/mesh_com/modules/utils/package/python_packages2/requirements.txt

  #copy libraries for pypcap

  cd $MESH_FOLDER/modules/utils/package/python_packages
  for f in {*.whl,*.gz};
  do
    name="$(echo $f | cut -d'-' -f1)"
    if python -c 'import pkgutil; exit(not pkgutil.find_loader("$name"))'; then
      echo $name "installed"
    else
      echo $name "not found"
      echo "installing" $name
      pip install --no-index "$f" --find-links .;
  fi
  done;
  cd .. ;

  if [ "$ML" == "true" ]; then

    tar -C $MESH_FOLDER/modules/utils/package/ -zxvf $MESH_FOLDER/modules/utils/package/machine_learning_packages.tar.gz
    tar -C $MESH_FOLDER/modules/utils/package/ -zxvf $MESH_FOLDER/modules/utils/package/machine_learning2.tar.gz

    cd $MESH_FOLDER/modules/utils/package/machine_learning_packages

    for f in {*.whl,*.gz};
    do
      name="$(echo $f | cut -d'-' -f1)"
      if python -c 'import pkgutil; exit(not pkgutil.find_loader("$name"))'; then
        echo $name "installed"
      else
        echo $name "not found"
        echo "installing" $name
        pip install --no-index "$f" --find-links .;
    fi
    done;
    cd .. ;

    cd $MESH_FOLDER/modules/utils/package/machine_learning2

    for f in {*.whl,*.gz};
    do
      name="$(echo $f | cut -d'-' -f1)"
      if python -c 'import pkgutil; exit(not pkgutil.find_loader("$name"))'; then
        echo $name "installed"
      else
        echo $name "not found"
        echo "installing" $name
        pip install --no-index "$f" --find-links .;
    fi
    done;
    cd .. ;

    cp $MESH_FOLDER/modules/utils/package/machine_learning_packages/libpcap.so.0.8 /usr/lib/.
    cp $MESH_FOLDER/modules/utils/package/machine_learning_packages/pcap.cpython-39-aarch64-linux-gnu.so /usr/lib/python3.9/site-packages/.
  fi
}

configure()
{
  if [ -f "/opt/mesh.conf" ]; then
      source /opt/mesh.conf
      mode=$CONCURRENCY
      ch=$MCC_CHANNEL
      ipaddr=$IP
      nmask=$MASK
      cc=$COUNTR
      psk=$KEY
      txpwr=$TXPOWER
      algo=$ROUTING
      mesh_if=$MESH_VIF
      meshVersion=$MS15
      ML=$ML
  else
      mode="mesh"
  fi
}

mesh_service()
{
  #start mesh service if mesh provisioning is done
  hw_platform=$(cat /proc/cpuinfo | grep Model | awk '{print $5}')
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
}

provisioning()
{
   killall hostapd
   killall wpa_supplicant
   if [[ -d /sys/class/net/br-lan ]]; then
         ifconfig br-lan down
         brctl delbr br-lan
   fi
   ifname_ap="$(ifconfig -a | grep wlan* | awk -F':' '{ print $1 }')"
   ifname_mp="$(ifconfig -a | grep wlp1* | awk -F':' '{ print $1 }')"
   ifconfig $ifname_ap down
   ifconfig $ifname_mp down
   ifconfig $ifname_ap up
   ifconfig $ifname_mp up
   if [ -z "$1" ]; then
    exit 1
  fi
}


create_ap_config()
{
cat <<EOF > ap.conf
network={
ssid="WirelessLab"
psk="ssrcdemo"
}
EOF
}


bug_initializing()
{
/sbin/ip link set wlan0 down
/sbin/ip link set wlan0 name wlan1
/sbin/ip link set wlan1 up
}


generate_random_mesh_ip() {
  local last_oct=$((16#$1))
  local random_ip="10.10.10.$((last_oct))"
  echo "$random_ip"
}

update_mesh_com_conf() {
  local random_ip="$1"
  sed -i "s/ip: .*/ip: $random_ip/" "$MESH_COM_CONF"
}


update_auth_ap_conf() {
  local auth_ap="$1"
  sed -i "s/ip: .*/ip: $auth_ap/" "$AUTH_AP_CONF"
}

get_available_networks() {
  /sbin/ip -4 -o addr show scope global |
    awk '{gsub(/\/.*/,"",$4); print $4}' |
    awk '{split($1,a,"[.]"); print a[1],a[2],a[3]}' OFS="." |
    sort -u
}


start_mesh() {
  cd "$SC_MESH_FOLDER"
  python3 -u main.py
}


get_unused_network() {
  local used_networks="$1"
  local available_networks=($(seq 15 50 | grep -Fxv -e{10,"used_networks[@]"} | shuf))
  local unused_network="1${available_networks[0]}.0.0"
  echo "$unused_network"
}

configure
bug_initializing

###Deciding IP address to be assigned to br-lan from WiFi MAC
mesh_if_mac="$(ip -brief link | grep "$mesh_if" | awk '{print $3; exit}')"
ip_random="$(echo "$mesh_if_mac" | cut -b 16-17)"
br_lan_ip="192.168.1."$((16#$ip_random))


if [ "$MESH_VERSION" == "1.5" ]; then
  install_packages
  # Generate a random IP address for the mesh network
  ip_random="$(openssl rand -hex 1)"
  random_ip="$(generate_random_mesh_ip "$ip_random")"

  # Update mesh configuration file with the random IP
  update_mesh_com_conf "$random_ip"

  # Check if the network is available and generate a new one if necessary
  used_networks=($(get_available_networks))
  if [[ "${#used_networks[@]}" -gt 0 ]]; then
    unused_network="$(get_unused_network "${used_networks[@]}")"
    auth_ap="$unused_network.1"
  else
     auth_net="1$(shuf -i 15-50 -n 1).0.0"
     auth_ap=$auth_net".1"
  fi

  # Update authentication AP configuration file with the new IP
  update_auth_ap_conf "$auth_ap"

  provisioning true

  # Copy the root certificate for testing purposes
  cp "$ROOT_CERT" "/etc/ssl/certs/"

  # Create the DHCP leases file
  touch "$DHCPD_LEASES"

  cp $COMMON_FOLDER/test/root_cert.der /etc/ssl/certs/ # this is for testing only, must be provided
  touch /var/lib/dhcp/dhcpd.leases

  uid=$(echo -n $mesh_if_mac | b2sum -l 32)
  uid=${uid::-1}
  /bin/bash $MESH_FOLDER/common/scripts/generate_keys.sh $uid
  start_mesh
fi


if [ "$mode" == "provisioning" ]; then
  provisioning
fi

if [ "$mode" = "sta+mesh" ]; then
  mesh_service
  create_ap_config
  #Connect to default GW AP
  iface=$(ifconfig -a | grep wlan* | awk -F':' '{ print $1 }')
  wpa_supplicant -Dnl80211 -i$iface -c ap.conf -B
  sleep 3
  udhcpc -i $iface

elif [ "$mode" = "ap+mesh_mcc" ]; then
  # Create bridge br-lan
  mesh_service
  brctl addbr br-lan
  ifname_ap="$(ifconfig -a | grep wlan* | awk -F':' '{ print $1 }')"
  ap_if_mac="$(ip -brief link | grep "$ifname_ap" | awk '{print $3; exit}')"
  ssid="comms_sleeve#$(echo "$ap_if_mac" | cut -b 13-14,16-17)"
  # Set frequency band and channel from given frequency
  calculate_wifi_channel "$ch"
  ifconfig $ifname_ap up

 # AP hostapd config
cat <<EOF >/var/run/hostapd.conf
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
        (olsrd -i br-lan -d 0)&
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
  mesh_service
  # chanbw config
  mount -t debugfs none /sys/kernel/debug
  if [ -f "/sys/kernel/debug/ieee80211/phy0/ath9k/chanbw" ]; then
      echo 20 > /sys/kernel/debug/ieee80211/phy0/ath9k/chanbw
  fi

  wifidev="$(ifconfig -a | grep wlp1* | awk -F':' '{ print $1 }')"
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
cat <<EOF >/var/run/hostapd.conf
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
#beacon_int=1000

### IEEE 802.11n
ieee80211n=1
#ht_capab=[HT40+][LDPC][SHORT-GI-20][SHORT-GI-40][TX-STBC][RX-STBC1][DSSS_CCK-40]

### IEEE 802.11ac
#ieee80211ac=1
#vht_capab=[MAX-MPDU-11454][RXLDPC][SHORT-GI-80][TX-STBC-2BY1][RX-STBC-1]
EOF

  # Start AP
  /usr/sbin/hostapd -B /var/run/hostapd.conf -dd -f /tmp/hostapd_"$ifname_ap".log

  # Bridge AP and Mesh
  brctl addif br-lan bat0 "$ifname_ap"
  ifconfig br-lan "$ipaddr" netmask "$nmask"
  ifconfig br-lan up
  echo
  ifconfig br-lan
  iptables -P FORWARD ACCEPT
  ip addr flush dev bat0
  echo "Mesh Point + AP done."
else
  brctl addbr br-lan
  # Bridge eth1 and Mesh
  brctl addif br-lan bat0 eth1
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

#start gw manager
#nohup python -u /opt/mesh_com/modules/sc-mesh-secure-deployment/src/gw/main.py
# current path is /opt/mesh_com-mesh_com_1.5/modules/sc-mesh-secure-deployment/src/1_5
nohup python -u ../gw/main.py

fi


#start comms sleeve web server for companion phone
#nohup python -u /opt/mesh_com/modules/utils/docker/comms_sleeve_server.py -ip $br_lan_ip -ap_if $ifname_ap  -mesh_if bat0