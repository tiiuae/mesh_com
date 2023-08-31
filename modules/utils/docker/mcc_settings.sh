#! /bin/bash

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
      meshVersion=$MSVERSION
      ML=$ML
      bridge=$BRIDGE
  else
      mode="mesh"
  fi
}

configure

###Deciding IP address to be assigned to br-lan from WiFi MAC
mesh_if_mac="$(ip -brief link | grep "$mesh_if" | awk '{print $3; exit}')"
ip_random="$(echo "$mesh_if_mac" | cut -b 16-17)"
br_lan_ip="192.168.1."$((16#$ip_random))
ifname_ap="$(ifconfig -a | grep "wlan*" | awk -F':' '{ print $1 }')"

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

create_ap_config()
{
cat <<EOF > ap.conf
network={
ssid="WirelessLab"
psk="ssrcdemo"
}
EOF
}

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

  #LoadPlugin "/usr/lib/olsrd_jsoninfo.so.1.1"

  #{

    #PlParam "port"          "9090"

    #PlParam "accept"        "0.0.0.0"

  #}

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

# mcc_settings
echo "Running mcc_settings"
# Create bridge br-lan
brctl addbr br-lan
ap_if_mac="$(ip -brief link | grep "$ifname_ap" | awk '{print $3; exit}')"
ssid="comms_sleeve#$(echo "$ap_if_mac" | cut -b 13-14,16-17)"
# Set frequency band and channel from given frequency
calculate_wifi_channel "$ch"
ifconfig "$ifname_ap" up

# AP hostapd config
cat <<EOF >/var/run/hostapd.conf
country_code=AE
interface=$ifname_ap
ssid=$ssid
hw_mode=$retval_band
channel=$retval_channel
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
  IPV6_PREFIX=$( echo fd`dd if=/dev/urandom bs=7 count=1 status=none | xxd -p` | sed 's/\(....\)/\1:/g' )
  ip addr add $SUBNET.1/24 dev br-lan && ip addr add $IPV6_PREFIX:1/64 dev br-lan
  create_olsrd_config "wlp1s0" "$SUBNET" "$IPV6_PREFIX"
  # FIXME: launch olsrd
  create_dhcpd_config "$SUBNET"
  dhcpd -f br-lan
  create_radvd_config "$IPV6_PREFIX"
# FIXME: launch radvd
  wlp1s0_ip="192.168.11."$((16#$ip_random))
  ifconfig wlp1s0 "$wlp1s0_ip" 
  brctl addif br-lan "$ifname_ap"
  iptables -A FORWARD --in-interface $mesh_if -j ACCEPT
  killall olsrd 2>/dev/null	
  (qos-olsrd -i wlp1s0 -d 0)&
  ifconfig "$mesh_if" mtu 1500
else
  ##batman-adv###
  brctl addif br-lan bat0 "$ifname_ap"
  iptables -A FORWARD --in-interface bat0 -j ACCEPT
  ifconfig bat0 mtu 1500
fi

# Set mtu back to 1500 to support e2e connectivity
# TODO: Investigate this as it should still work with 1460
ifconfig br-lan mtu 1500

ifconfig br-lan "$br_lan_ip" netmask "255.255.255.0"
ifconfig br-lan up
echo
ifconfig br-lan
# Add forwarding rules from AP to $mesh_if interface
iptables -P FORWARD ACCEPT
route del -net 192.168.1.0 gw 0.0.0.0 netmask 255.255.255.0 dev br-lan
route add -net 192.168.1.0 gw "$br_lan_ip" netmask 255.255.255.0 dev br-lan
iptables --table nat -A POSTROUTING --out-interface "$ifname_ap" -j MASQUERADE

#setup minimalistic Mumble server configuration
rm /etc/umurmur.conf
cp /opt/mesh_com/modules/utils/docker/umurmur.conf /etc/umurmur.conf