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
      brctl addif br-lan "$mesh_if" "$ifname_ap"
      iptables -A FORWARD --in-interface "$mesh_if" -j ACCEPT
      killall olsrd 2>/dev/null
      (olsrd -i br-lan -d 0)&
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