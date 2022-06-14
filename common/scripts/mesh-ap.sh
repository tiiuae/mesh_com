#! /bin/bash
################################################################################
# Creates dhcp conf, configures interface ip, and configures routing for mesh AP
################################################################################
subnet=$(awk -F"." '{print $1"."$2"."$3".0"}'<<<$2)
range1=$(awk -F"." '{print $1"."$2"."$3".2"}'<<<$2)
range2=$(awk -F"." '{print $1"."$2"."$3".16"}'<<<$2)
# Configure dhcp
#wpa_supplicant -i $1 -c /etc/wpa_supplicant/wpa_supplicant-$1.conf -B
/usr/sbin/hostapd -B /var/run/hostapd.conf -f /tmp/hostapd.log
cat <<EOF > /etc/dhcp/dhcpd.conf
subnet $subnet netmask 255.255.255.0 {
  range $range1 $range2;
  option domain-name-servers 8.8.4.4, 208.67.222.222;
  option routers $2;
}
EOF
ifconfig "$1" $2
echo "DHCPDARGS=$1" > /etc/sysconfig/dhcpd
killall dhcpd
dhcpd -cf /etc/dhcp/dhcpd.conf
# output all traffic from $2.10.0.0/24 to bat0 (i.e. mesh inf with a gw route)
sysctl -w net.ipv4.ip_forward=1
iptables -t nat -A POSTROUTING -s $2/24 -o bat0 -j MASQUERADE
