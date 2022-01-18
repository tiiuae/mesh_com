#! /bin/bash
################################################################################
# Creates dhcp conf, configures interface ip, and configures routing for mesh AP
################################################################################
# Configure dhcp
prefix=50
wpa_supplicant -i $1 -c /etc/wpa_supplicant/wpa_supplicant-$1.conf -B
cat <<EOF > /etc/dhcp/dhcpd.conf
subnet $prefix.10.0.0 netmask 255.255.255.0 {
  range $prefix.10.0.2 $prefix.10.0.16;
  option domain-name-servers 8.8.4.4, 208.67.222.222;
  option routers $prefix.10.0.1;
}
EOF
ifconfig "$1" $prefix.10.0.1
service isc-dhcp-server start
# output all traffic from $prefix.10.0.0/24 to bat0 (i.e. mesh inf with a gw route)
sysctl -w net.ipv4.ip_forward=1
iptables -t nat -A POSTROUTING -s $prefix.10.0.0/24 -o bat0 -j MASQUERADE