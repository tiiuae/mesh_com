#! /bin/bash

################################################################################
# Creates dhcp conf, configures interface ip, and vonfigures routing for mesh AP
################################################################################
# Configure dhcp
cat <<EOF > /etc/dhcp/dhcpd.conf
subnet $2.10.0.0 netmask 255.255.255.0 {
  range $2.10.0.2 $2.10.0.16;
  option domain-name-servers 8.8.4.4, 208.67.222.222;
  option routers $2.10.0.1;
}
EOF
ifconfig "$1" $2.10.0.1
service isc-dhcp-server start
# output all traffic from $2.10.0.0/24 to bat0 (i.e. mesh inf with a gw route)
sysctl -w net.ipv4.ip_forward=1
sudo iptables -t nat -A POSTROUTING -s $2.10.0.0/24 -o bat0 -j MASQUERADE
