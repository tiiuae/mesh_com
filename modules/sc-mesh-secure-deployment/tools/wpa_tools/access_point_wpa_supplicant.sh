#! /bin/bash

#to run this file:
#chmod + x access_point_wpa_supplicant.sh
#./access_point_wpa_supplicant.sh <interface to run the AP>
#TODO: Make it persistent


AP_interface=$1

#install dhcp-server
REQUIRED_PKG="isc-dhcp-server"
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $REQUIRED_PKG|grep "install ok installed")
echo Checking for $REQUIRED_PKG: "$PKG_OK"
if [ "" = "$PKG_OK" ]; then
  echo "No $REQUIRED_PKG. Setting up $REQUIRED_PKG."
  sudo apt-get --yes install $REQUIRED_PKG
fi

REQUIRED_PKG="net-tools"
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $REQUIRED_PKG|grep "install ok installed")
echo Checking for $REQUIRED_PKG: "$PKG_OK"
if [ "" = "$PKG_OK" ]; then
  echo "No $REQUIRED_PKG. Setting up $REQUIRED_PKG."
  sudo apt-get --yes install $REQUIRED_PKG
fi

echo "Killing wpa_supplicant..."
killall wpa_supplicant 2>/dev/null
sudo nmcli network off


#add to wpa_supplicant.conf
wpa_supplicant -B -i "$AP_interface" -c access-point.conf &

echo 'INTERFACES='"$AP_interface" >> /etc/default/isc-dhcp-server
cat <<EOF > /etc/dhcp/dhcpd.conf
subnet 10.10.0.0 netmask 255.255.255.0 {
  range 10.10.0.2 10.10.0.16;
  option domain-name-servers 8.8.4.4, 208.67.222.222;
  option routers 10.10.0.1;
}
EOF


#give configuration to AP_interface (assuming wlan0 will be the AP)
ifconfig "$AP_interface" 10.10.0.1
sudo service isc-dhcp-server start
echo 1| sudo tee /proc/sys/net/ipv4/ip_forward

internet_interface=$(ip -o link show | awk -F': ' '{print $2}' | awk '$1 != "lo"' | awk '$1 != "'"$AP_interface"'"' | awk '$1 != "eth0"'  | awk '$1 != "bat0"')

sudo iptables -t nat -A POSTROUTING -s 10.10.0.0/16 -o "$internet_interface" -j MASQUERADE  # internet_interface should be the interface that currently has internet


