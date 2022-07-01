#! /bin/bash

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

if [[ -z "${1}"  || -z "${2}" ]]; then
    mode="ap+mesh"
    ch="2412"
else
   mode=$1
   ch=$2
fi

#install the python packages
if [ -d "/opt/mesh_com/modules/utils/package/python_packages" ]; then 
   echo "Directory /opt/mesh_com/modules/utils/package/python_packages exists."
else
   tar -C /opt/mesh_com/modules/utils/package/ -zxvf /opt/mesh_com/modules/utils/package/python_packages.tar.gz
   tar -C /opt/mesh_com/modules/utils/package/ -zxvf /opt/mesh_com/modules/utils/package/python_packages2.tar.gz
fi
pip install --no-index --find-links /opt/mesh_com/modules/utils/package/python_packages -r /opt/mesh_com/modules/utils/package/python_packages/requirements.txt
pip install --no-index --find-links /opt/mesh_com/modules/utils/package/python_packages2 -r /opt/mesh_com/modules/utils/package/python_packages2/requirements.txt

#copy libraries for pypcap
cp /opt/mesh_com/modules/utils/package/python_packages2/libpcap.so.0.8 /usr/lib/.
cp /opt/mesh_com/modules/utils/package/python_packages2/pcap.cpython-39-aarch64-linux-gnu.so /usr/lib/python3.9/site-packages/.

#start mesh service if mesh provisoning is done
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

create_ap_config()
{
cat <<EOF > ap.conf
network={
ssid="WirelessLab"
psk="ssrcdemo"
}
EOF
}

if [ "$mode" = "sta+mesh" ]; then
  create_ap_config
  #Connect to default GW AP
  iface=$(ifconfig -a | grep wlan* | awk -F':' '{ print $1 }')
  wpa_supplicant -Dnl80211 -i$iface -c ap.conf -B
  sleep 3
  udhcpc -i $iface
elif [ "$mode" = "ap+mesh" ]; then
  # Create bridge br-lan
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
  brctl addif br-lan bat0 "$ifname_ap"
  ifconfig br-lan "192.168.1.20" netmask "255.255.255.0"
  ifconfig br-lan up
  echo
  ifconfig br-lan
  # Add forwading rules from AP to bat0 interface
  iptables -P FORWARD ACCEPT
  route del -net 192.168.1.0 gw 0.0.0.0 netmask 255.255.255.0 dev br-lan
  route add -net 192.168.1.0 gw 192.168.1.20 netmask 255.255.255.0 dev br-lan
  iptables -A FORWARD --in-interface bat0 -j ACCEPT
  iptables --table nat -A POSTROUTING --out-interface $ifname_ap -j MASQUERADE
fi

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

#start gw manager
nohup python -u /opt/mesh_com/modules/sc-mesh-secure-deployment/src/gw/main.py
