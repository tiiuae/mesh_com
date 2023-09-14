#! /bin/bash
COMMS_PCB_VERSION_FILE="/opt/hardware/comms_pcb_version"

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

# bridge_settings
echo "Running bridge_settings"
brctl addbr br-lan
# Selecting the interfaces 

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
    MinRtrAdvInterval 3;
    MaxRtrAdvInterval 10;
    prefix $IPV6_PREFIX:0/64 {
    };
  };
EOF
}



# Bridge ethernet and Mesh for Batman-avd and routing for QoS-OLSR
if [ "$algo" = "olsr" ]; then
  brctl addif br-lan "$eth_port"
  IPV6_PREFIX=$( echo fd`dd if=/dev/urandom bs=7 count=1 status=none | xxd -p` | sed 's/\(....\)/\1:/g' )
  ip addr add $SUBNET.1/24 dev br-lan && ip addr add $IPV6_PREFIX:1/64 dev br-lan
  create_olsrd_config "wlp1s0" "$SUBNET" "$IPV6_PREFIX"
  create_dhcpd_config "$SUBNET"
  dhcpd -f br-lan
  create_radvd_config "$IPV6_PREFIX"
  br_lan_ip="$SUBNET."$((16#$ip_random))	
  wlp1s0_ip="192.168.11."$((16#$ip_random))
  ifconfig br-lan up
  ifconfig wlp1s0 "$wlp1s0_ip" 
  iptables -A FORWARD --in-interface $mesh_if -j ACCEPT
  iptables -A FORWARD -i br-lan -o $mesh_if -j ACCEPT 
  iptables -A FORWARD -o br-lan -i $mesh_if -j ACCEPT
  killall olsrd 2>/dev/null	
  (qos-olsrd -i wlp1s0 -d 0)&
  ifconfig "$mesh_if" mtu 1500
else
  brctl addif br-lan bat0 "$eth_port"
  iptables -A FORWARD --in-interface bat0 -j ACCEPT
  # Set mtu back to 1500 to support e2e connectivity
  # TODO: Investigate this as it should still work with 1460
  ifconfig bat0 mtu 1500
  ifconfig br-lan mtu 1500
  echo "$br_lan_ip"
  ifconfig br-lan "$br_lan_ip" netmask "255.255.255.0"
  ifconfig br-lan up
  echo
  ifconfig br-lan
fi
#Automatically assign the IP address on the brlan for the other interface 
dhcpd -cf  /etc/dhcp/dhcpd.conf br-lan

# Add forwarding rules from AP to $mesh_if interface
iptables -P FORWARD ACCEPT
route del -net 192.168.1.0 gw 0.0.0.0 netmask 255.255.255.0 dev br-lan
route add -net 192.168.1.0 gw $br_lan_ip netmask 255.255.255.0 dev br-lan
iptables -A FORWARD --in-interface bat0 -j ACCEPT
iptables --table nat -A POSTROUTING --out-interface $br_lan_ip -j MASQUERADE
