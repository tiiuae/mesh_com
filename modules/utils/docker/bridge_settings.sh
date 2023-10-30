#! /bin/bash
COMMS_PCB_VERSION_FILE="/opt/hardware/comms_pcb_version"
source ./qos_olsrd_conf.sh
configure()
{
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
      meshVersion=$MSVERSION
      ML=$ML
      bridge=$BRIDGE
      subnet=$SUBNET
      wifi_interface=$WIFI_INTERFACE
      
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




# Bridge ethernet and Mesh for Batman-avd and routing for QoS-OLSR
if [ "$algo" = "olsr" ]; then
  brctl addif br-lan "$eth_port"
  IPV6_PREFIX=$( echo fd`dd if=/dev/urandom bs=7 count=1 status=none | xxd -p` | sed 's/\(....\)/\1:/g' )
  ip addr add $SUBNET.1/24 dev br-lan 
  ip addr add $IPV6_PREFIX:1/64 dev br-lan
  create_olsrd_config "$WIFI_INTERFACE" "$SUBNET" 
  create_olsrd_config6 "$WIFI_INTERFACE" "$IPV6_PREFIX"
  create_dhcpd_config "$SUBNET"
  dhcpd -f br-lan
  create_radvd_config "$IPV6_PREFIX"
  br_lan_ip="$SUBNET."$((16#$ip_random))	
  wlp1s0_ip="192.168.11."$((16#$ip_random))
  ifconfig br-lan up
  ifconfig "$WIFI_INTERFACE" "$wlp1s0_ip" 
  #iptables -A FORWARD --in-interface $mesh_if -j ACCEPT
  #iptables -A FORWARD -i br-lan -o $mesh_if -j ACCEPT 
  #iptables -A FORWARD -o br-lan -i $mesh_if -j ACCEPT
  killall olsrd 2>/dev/null	
  qos-olsrd -i "$WIFI_INTERFACE" -d 0 -f /etc/olsrd/olsrd.conf
  qos-olsrd -i "$WIFI_INTERFACE" -d 0 -f /etc/olsrd/olsrd6.conf
  ifconfig "$mesh_if" mtu 1500
  #Automatically assign the IP address on the brlan for the other interface 
  dhcpd -cf  /etc/dhcp/dhcpd.conf br-lan
elif [ "$algo" = "batman-adv" ]; then
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
  # Add forwarding rules from AP to $mesh_if interface
  #iptables -P FORWARD ACCEPT
  #route del -net 192.168.1.0 gw 0.0.0.0 netmask 255.255.255.0 dev br-lan
  #route add -net 192.168.1.0 gw $br_lan_ip netmask 255.255.255.0 dev br-lan
  #iptables -A FORWARD --in-interface bat0 -j ACCEPT
  #iptables --table nat -A POSTROUTING --out-interface $br_lan_ip -j MASQUERADE
else 
  echo You may choose for now either batman-adv or olsr protocol
  exit 1	
fi

radvd -C /etc/radvd.conf -d 0

