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
# Bridge ethernet and Mesh

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

# Set mtu back to 1500 to support e2e connectivity
# TODO: Investigate this as it should still work with 1460
ifconfig bat0 mtu 1500
ifconfig br-lan mtu 1500

brctl addif br-lan bat0 $eth_port
echo $br_lan_ip
ifconfig br-lan $br_lan_ip netmask "255.255.255.0"
ifconfig br-lan up
echo
ifconfig br-lan
# Add forwarding rules from AP to $mesh_if interface
iptables -P FORWARD ACCEPT
route del -net 192.168.1.0 gw 0.0.0.0 netmask 255.255.255.0 dev br-lan
route add -net 192.168.1.0 gw $br_lan_ip netmask 255.255.255.0 dev br-lan
iptables -A FORWARD --in-interface bat0 -j ACCEPT
iptables --table nat -A POSTROUTING --out-interface $br_lan_ip -j MASQUERADE