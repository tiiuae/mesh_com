#!/bin/bash
source /var/run/mptcp.conf

### MPTCP  ###
_slaac=$(grep "SLAAC" /var/run/mptcp.conf | awk -F '=' '{print $2}')
_slaac_interfaces=$(echo $_slaac | tr ' ' '\n' | sort | uniq | tr '\n' ' ' | sed -e 's/[[:space:]]*$//')

iface_list=$(grep 'INTERFACE' /var/run/mptcp.conf)
i=0;
while read n;
do
   iface=$(echo $n | awk -F "=" 'NR==1{print $2}');
   echo "iface: $iface"
   if [ "$iface" == "wlp2s0" ]; then
       IPv6_prefix="fd1d::"
   elif [ "$iface" == "halow1" ]; then
        IPv6_prefix="fdbd::"
   elif [ "$iface" == "br-lan" ]; then
        IPv6_prefix="fdcd::"
   fi
   
   echo "_slaac_interfaces: $_slaac_interfaces"
   for _slaac_interface in $_slaac_interfaces; do
   	MAC_ADDRESS_FILE="/sys/class/net/$_slaac_interface/address"
        MAC_ADDRESS=`cat "$MAC_ADDRESS_FILE"`
        echo "MAC_ADDRESS: $MAC_ADDRESS"
        # Split the MAC address into its components
	mac_parts=(${MAC_ADDRESS//:/ })
	# Invert the 7th bit of the first byte
	first_byte=$(printf "%02x" $((0x${mac_parts[0]} ^ 0x02)))
	# Assemble the EUI-64 address
	eui64="${first_byte}${mac_parts[1]}:${mac_parts[2]}ff:fe${mac_parts[3]}:${mac_parts[4]}${mac_parts[5]}"
        # Combine the IPv6 prefix with the EUI-64 address
	IPv6_address="${IPv6_prefix}${eui64}/64"
        echo "IPv6_address:$IPv6_address"
        i=$(($i+1));
        ip -6 addr add $IPv6_address dev $iface
        ip -6 rule add from $IPv6_address table $i
        ip route add $IPv6_prefix/64 dev $iface scope link table $i
        ip mptcp endpoint add $IPv6_prefix$eui64 signal
  done  
done <<< "$iface_list"

ip mptcp limits set subflow $i add_addr_accepted $i
echo "SUBFLOWS=$i" >> /var/run/mptcp.conf


olsrd -ipv6 > /opt/olsrd.log &

mptcpize run ss-server -c /opt/mesh_com/modules/sc-mesh-secure-deployment/src/2_0/features/mptcp/ss-server_config.json > /opt/ss_server.log &
