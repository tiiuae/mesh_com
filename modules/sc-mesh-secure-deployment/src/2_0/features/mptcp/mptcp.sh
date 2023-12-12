#!/bin/bash
source /var/run/mptcp.conf
### MPTCP  ###
iface_list=$(grep 'INTERFACE' /var/run/mptcp.conf)
i=1;
while read n;
do
   iface=$(echo $n | awk -F "=" 'NR==1{print $2}');
   echo $iface;
   IP=$(ifconfig $iface | grep 'inet' | cut -d: -f2 | awk '{print $2}')
   NM=$(ifconfig $iface | grep 'netmask' | cut -d: -f2 | awk '{print $4}')
   IFS=. read -r i1 i2 i3 i4 <<< $IP
   IFS=. read -r m1 m2 m3 m4 <<< $NM
   NP=$(printf "%d.%d.%d.%d\n" "$((i1 & m1))" "$((i2 & m2))" "$((i3 & m3))" "$((i4 & m4))")
   MASK=$(ip addr show $iface | grep 'inet'| cut -d: -f2 | awk '{print $2}' | awk -F "/" '{print $2}')

   ip rule add from $IP table $i
   ip route add $NP/$MASK dev $iface scope link table $i

   ip mptcp endpoint add $IP signal

   if  [[ $iface == $BRIDGE_IFACE ]]; then
   	  BR_NP=$NP
      BR_MASK=$MASK
   fi
   i=$(($i+1));
done <<< "$iface_list"
SUBFLOWS=$i
ip mptcp limits set subflow $SUBFLOWS add_addr_accepted $SUBFLOWS

BR_PHY=$(brctl show | grep $BRIDGE_IFACE | awk -F " " '{printf $4}')
iptables -A FORWARD ! -p tcp -m physdev --physdev-in $BR_PHY -j ACCEPT

### PROXY ###
###iptables ss-redir rules###
iptables -t nat -N SSREDIR

iptables -t nat -A PREROUTING -p tcp -j SSREDIR

iptables -t nat -A SSREDIR -p tcp -d 127.0.0.0/8 -j RETURN
iptables -t nat -A SSREDIR -p tcp -d 10.0.0.0/8 -j RETURN

iptables -t nat -A SSREDIR -p tcp  -s $BR_NP/$BR_MASK -j REDIRECT --to-ports 1080

##currently the server ips (end system's ip) are hardcoded here. However with SLAAC this should be fixed
cat <<EOF > /var/run/ss-redir.json
{
    "server" : ["192.168.2.20"],
    "server_port" : 8388,
    "local_address" : "0.0.0.0",
    "local_port" : 1080,
    "password" : "mptcp",
    "timeout" : 300,
    "method" : "aes-256-cfb",
}
EOF

cat <<EOF > /var/run/ss-server.json
{
    "server" : ["[::0]", "0.0.0.0"],
    "server_port" : 8388,
    "local_port" : 1080,
    "password" : "mptcp",
    "timeout" : 300,
    "method" : "aes-256-cfb",
}
EOF
mptcpize run ss-redir -c /var/run/ss-redir.json &
mptcpize run ss-server -c /var/run/ss-server.json &





