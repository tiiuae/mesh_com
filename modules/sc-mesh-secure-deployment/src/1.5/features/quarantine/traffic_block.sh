#! /bin/bash

MAC=$1
EBT=/sbin/ebtables
IFACE=$2

function ebtables_block () {
	echo "EBTABLES blocking"
	## die if ebtables not found
	[[ -x "$EBT" ]] || { echo "$0: error \"$EBT\" not found."; exit 1; }
	$EBT -A FORWARD -s $MAC -j DROP  -i $IFACE
	$EBT -A INPUT   -s $MAC -j DROP  -i $IFACE
	$EBT -A FORWARD -s $MAC -j DROP
	$EBT -A INPUT   -s $MAC -j DROP
	$EBT -A FORWARD -s $MAC -j DROP  -i bat0
	$EBT -A INPUT   -s $MAC -j DROP  -i bat0
	$EBT -A OUTPUT  -s $MAC -j DROP  -o $IFACE
	$EBT -A OUTPUT  -s $MAC -j DROP  -o bat0
}

function nft_block {
	echo "NFT TABLES blocking"
	nft add table ip filter
	nft add chain ip filter INPUT { type filter hook input priority 0 \; } # create chain
	nft add rule filter INPUT iif $IFACE ether saddr == $MAC
	nft add rule filter INPUT iif bat0 ether saddr == $MAC
}

function iptables_block {
	echo "IPTABLES blocking"
	iptables -A INPUT -i bat0 -m mac ! --mac-source $MAC -j DROP
	iptables -A INPUT -i $IFACE -m mac ! --mac-source  $MAC -j DROP
}

function flush_all {
	echo "flushing all"
	iptables-save > iptables-`date "+%Y%m%d-%H%M%S"`
	iptables -F
#	$EBT -F
#	nft flush ruleset #flushing all the rules
}

## die if we are not root
[[ "$(id -u)" != "0" ]] && { echo "Error: $0 script must be run as root." 2>&1; exit 2; }
flush_all
#ebtables_block
iptables_block
#nft_block