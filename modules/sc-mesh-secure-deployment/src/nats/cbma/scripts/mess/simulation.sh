#! /bin/bash -e


export MAYBE1500='1400'



export SHELLOPTS
export PATH="$PATH:/usr/games"
export PS4='\033[1m${LINENO}:\033[0m	'

# modprobe mac80211_hwsim radios=6

# This file creates a star infrastructure with 6 nodes all communicating in the
# lower tier but with two separate upper tiers: inner nodes A, B & C separate
# from the outer nodes D, E & F:
#
#    D - (brAD) - A \       / B - (brBE) - E
#		    (brABC)
#		       |
#		       C
#		       |
#		     (brCF)
#		       |
#		       F
#
# Each node is simulated with a network namespace (kind of like a VM, just very
# lightweight) and the bridges between them are not required for this to work,
# these extra bridges are here only to make it easy to observe with Wireshark.


NODES=`echo {A..C}`


set -x
#command 2> >(sed $'s/^[^+].*/\e[31m&\e[m/'>&2)

random_mac_address()
{
        if [ 1 -eq $# ]; then
                OPTIONAL_RECURSION_LEVEL="$1"
        else
                OPTIONAL_RECURSION_LEVEL=0
        fi

        set -o pipefail
        if REST=`dd if=/dev/urandom bs=5 count=1 status=none | xxd -p | sed 's/\(..\)/:\1/g'`; then
                MAC="02$REST"
        else
                MAC=`printf '02:%02x:%02x:%02x:%02x:%02x' $(( $RANDOM % 256 )) $(( $RANDOM % 256 ))
$(( $RANDOM % 256 )) $(( $RANDOM % 256 )) $(( $RANDOM % 256 ))`
        fi
        if ip link show | grep -Fq "$MAC"; then
                if [ $OPTIONAL_RECURSION_LEVEL -gt 9 ]; then
                        >&2 echo 'No randomness? Something seriously wrong!'
                        exit 1
                else
                        random_mac_address $(( $OPTIONAL_RECURSION_LEVEL + 1 ))
                fi
        else
                echo "$MAC"
        fi
}
 


secure_bat0()
{
	NAMESPACE="$1"
	DEV='bat0'

	for I in $NODES; do
		[ "$I" != "$NAMESPACE" ] || continue
		REMOTE_MAC="00:20:91:$I$I:${I}1:11"
		TXKEY=`echo -n "$NAMESPACE$I" | sha256sum | cut -d ' ' -f 1`
		RXKEY=`echo -n "$I$NAMESPACE" | sha256sum | cut -d ' ' -f 1`
		./create_mess.sh u "$DEV" "$REMOTE_MAC" "$TXKEY" "$RXKEY"
	done
}


secure_halow0()
{
	NAMESPACE="$1"
	DEV="$2"

	I="$NAMESPACE"
	ip link set dev "$DEV" address "00:20:91:$I$I:${I}3:33"
	ip link set dev "$DEV" mtu $(( $MAYBE1500 + 72 ))
	ip link set dev "$DEV" up
	iw dev "$DEV" mesh join gold freq 2462

	for I in $NODES; do
		[ "$I" != "$NAMESPACE" ] || continue
		REMOTE_MAC="00:20:91:$I$I:${I}3:33"
		TXKEY=`echo -n "$NAMESPACE$I" | sha256sum | cut -d ' ' -f 1`
		RXKEY=`echo -n "$I$NAMESPACE" | sha256sum | cut -d ' ' -f 1`
		./create_mess.sh u "$DEV" "$REMOTE_MAC" "$TXKEY" "$RXKEY"
	done
}



secure_wlan0()
{
	NAMESPACE="$1"
	DEV="$2"

	I="$NAMESPACE"
	ip link set dev "$DEV" address "00:20:91:$I$I:${I}0:00"
	ip link set dev "$DEV" mtu $(( $MAYBE1500 + 72 ))
	ip link set dev "$DEV" up
	iw dev "$DEV" mesh join gold freq 2412 HT40+

	for I in $NODES; do
		[ "$I" != "$NAMESPACE" ] || continue
		REMOTE_MAC="00:20:91:$I$I:${I}0:00"
		TXKEY=`echo -n "$NAMESPACE$I" | md5sum | cut -d ' ' -f 1`
		RXKEY=`echo -n "$I$NAMESPACE" | md5sum | cut -d ' ' -f 1`
		./create_mess.sh l "$DEV" "$REMOTE_MAC" "$TXKEY" "$RXKEY"
	done
}



setup_node()
{
	local NS="$1"
	WIFI_DEV="$2"
	HALOW_DEV="$3"

	# Ubuntu built-in kernel unfortunately has # CONFIG_BATMAN_ADV_BATMAN_V is not set
	batctl meshif bat0 interface create routing_algo BATMAN_V || ip link add name bat0 type batadv
	# Might also be missing https://www.open-mesh.org/attachments/950
	ip link set dev bat0 mtu $(( $MAYBE1500 + 72 )) || true
	ip link set dev bat0 address "00:20:91:$NS$NS:${NS}1:11"
	ip link set dev bat0 up
	secure_wlan0 "$NS" "$WIFI_DEV"
	if [ -n "$HALOW_DEV" ]; then
		ip link add name br-lan address `random_mac_address` type bridge
		ip link set dev br-lan mtu $MAYBE1500
		ip link set dev br-lan up
		batctl meshif bat1 interface create routing_algo BATMAN_V || ip link add name bat1 type batadv
		ip link set dev bat1 master br-lan
		ip link set dev bat1 address "00:20:91:$NS$NS:${NS}2:22"
		ip link set dev bat1 mtu $MAYBE1500
		ip link set dev bat1 up
		secure_bat0 "$NS"
		secure_halow0 "$NS" "$HALOW_DEV"
	fi
}



if [ 0 -eq $# ]; then
	if [ -e ./max_nest_dev.sh ]; then
		MAX_NEST_DEV=`./max_nest_dev.sh`
	else
		MAX_NEST_DEV=8
	fi
	modprobe -r mac80211_hwsim
	if [ $MAX_NEST_DEV -ge 10 ]; then
		modprobe mac80211_hwsim radios=6
	else
		modprobe mac80211_hwsim radios=3
	fi
	N=0
	for I in $NODES; do
		ip netns delete "$I" || true
		ip netns add "$I"
		PHY=`ls "/sys/devices/virtual/mac80211_hwsim/hwsim$N/ieee80211/"`
		DEV=`ls "/sys/class/ieee80211/$PHY/device/net/"`
		iw dev "$DEV" del
		iw phy "$PHY" set netns name "$I"
		WIFI_DEV='wlan0'
		ip netns exec "$I" iw phy "$PHY" interface add "$WIFI_DEV" type mesh
		if [ $MAX_NEST_DEV -ge 10 ]; then
			N=$(( $N + 1 ))
			PHY=`ls "/sys/devices/virtual/mac80211_hwsim/hwsim$N/ieee80211/"`
			DEV=`ls "/sys/class/ieee80211/$PHY/device/net/"`
			iw dev "$DEV" del
			iw phy "$PHY" set netns name "$I"
			HALOW_DEV='halow0'
			ip netns exec "$I" iw phy "$PHY" interface add "$HALOW_DEV" type mesh
		fi
		ip netns exec "$I" "./$0" setup_node "$I" "$WIFI_DEV" "$HALOW_DEV"
		N=$(( $N + 1 ))
	done
	set +x
	export -n SHELLOPTS
	#read -p 'Press enter when ready to capture with Wireshark'
	ifconfig hwsim0 up
	echo 'FIXME: do something'
	for ROUND in {1..10}; do
		for I in $NODES; do
			echo "$ROUND: Checking traffic in $I (batman-adv might need a moment)"
			ip netns exec $I ./check_traffic.sh
		done
	done
	read -p 'Press enter when ready to exit'
	echo 'FIXME: cleanup'
else
	$@
fi
