#! /bin/bash


source "${BASH_SOURCE%/*}/constants.rc"

flip_locally_administered_bit()
{
	MAC_ADDRESS="$1"

	NIBBLE1=`echo "$MAC_ADDRESS" | cut -c 1`
	LOCALLY_ADMINISTERED_FLIPPED_NIBBLE2=`echo "$MAC_ADDRESS" | cut -c 2 | tr '[0123456789abcdef]' '[23016745ab89efcd]'`
	REST_OF_MAC_ADDRESS=`echo "$MAC_ADDRESS" | cut -c 3-`
	echo "$NIBBLE1$LOCALLY_ADMINISTERED_FLIPPED_NIBBLE2$REST_OF_MAC_ADDRESS"
}



mac_to_eui64()
{
	MAC_ADDRESS="$1"

	BYTE1=`flip_locally_administered_bit "$MAC_ADDRESS" | cut -c 1,2`
	BYTE2=`echo "$MAC_ADDRESS" | cut -c 4,5`
	BYTE3=`echo "$MAC_ADDRESS" | cut -c 7,8`
	BYTE4=`echo "$MAC_ADDRESS" | cut -c 10,11`
	BYTE5=`echo "$MAC_ADDRESS" | cut -c 13,14`
	BYTE6=`echo "$MAC_ADDRESS" | cut -c 16,17`
	echo "$BYTE1$BYTE2:${BYTE3}ff:fe${BYTE4}:$BYTE5$BYTE6"
}



# Workaround for yet another kernel bug: 802.1Q VLAN tagging just to make BATMAN_V's ELP frames a few bytes bigger to not be mistakenly considered too small
create_macscbpad_interface()
{
	if ! ip link add link "$MACSCBUB_NAME" name "$MACSCBPAD_NAME" type vlan id 1 reorder_hdr on; then
		return `false`
	fi
	ip link set dev "$MACSCBPAD_NAME" group "$GROUP_ID" || true
	ip link set dev "$MACSCBPAD_NAME" arp off || true
	ip link set dev "$MACSCBPAD_NAME" multicast off || true
	ip link set dev "$MACSCBPAD_NAME" alias "$LEVEL intentional overhead workaround" || true
	ip link set dev "$MACSCBPAD_NAME" addrgenmode none || true
	if ! ip link set dev "$MACSCBPAD_NAME" master "$MACBR_NAME" \
	|| ! ip link set dev "$MACSCBPAD_NAME" type bridge_slave isolated on; then
		return `false`
	fi
	ip link set dev "$MACSCBPAD_NAME" type bridge_slave guard on || true
	ip link set dev "$MACSCBPAD_NAME" type bridge_slave hairpin off || true
	ip link set dev "$MACSCBPAD_NAME" type bridge_slave learning off
	bridge link set dev "$MACSCBPAD_NAME" learning off
	bridge link set dev "$MACSCBPAD_NAME" learning_sync off
	ip link set dev "$MACSCBPAD_NAME" type bridge_slave bcast_flood on
	ip link set dev "$MACSCBPAD_NAME" type bridge_slave neigh_suppress off || true
	ebtables -t nat -A "$MACBR_NAME" -o "$MACSCBPAD_NAME" -d '!' Broadcast -j DROP
	ebtables -t nat -A PREROUTING -i "$MACSCBPAD_NAME" -j dnat --to-destination ff:ff:ff:ff:ff:ff && ebtables -t nat -A "$MACBR_NAME" -o "$MACSCBPAD_NAME" -j dnat --to-destination "$REMOTE_MAC"
	if ! ip link set dev "$MACSCBPAD_NAME" up; then
		ip link delete "$MACSCBPAD_NAME"
		return `false`
	fi
}



# MACsec Single Channel Broadcast to unicast to broadcast (batadv workaround)
create_macscbub_interface()
{
	local PORT=0
	local TXH="$TXKEY"
	local RXH="$RXKEY"
	# Required due to the following bugs:
	# `ip ... scb on ...` mistakenly implies port 1 instead of 0 as required by standard
	# `ip ... port 0 ...` incorrectly rejects port 0 as invalid regardless of scb on/off
	# `ip ... sci ...` works around the above but kernel incorrectly forces `send_sci on`
	# Broadcasts should have room for SCI, though it might mess up MTU calculations...
	local SCB_TXSCI=`echo "${LOCAL_MAC}0000" | tr -d ':'`
	local SCB_RXSCI=`echo "${REMOTE_MAC}0000" | tr -d ':'`

	if ! ip link add link "$MACVLAN_NAME" name "$MACSCBUB_NAME" mtu "$MACSEC_MTU" type macsec sci "$SCB_TXSCI" icvlen 8 encrypt "$MACSEC_ENCRYPT" protect on scb on cipher "$MACSEC_CIPHER" \
	|| ! ip macsec add "$MACSCBUB_NAME" rx sci "$SCB_RXSCI"; then
		return `false`
	fi
	ip link set dev "$MACSCBUB_NAME" group "$GROUP_ID" || true
	ip link set dev "$MACSCBUB_NAME" arp off || true
	ip link set dev "$MACSCBUB_NAME" multicast off || true
	ip link set dev "$MACSCBUB_NAME" alias "$LEVEL broad2uni2broadcast with $REMOTE_MAC" || true
	ip link set dev "$MACSCBUB_NAME" addrgenmode none || true
	for SA in 0 1 2 3; do
		TXH=`echo "$PORT$TXH$SA" | sha512sum | cut -c "-$MACSEC_HEXLEN"`
		RXH=`echo "$PORT$RXH$SA" | sha512sum | cut -c "-$MACSEC_HEXLEN"`
		if ! ip macsec add "$MACSCBUB_NAME" tx sa "$SA" pn 1 on key "$PORT$SA" "$TXH" \
		|| ! ip macsec add "$MACSCBUB_NAME" rx sci "$SCB_RXSCI" sa "$SA" pn 1 on key "$PORT$SA" "$RXH"; then
			ip link delete "$MACSCBUB_NAME"
		return `false`
		fi
	done
	if ! ip link set dev "$MACSCBUB_NAME" up || ! create_macscbpad_interface; then
		ip link delete "$MACSCBUB_NAME"
		return `false`
	fi
}



create_macsec_interface()
{
	local PORT=1
	local TXH="$TXKEY"
	local RXH="$RXKEY"

	# TODO: patch Wireshark to not be confused by icvlen < 16 anymore
	if ! ip link add link "$MACVLAN_NAME" name "$MACSEC_NAME" mtu "$MACSEC_MTU" type macsec icvlen 8 encrypt "$MACSEC_ENCRYPT" protect on end_station on send_sci off cipher "$MACSEC_CIPHER" \
	|| ! ip macsec add "$MACSEC_NAME" rx port 1 address "$REMOTE_MAC"; then
		return `false`
	fi
	ip link set dev "$MACSEC_NAME" group "$GROUP_ID" || true
	ip link set dev "$MACSEC_NAME" arp off || true
	ip link set dev "$MACSEC_NAME" multicast off || true
	ip link set dev "$MACSEC_NAME" alias "$LEVEL MACsec dedicated to $REMOTE_MAC peer" || true
	ip link set dev "$MACSEC_NAME" addrgenmode none || true
	for SA in 0 1 2 3; do
		TXH=`echo "$PORT$TXH$SA" | sha512sum | cut -c "-$MACSEC_HEXLEN"`
		RXH=`echo "$PORT$RXH$SA" | sha512sum | cut -c "-$MACSEC_HEXLEN"`
		if ! ip macsec add "$MACSEC_NAME" tx sa "$SA" pn 1 on key "$PORT$SA" "$TXH" \
		|| ! ip macsec add "$MACSEC_NAME" rx port 1 address "$REMOTE_MAC" sa "$SA" pn 1 on key "$PORT$SA" "$RXH"; then
			ip link delete "$MACSEC_NAME"
			return `false`
		fi
	done
	if ! ip link set dev "$MACSEC_NAME" master "$MACBR_NAME" \
	|| ! ip link set dev "$MACSEC_NAME" type bridge_slave isolated on; then
		return `false`
	fi
	ip link set dev "$MACSEC_NAME" type bridge_slave guard on || true
	ip link set dev "$MACSEC_NAME" type bridge_slave hairpin off || true
	bridge fdb add "$REMOTE_MAC" dev "$MACSEC_NAME" && ip link set dev "$MACSEC_NAME" type bridge_slave learning off
	ip link set dev "$MACSEC_NAME" type bridge_slave bcast_flood off || true
	ip link set dev "$MACSEC_NAME" type bridge_slave neigh_suppress off || true
	ebtables -t nat -A "$MACBR_NAME" -o "$MACSEC_NAME" -d Broadcast -j DROP
	if ! ip link set dev "$MACSEC_NAME" up; then
		ip link delete "$MACSEC_NAME"
		return `false`
	fi
}



random_mac_address()
{
	local OPTIONAL_RECURSION_LEVEL
	local REST
	local MAC

	if [ 1 -eq $# ]; then
		OPTIONAL_RECURSION_LEVEL="$1"
	else
		OPTIONAL_RECURSION_LEVEL=0
	fi

	set -o pipefail
	if REST=`dd if=/dev/urandom bs=5 count=1 status=none | xxd -p | sed 's/\(..\)/:\1/g'`; then
		MAC="02$REST"
	else
		MAC=`printf '02:%02x:%02x:%02x:%02x:%02x' $(( $RANDOM % 256 )) $(( $RANDOM % 256 )) $(( $RANDOM % 256 )) $(( $RANDOM % 256 )) $(( $RANDOM % 256 ))`
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



create_macvlan_interface()
{
	if ! ip link add link "$BASE_INTERFACE_NAME" name "$MACVLAN_NAME" address "$LOCAL_MAC" mtu "$MACVLAN_MTU" type macvlan mode source bcqueuelen 0 \
	|| ! ip link set link dev "$MACVLAN_NAME" type macvlan macaddr add "$REMOTE_MAC"; then
		return `false`
	fi
	ip link set dev "$MACVLAN_NAME" group "$GROUP_ID" || true
	ip link set dev "$MACVLAN_NAME" arp off || true
	ip link set dev "$MACVLAN_NAME" multicast off || true
	ip link set dev "$MACVLAN_NAME" alias "$LEVEL MACVLAN dedicated to $REMOTE_MAC peer" || true
	ip link set dev "$MACVLAN_NAME" addrgenmode none || true
	if ! create_macsec_interface; then
		ip link delete "$MACVLAN_NAME"
		return `false`
	fi
	if ! create_macscbub_interface; then
		ip link delete "$MACSEC_NAME"
		ip link delete "$MACVLAN_NAME"
		return `false`
	fi
	# Kernel bug workaround: MACVLAN interfaces refuse to share the same MAC
	# address even though it does not matter in source mode, but they don't
	# notice until they are brought up so we first create the MACsec ifaces
	# (that we cannot set the MAC address on due to yet another kernel bug)
	# so that they get the correct MAC address without having to specify it.
	ip link set dev "$MACVLAN_NAME" broadcast "$LOCAL_MAC"
	if ! ip link set dev "$MACVLAN_NAME" address `random_mac_address` \
	|| ! ip link set dev "$MACVLAN_NAME" up; then
		ip link delete "$MACSCBUB_NAME"
		ip link delete "$MACSEC_NAME"
		ip link delete "$MACVLAN_NAME"
		return `false`
	fi
}



create_bridge_if_needed()
{
	if [ ! -e "$SCN/$MACBR_NAME/bridge" ]; then
		if ! ip link add name "$MACBR_NAME" address "$LOCAL_MAC" mtu "$MACBR_MTU" type bridge; then
			return `false`
		fi
		ip link set dev "$MACBR_NAME" group "$GROUP_ID" || true
		ip link set dev "$MACBR_NAME" arp off || true
		ip link set dev "$MACBR_NAME" multicast off || true
		ip link set dev "$MACBR_NAME" alias "$LEVEL MACVLAN/MACsec bridge above $BASE_INTERFACE_NAME" || true
		ip link set dev "$MACBR_NAME" addrgenmode eui64 || true
		ip link set dev "$MACBR_NAME" type bridge no_linklocal_learn 1 || true
		ebtables -t nat -N "$MACBR_NAME" || true
		ebtables -t nat -A OUTPUT -j "$MACBR_NAME" --logical-out "$MACBR_NAME" || true
		if ! ip link set dev "$MACBR_NAME" up \
		|| ! batctl meshif "$BATMAN_NAME" interface add "$MACBR_NAME"; then
			ip link delete "$MACBR_NAME"
			return `false`
		fi
	fi
	create_macvlan_interface || return `false`
	REMOTE_EUI64=`mac_to_eui64 "$REMOTE_MAC"`
        REMOTE_LLA="fe80::$REMOTE_EUI64"
	ip neigh replace "$REMOTE_LLA" lladdr "$REMOTE_MAC" dev "$MACBR_NAME"
}



if [ $# -ne 6 ]; then
	>&2 echo "Usage: $0 u bat0 00:20:91:f8:e7:d6 0102030405060708091011121314151617181920212223242526272829303132 3231302928272625242322212019181716151413121110090807060504030201 on"
	>&2 echo "Usage: $0 l wlan0 04:f0:21:a6:b7:c8 01020304050607080910111213141516 16151413121110090807060504030201 off"
	exit 1
fi
export L_OR_U="$1"
export BASE_INTERFACE_NAME="$2"
export REMOTE_MAC="$3"
export TXKEY="$4"
export RXKEY="$5"
export MACSEC_ENCRYPT="$6"

case "$L_OR_U" in
	u)
		export LEVEL='Upper'
		export BATMAN_NAME='bat1'
		export MACBR_MTU=$(( $HOPEFULLY1500 + $BATMAN_OVERHEAD ))
		export MACSEC_MTU=$(( $HOPEFULLY1500 + $BATMAN_OVERHEAD ))
		export MACVLAN_MTU=$(( $HOPEFULLY1500 + $BATMAN_OVERHEAD + $MACSEC_OVERHEAD ))
		export MACSEC_HEXLEN=64
		export MACSEC_CIPHER='gcm-aes-256'
		;;
	l)
		export LEVEL='Lower'
		export BATMAN_NAME='bat0'
		export MACBR_MTU=$(( $HOPEFULLY1500 + $BATMAN_OVERHEAD + $MACSEC_OVERHEAD + $BATMAN_OVERHEAD ))
		export MACSEC_MTU=$(( $HOPEFULLY1500 + $BATMAN_OVERHEAD + $MACSEC_OVERHEAD + $BATMAN_OVERHEAD ))
		export MACVLAN_MTU=$(( $HOPEFULLY1500 + $BATMAN_OVERHEAD + $MACSEC_OVERHEAD + $BATMAN_OVERHEAD + $MACSEC_OVERHEAD ))
		export MACSEC_HEXLEN=32
		export MACSEC_CIPHER='gcm-aes-128'
		;;
	*)
		>&2 echo "Error: unknown level '$L_OR_U'"
		exit 2
esac
MAC_REGEX='[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}'
if ! grep -Eiqx "$MAC_REGEX" "$SCN/$BASE_INTERFACE_NAME/address"; then
	>&2 echo "Error: '$BASE_INTERFACE_NAME' does look like a usable interface"
	exit 3
fi
if echo "$REMOTE_MAC" | grep -Eiqx "$MAC_REGEX"; then
	REMOTE_MAC=`echo "$REMOTE_MAC" | tr '[:upper:]' '[:lower:]'`
else
	>&2 echo "Error: '$REMOTE_MAC' does not look like a MAC address"
	exit 4
fi
for KEY in "$TXKEY" "$RXKEY"; do
	if [ $MACSEC_HEXLEN -ne `echo "$KEY" | tr -d -c '[:xdigit:]' | wc -c` ]; then
		>&2 echo "Warning: '$KEY' must be $MACSEC_HEXLEN hexadecimal characters in $LEVEL MACsec"
	fi
done
export GROUP_ID=`cat "$SCN/$BASE_INTERFACE_NAME/ifindex"`
export LOCAL_MAC=`cat "$SCN/$BASE_INTERFACE_NAME/address"`
export MACBR_NAME=`echo "${L_OR_U}mb$LOCAL_MAC" | tr -d ':'`
export MACSEC_NAME=`echo "${L_OR_U}ms$REMOTE_MAC" | tr -d ':'`
export MACVLAN_NAME=`echo "${L_OR_U}mv$REMOTE_MAC" | tr -d ':'`
export MACSCBUB_NAME=`echo "${L_OR_U}mx$REMOTE_MAC" | tr -d ':'`
export MACSCBPAD_NAME=`echo "${L_OR_U}mp$REMOTE_MAC" | tr -d ':'`

# Workaround to use ebtables-legacy as "--logical-out" seems to be broken in ebtables-nft
if ! ebtables -V | grep -iq legacy; then
	! type ebtables-legacy >/dev/null 2>&1 || alias ebtables="ebtables-legacy"
fi

create_bridge_if_needed
exit $?
