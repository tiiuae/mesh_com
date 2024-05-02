#! /bin/bash



export SCN='/sys/class/net'


cleanup_macscbpad_interface()
{
	ebtables -t nat -D "$MACBR_NAME" -o "$MACSCBPAD_NAME" -j dnat --to-destination "$REMOTE_MAC"
	ebtables -t nat -D PREROUTING -i "$MACSCBPAD_NAME" -j dnat --to-destination ff:ff:ff:ff:ff:ff
	ebtables -t nat -D "$MACBR_NAME" -o "$MACSCBPAD_NAME" -d '!' Broadcast -j DROP
	ip link delete "$MACSCBPAD_NAME"
}



# MACsec Single Channel Broadcast to unicast to broadcast (batadv workaround)
cleanup_macscbub_interface()
{
	cleanup_macscbpad_interface
	ip link delete "$MACSCBUB_NAME"
}



cleanup_macsec_interface()
{
	ebtables -t nat -D "$MACBR_NAME" -o "$MACSEC_NAME" -d Broadcast -j DROP
	ip link delete "$MACSEC_NAME"
}



cleanup_macvlan_interface()
{
	cleanup_macscbub_interface
	cleanup_macsec_interface
	ip link delete "$MACVLAN_NAME"
}



if [ $# -ne 3 ]; then
	>&2 echo "Usage: $0 u bat0 00:20:91:f8:e7:d6"
	>&2 echo "Usage: $0 l wlan0 04:f0:21:a6:b7:c8"
	exit 1
fi
export L_OR_U="$1"
export BASE_INTERFACE_NAME="$2"
export REMOTE_MAC="$3"

case "$L_OR_U" in
	u)
		export BATMAN_NAME='bat1'
		;;
	l)
		export BATMAN_NAME='bat0'
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

cleanup_macvlan_interface
exit $?
