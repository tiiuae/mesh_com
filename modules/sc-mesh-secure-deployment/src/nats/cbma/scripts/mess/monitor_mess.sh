#! /bin/bash


SCN='/sys/class/net'
SECONDS_BETWEEN_ATTEMPTS=5
ATTEMPTS_BEFORE_GIVING_UP=3
KEY_REFRESH_TIMEOUT=60
PING_TIMEOUT=2



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



# Monitors the state of the MACsec interfaces by
# - Doing batman ping to monitor the MS interface
# - Checking the network usage statistics of the MX interface
# Also exits with code 255 if MACsec keys have to be refreshed
monitoring_loop()
{
	REMOTE_EUI64="$(mac_to_eui64 "$REMOTE_MAC")"
	REMOTE_LLA="fe80::$REMOTE_EUI64"
	MACBR_NAME="${L_OR_U}mb${LOCAL_MAC//:/}"
	MACSCBUB_NAME="${L_OR_U}mx${REMOTE_MAC//:/}"
	FAILURES=0

	KEY_REFRESH_ELAPSED=0
	while sleep $SECONDS_BETWEEN_ATTEMPTS; do
		read -r NEW_STATS < "${SCN}/${MACSCBUB_NAME}/statistics/rx_packets"
		if [[ "$OLD_STATS" != "$NEW_STATS" ]] \
				 && ip neigh replace "$REMOTE_LLA" lladdr "$REMOTE_MAC" dev "$MACBR_NAME" \
				 && ping -W "$PING_TIMEOUT" -c 1 "$REMOTE_LLA%$MACBR_NAME"
		then
			[[ $(( KEY_REFRESH_ELAPSED+=SECONDS_BETWEEN_ATTEMPTS )) -ne $KEY_REFRESH_TIMEOUT ]] || return 255

			OLD_STATS="$NEW_STATS"
			FAILURES=0
		else
			[[ $(( ++FAILURES )) -lt $ATTEMPTS_BEFORE_GIVING_UP ]] || return `false`
		fi
	done
}



if [[ $# -ne 3 ]]; then
	>&2 echo "Usage: $0 u bat0 00:20:91:f8:e7:d6"
	>&2 echo "Usage: $0 l wlan0 04:f0:21:a6:b7:c8"
	exit 1
fi
L_OR_U="${1@L}"
BASE_INTERFACE_NAME="$2"
REMOTE_MAC="${3@L}"

case "$L_OR_U" in
	u)
		BATMAN_NAME='bat1'
		;;
	l)
		BATMAN_NAME='bat0'
		;;
	*)
		>&2 echo "Error: unknown level '$L_OR_U'"
		exit 2
esac
LOCAL_MAC="$(< "$SCN/$BASE_INTERFACE_NAME/address")"
if [[ ! $LOCAL_MAC =~ ^([0-9a-f]{2}:){5}[0-9a-f]{2}$ ]]; then
	>&2 echo "Error: '$BASE_INTERFACE_NAME' does not look like a usable interface"
	exit 3
fi
if [[ ! $REMOTE_MAC =~ ^([0-9a-f]{2}:){5}[0-9a-f]{2}$ ]]; then
	>&2 echo "Error: '$REMOTE_MAC' does not look like a MAC address"
	exit 4
fi
monitoring_loop
exit $?
