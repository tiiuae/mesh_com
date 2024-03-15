#! /bin/bash


SCN='/sys/class/net'
SECONDS_BETWEEN_ATTEMPTS=5
ATTEMPTS_BEFORE_GIVING_UP=3
KEY_REFRESH_TIMEOUT=60
PING_TIMEOUT=2

# Monitors the state of the MACsec interfaces by
# - Doing batman ping to monitor the MS interface
# - Checking the network usage statistics of the MX interface
# Also exits with code 255 if MACsec keys have to be refreshed
monitoring_loop()
{
	MACSCBUB_NAME="${L_OR_U}mx${REMOTE_MAC//:/}"
	FAILURES=0

	KEY_REFRESH_ELAPSED=0
	while sleep $SECONDS_BETWEEN_ATTEMPTS; do
		read -r NEW_STATS < "${SCN}/${MACSCBUB_NAME}/statistics/rx_packets"
		if [[ "$OLD_STATS" != "$NEW_STATS" ]] && \
				 batctl meshif "$BATMAN_NAME" ping -c 1 -t "$PING_TIMEOUT" "$REMOTE_MAC"
		then
			[[ $(( KEY_REFRESH_ELAPSED+=SECONDS_BETWEEN_ATTEMPTS )) -ne $KEY_REFRESH_TIMEOUT ]] || return 255

			OLD_STATS="$NEW_STATS"
			FAILURES=0
		else
			[[ $(( ++FAILURES )) -lt $ATTEMPTS_BEFORE_GIVING_UP ]] || return `false`
		fi
	done
}


if [[ $# -ne 2 ]]; then
	>&2 echo "Usage: $0 u 00:20:91:f8:e7:d6"
	>&2 echo "Usage: $0 l 04:f0:21:a6:b7:c8"
	exit 1
fi
L_OR_U="${1@L}"
REMOTE_MAC="${2@L}"

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
if [[ ! $REMOTE_MAC =~ ^([0-9a-f]{2}:){5}[0-9a-f]{2}$ ]]; then
	>&2 echo "Error: '$REMOTE_MAC' does not look like a MAC address"
	exit 4
fi
monitoring_loop
exit $?
