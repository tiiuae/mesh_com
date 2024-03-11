#! /bin/bash



TMP=`mktemp -d /tmp/check_traffic.XXXXXXXXXX`



collect_stats()
{
	local DEV="$1"

	cp "/sys/class/net/$DEV/statistics/rx_packets" "$TMP/$DEV"
	for I in /sys/class/net/$DEV/lower_*; do
		if [ "$I" = "/sys/class/net/$DEV/lower_*" ]; then
			break
		fi
		local LOWER_DEV=`basename "$I" | cut -d _ -f 2-`
		collect_stats "$LOWER_DEV"
	done
}



compare_stats()
{
	local DEV="$1"

	if [ 2 -eq $# ]; then
		local PREVIOUS="$2/"
	else
		local PREVIOUS=''
	fi
	BEFORE=`cat "$TMP/$DEV"`
	AFTER=`cat "/sys/class/net/$DEV/statistics/rx_packets"`
	if [ $BEFORE -eq $AFTER ]; then
		if [ 0 -eq $AFTER ]; then
			local COLOR=31
		else
			local COLOR=33
		fi
	else
		local COLOR=32
	fi
	for I in /sys/class/net/$DEV/lower_*; do
		if [ "$I" = "/sys/class/net/$DEV/lower_*" ]; then
			echo -e "$PREVIOUS\033[$COLOR;1m$DEV\033[0m"
			break
		fi
		local LOWER_DEV=`basename "$I" | cut -d _ -f 2-`
		compare_stats "$LOWER_DEV" "$PREVIOUS\033[$COLOR;1m$DEV\033[0m"
	done
}


for I in 0 1; do
	batctl meshif bat$I n
done
collect_stats 'br-lan'
collect_stats 'bat0'
arping -I br-lan -c 3 -w 3 -S 0.0.0.0 -q 0.0.0.0 &
arping -I bat0 -c 3 -w 3 -S 0.0.0.0 -q 0.0.0.0 &
for I in 3 2 1; do
	echo -n "$I... "
	sleep 1
done
echo ''
compare_stats 'br-lan' > "$TMP/report"
cat "$TMP/report"
grep -Fq bat0 "$TMP/report" || compare_stats 'bat0'
echo "$TMP" | grep -Eq '^/tmp/' && rm -r "$TMP"
