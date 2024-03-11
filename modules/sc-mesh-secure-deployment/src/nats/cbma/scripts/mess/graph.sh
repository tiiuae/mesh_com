#! /bin/bash



we_need_to_go_deeper()
{
	local DEV="$1"

	ALIAS=`cat "/sys/class/net/$DEV/ifalias"`
	echo "\"$DEV\" [label=\"$DEV\\n$ALIAS\"]"
	for I in /sys/class/net/$DEV/lower_*; do
		if [ "$I" = "/sys/class/net/$DEV/lower_*" ]; then
			break
		fi
		local LOWER_DEV=`basename "$I" | cut -d _ -f 2-`
		echo "	\"$DEV\" -> \"$LOWER_DEV\""
		we_need_to_go_deeper "$LOWER_DEV"
	done
}



graphviz_syntax() {
	echo 'digraph {'
	we_need_to_go_deeper 'br-lan' | sort -u
	echo '}'
}



graphviz_syntax | dot -Tsvg > /tmp/totem.svg
