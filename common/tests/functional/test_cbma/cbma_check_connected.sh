#!/bin/sh

TIMEOUT_SECONDS=10


if [ $# -eq 0 ]; then
    echo "Usage: $0 <bat-iface> [<peer-mac>]"
    exit 1
fi

check_dependencies() {
    for t in batctl; do
        if ! type $t >/dev/null 2>&1; then
            echo "[!] FATAL: '$t' is missing!" >&2
            exit 2
        fi
    done
}

check_interface() {
    iface="$1"

    if [ ! -e "/sys/class/net/${iface}/address" ]; then
        echo "'$iface' does not exist" >&2
        exit 3
    fi
}

check_mac_address() {
    mac="$1"

    if [ -n "$mac" ] && ! echo "$mac" | grep -Eiqx '([0-9a-f]{2}:){5}[0-9a-f]{2}'; then
	      echo "'$mac' is not a valid MAC address" >&2
	      exit 4
    fi
}

cbma_check_connected() {
    iface="$1"
    peer_mac="${2:-.}"

    elapsed_seconds=0
    while [ $((elapsed_seconds++)) -lt $TIMEOUT_SECONDS ]; do
        peers="$(batctl meshif $iface n -H -n)"
        if expr "$peers" : "$peer_mac" 2>/dev/null >&2; then
            echo 'Pass'
            return
        fi
        sleep 1
    done
    echo 'Fail'
    exit 5
}

check_dependencies
check_interface $1
check_mac_address $2
cbma_check_connected $@
