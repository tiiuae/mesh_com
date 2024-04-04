#!/bin/sh

MULTICAST_PORT=15002
MULTICAST_PREFIX='ff33'
TIMEOUT_SECONDS=10


if [ $# -eq 0 ]; then
    echo "Usage: $0 <iface> [<port>]"
    exit 1
fi

check_dependencies() {
    for t in tcpdump timeout; do
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
    case "$iface" in bat*) MULTICAST_PORT=$((MULTICAST_PORT + 1));; esac
}

cbma_check_running() {
    iface="$1"
    port="${2:-$MULTICAST_PORT}"

    packet="$(timeout $TIMEOUT_SECONDS tcpdump -c1 -i "$iface" -qQ out udp and port "$port" 2>/dev/null)"
    mcast_group="$(expr "$packet" : '.*[[:space:]]\([^.]\+\).[0-9]\+:')"

    if [ "${mcast_group%%:*}" != 'ff33' ]; then
        echo 'Fail'
        exit 4
    fi
    echo 'Pass'
}

check_dependencies
check_interface $1
cbma_check_running $@
