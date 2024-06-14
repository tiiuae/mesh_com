#!/bin/sh

BATMAN_ETHER_PROTO='0x4305'
MACSEC_ETHER_PROTO='0x88e5'
MACSEC_PACKET_BYTES_OFFSET=36
TIMEOUT_SECONDS=10


if [ $# -eq 0 ]; then
    echo "Usage: $0 <non-bat-iface> [<peer-mac>]"
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

check_mac_address() {
    mac="$1"

    if [ -n "$mac" ] && ! echo "$mac" | grep -Eiqx '([0-9a-f]{2}:){5}[0-9a-f]{2}'; then
	      echo "'$mac' is not a valid MAC address" >&2
	      exit 4
    fi
}

cbma_check_secured() {
    iface="$1"
    peer_mac="$2"

    if [ -n "$peer_mac" ]; then
        capture_iface="lms$(echo $peer_mac | tr -d ':')"
    else
        capture_iface="lmb$(tr -d ':' < /sys/class/net/"$iface"/address)"
    fi
    if ! ( check_interface "$capture_iface" 2>/dev/null ); then
        echo "'$peer_mac' peer is not connected"
        exit 5
    fi

    packet="$(timeout $TIMEOUT_SECONDS tcpdump -c1 -i "$capture_iface" -qQ out ether proto $BATMAN_ETHER_PROTO and ether[$MACSEC_PACKET_BYTES_OFFSET:2] == $MACSEC_ETHER_PROTO 2>/dev/null)"

    if [ -z "$packet" ]; then
        echo 'Fail'
        exit 6
    fi
    echo 'Pass'
}

check_dependencies
check_interface $1
check_mac_address $2
cbma_check_secured $@
