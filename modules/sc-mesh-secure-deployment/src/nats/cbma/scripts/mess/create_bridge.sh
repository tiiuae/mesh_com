#!/bin/bash


source "${BASH_SOURCE%/*}/constants.rc"

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
        ebtables -t nat -N "$MACBR_NAME" -P DROP || true
        ebtables -t nat -A OUTPUT -j "$MACBR_NAME" --logical-out "$MACBR_NAME" || true
        if ! ip link set dev "$MACBR_NAME" up \
           || ! batctl meshif "$BATMAN_NAME" interface add "$MACBR_NAME"; then
            ip link delete "$MACBR_NAME"
            return `false`
        fi
    fi
}


if [ $# -ne 2 ]; then
    >&2 echo "Usage: $0 u bat0"
    >&2 echo "Usage: $0 l wlan0"
    exit 1
fi
export L_OR_U="$1"
export BASE_INTERFACE_NAME="$2"

case "$L_OR_U" in
    u)
        export LEVEL='Upper'
        export BATMAN_NAME='bat1'
        export MACBR_MTU=$(( $HOPEFULLY1500 + $BATMAN_OVERHEAD ))
        ;;
    l)
        export LEVEL='Lower'
        export BATMAN_NAME='bat0'
        export MACBR_MTU=$(( $HOPEFULLY1500 + $BATMAN_OVERHEAD + $MACSEC_OVERHEAD + $BATMAN_OVERHEAD ))
        ;;
    *)
        >&2 echo "Error: unknown level '$L_OR_U'"
        exit 2
esac
MAC_REGEX='[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}'
if ! grep -Eiqx "$MAC_REGEX" "$SCN/$BASE_INTERFACE_NAME/address"; then
    >&2 echo "Error: '$BASE_INTERFACE_NAME' does not look like a usable interface"
    exit 3
fi
export GROUP_ID=`cat "$SCN/$BASE_INTERFACE_NAME/ifindex"`
export LOCAL_MAC=`cat "$SCN/$BASE_INTERFACE_NAME/address"`
export MACBR_NAME=`echo "${L_OR_U}mb$LOCAL_MAC" | tr -d ':'`

# Workaround to use ebtables-legacy as "--logical-out" seems to be broken in ebtables-nft
if ! ebtables -V | grep -iq legacy; then
    ! type ebtables-legacy >/dev/null 2>&1 || alias ebtables="ebtables-legacy"
fi

create_bridge_if_needed
exit $?
