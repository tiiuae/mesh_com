#!/bin/bash


source "${BASH_SOURCE%/*}/constants.rc"

cleanup_bridge_if_needed()
{
    for I in $SCN/$MACBR_NAME/lower_*; do
        if [ "$I" = "$SCN/$MACBR_NAME/lower_*" ]; then
            ebtables -t nat -D OUTPUT -j "$MACBR_NAME" --logical-out "$MACBR_NAME"
            ebtables -t nat -X "$MACBR_NAME"
            ip link delete "$MACBR_NAME"
        fi
        break
    done
}

if [ $# -ne 2 ]; then
    >&2 echo "Usage: $0 u bat0"
    >&2 echo "Usage: $0 l wlan0"
    exit 1
fi
export L_OR_U="$1"
export BASE_INTERFACE_NAME="$2"

MAC_REGEX='[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}'
if ! grep -Eiqx "$MAC_REGEX" "$SCN/$BASE_INTERFACE_NAME/address"; then
    >&2 echo "Error: '$BASE_INTERFACE_NAME' does not look like a usable interface"
    exit 2
fi
export LOCAL_MAC=`cat "$SCN/$BASE_INTERFACE_NAME/address"`
export MACBR_NAME=`echo "${L_OR_U}mb$LOCAL_MAC" | tr -d ':'`

# Workaround to use ebtables-legacy as "--logical-out" seems to be broken in ebtables-nft
if ! ebtables -V | grep -iq legacy; then
    ! type ebtables-legacy >/dev/null 2>&1 || alias ebtables="ebtables-legacy"
fi

cleanup_bridge_if_needed
exit $?
