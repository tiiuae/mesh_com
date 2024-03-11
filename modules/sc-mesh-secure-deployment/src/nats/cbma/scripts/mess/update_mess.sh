#!/bin/bash


SCN='/sys/class/net'


update_macsec_keys() {
    local MACSEC_NAME="${L_OR_U}ms${REMOTE_MAC//:/}"

    [[ -d "$SCN/$MACSEC_NAME" ]] || return 4

    local PORT=0
    local SA
	  for SA in {0..3}; do
		    TXH="$(sha512sum <<< "$PORT$TXH$SA" | cut -c "-$MACSEC_HEXLEN")"
		    RXH="$(sha512sum <<< "$PORT$RXH$SA" | cut -c "-$MACSEC_HEXLEN")"
        if ! ip macsec set "$MACSEC_NAME" tx sa "$SA" off || \
           ! ip macsec set "$MACSEC_NAME" rx port 1 address "$REMOTE_MAC" sa "$SA" off || \
           ! ip macsec del "$MACSEC_NAME" tx sa "$SA" || \
           ! ip macsec del "$MACSEC_NAME" rx port 1 address "$REMOTE_MAC" sa "$SA" || \
           ! ip macsec add "$MACSEC_NAME" tx sa "$SA" pn 1 on key "$PORT$SA" "$TXH" || \
           ! ip macsec add "$MACSEC_NAME" rx port 1 address "$REMOTE_MAC" sa "$SA" pn 1 on key "$PORT$SA" "$RXH"
        then
            ip link delete "$MACSEC_NAME"
            return 5
        fi
    done
}


update_macscbub_keys() {
    local MACSCBUB_NAME="${L_OR_U}mx${REMOTE_MAC//:/}"

    [[ -d "$SCN/$MACSCBUB_NAME" ]] || return 6

	  local SCB_RXSCI="${REMOTE_MAC//:/}0000"
    local PORT=0
    local SA
	  for SA in {0..3}; do
		    TXH="$(sha512sum <<< "$PORT$TXH$SA" | cut -c "-$MACSEC_HEXLEN")"
		    RXH="$(sha512sum <<< "$PORT$RXH$SA" | cut -c "-$MACSEC_HEXLEN")"
        if ! ip macsec set "$MACSCBUB_NAME" tx sa "$SA" off || \
           ! ip macsec set "$MACSCBUB_NAME" rx sci "$SCB_RXSCI" sa "$SA" off || \
           ! ip macsec del "$MACSCBUB_NAME" tx sa "$SA" || \
           ! ip macsec del "$MACSCBUB_NAME" rx sci "$SCB_RXSCI" sa "$SA" || \
           ! ip macsec add "$MACSCBUB_NAME" tx sa "$SA" pn 1 on key "$PORT$SA" "$TXH" || \
           ! ip macsec add "$MACSCBUB_NAME" rx sci "$SCB_RXSCI" sa "$SA" pn 1 on key "$PORT$SA" "$RXH"
        then
            ip link delete "$MACSCBUB_NAME"
            return 7
        fi
    done
}



if [[ $# -ne 4 ]]; then
    >&2 echo "Usage: $0 u 00:20:91:f8:e7:d6 0102030405060708091011121314151617181920212223242526272829303132 3231302928272625242322212019181716151413121110090807060504030201"
    >&2 echo "Usage: $0 l 04:f0:21:a6:b7:c8 01020304050607080910111213141516 16151413121110090807060504030201"
    exit 1
fi

L_OR_U="$1"
REMOTE_MAC="$2"
TXKEY="$3"
RXKEY="$4"

case "$L_OR_U" in
    u) MACSEC_HEXLEN=64 ;;
    l) MACSEC_HEXLEN=32 ;;
    *) echo "Error: unknown level '$L_OR_U'" >&2
       exit 2
esac

if [[ ! $REMOTE_MAC =~ ^([0-9a-f]{2}:){5}[0-9a-f]{2}$ ]]; then
    echo "Error: '$REMOTE_MAC' does not look like a MAC address" >&2
    exit 3
fi

update_macsec_keys && \
update_macscbub_keys

exit $?
