#!/bin/bash


NUM_NODES=2

KEYPAIR_TYPE="rsa"    # Can be ecdsa, eddsa, or rsa

DEFAULT_LOG_LEVEL="INFO"

CONSTANTS_RC="mess/constants.rc"

CBMA_DEBUG=0
BAT_DEBUG=0

EXIT_AFTER_TESTS=1


#####################################

[ "$USER" = "root" ] || { echo "[!] This script must be run as root" 1>&2; exit 1; }
[ "$CBMA_DEBUG" = "0" ] && unset CBMA_DEBUG || CBMA_DEBUG=/dev/fd/1
[ "$BAT_DEBUG" = "0" ] && unset BAT_DEBUG || BAT_DEBUG=/dev/fd/1

LOG_LEVEL="${LOG_LEVEL:-$DEFAULT_LOG_LEVEL}"

NODES=$(awk -vA=$(printf "%d" \'A) -vc=$NUM_NODES 'BEGIN{while(n<c)printf "%c\n",A + n++}')
TOTAL_NUM_INTERFACES=0


check_dependencies() {
    echo -n "[+] Checking dependencies... "

    if reqs="$(python3 -m pip freeze -r requirements.txt 2>&1 | grep -i 'not installed')"; then
        printf "\n[!] FATAL: install all requirements in requirements.txt!\n$reqs\n"
        exit 1
    fi

    for t in ebtables batctl faketime ip iw sed grep awk modprobe killall; do
        if ! type $t >/dev/null 2>&1; then
            printf "\n[!] FATAL: '$t' is missing!\n"
            exit 1
        fi
    done
    echo "Done"
}


check_max_nest_dev() {
    I="max_nest_dev"

    echo -n "[+] Getting maximum number of interfaces that can be chained together... "

    ip netns delete "$I" 2>/dev/null
    sleep 1
    ip netns add "$I"

    output="$(ip netns exec "$I" bash scripts/mess/max_nest_dev.sh 2>&1)"
    max_nest_dev="$(expr "$output" : "\([0-9]\+\)")"

    if [ -z "$max_nest_dev" ]; then
        max_nest_dev="$(expr "$output" : ".*"$'\n'"\([0-9]\+\)")"
        message="$(expr "$output" : "\(.*"$'\n'"\)${max_nest_dev}")"
    fi
    message="${message}$(expr "$output" : "${message}${max_nest_dev}"$'\n'"\(.*\)")"

    echo "$max_nest_dev"
    printf "${message}${message:+\n}" | sed 's/^.\+$/[!] &/'

    START_UPPER_BATMAN=$(( max_nest_dev >= 11 ))

    ip netns delete "$I"
}


wait_until_interface_ready() {
    I="$1"
    IFACE="$2"

    addr="$(ip netns exec "$I" ip addr show dev "${IFACE}" | awk '/inet6/{print $2}')"
    while ! ip netns exec "$I" \
            python3 -c "import socket; socket.create_server(('${addr%/*}', 0, 0, socket.if_nametoindex('${IFACE}')), family=socket.AF_INET6)" >/dev/null 2>&1
    do
        sleep 1
    done
}


get_base_mtu_from_constants_rc() (
    DIR="${1:-.}"

    . "${DIR}/${CONSTANTS_RC}"
    echo $HOPEFULLY1500
)


get_mtu_overhead_from_constants_rc() (
    DIR="${1:-.}"

    . "${DIR}/${CONSTANTS_RC}"
    echo $((MACSEC_OVERHEAD + BATMAN_OVERHEAD))
)


setup_wlan() {
    I="$1"
    WLAN="$2"

    TOTAL_NUM_INTERFACES=$((TOTAL_NUM_INTERFACES + 1))

    read PHY < "/sys/class/net/$WLAN/phy80211/name"

    iw dev "$WLAN" del 2>/dev/null
    iw phy "$PHY" interface add "$WLAN" type mesh

    iw phy "$PHY" set netns name "$I"

    ip netns exec "$I" iw dev "$WLAN" interface add "wlp1s${I}" type mesh
    ip netns exec "$I" iw dev "$WLAN" del

    # NOTE: Deleting the WLAN iface is a workaround as this doesn't work:
    # ip netns exec "$I" ip link set dev "$WLAN" name "wlp1s${I}"
    # ip netns exec "$I" iw dev "wlp1s${I}" set type mesh

    ip netns exec "$I" ip link set dev "wlp1s${I}" mtu $((BASE_MTU + MTU_OVERHEAD + \
                                                                     MTU_OVERHEAD * START_UPPER_BATMAN))

    ip netns exec "$I" ip link set dev "wlp1s${I}" address "00:20:91:0${I}:0${I}:0${I}"
    ip netns exec "$I" ip link set dev "wlp1s${I}" up
    ip netns exec "$I" iw dev "wlp1s${I}" mesh join gold freq 2412 HT40+

    wait_until_interface_ready "$I" "wlp1s${I}"
}


setup_eth() {
    I="$1"
    ETH="$2"

    TOTAL_NUM_INTERFACES=$((TOTAL_NUM_INTERFACES + 1))

    ip link del "$ETH" 2>/dev/null

    ip link add "$ETH" type veth peer name "eth${I}" netns "$I"

    ip netns exec "$I" ip link set dev "eth${I}" mtu $((BASE_MTU + MTU_OVERHEAD + \
                                                                   MTU_OVERHEAD * START_UPPER_BATMAN))
    ip netns exec "$I" ip link set dev "eth${I}" address "00:20:91:${I}0:${I}0:${I}0"

    ip link set "$ETH" up
    ip netns exec "$I" ip link set "eth${I}" up

    ip link set "$ETH" master vbr0

    wait_until_interface_ready "$I" "eth${I}"
}


setup_bat() {
    I="$1"
    BAT="$2"
    IFACE="$3"

    ip netns exec "$I" ip link add "$BAT" type batadv

    if [ -n "$IFACE" ]; then
        MAC="$(a="$(ip netns exec "$I" cat "/sys/class/net/$IFACE/address")" && \
                    printf "%02x${a#??}\n" $(( 0x${a%%:*} ^ 0x2 )))"
        ip netns exec "$I" ip link set dev "$BAT" address "$MAC"
    fi

    if [ $START_UPPER_BATMAN -eq 1 -a "$BAT" = "bat0" ]; then
        ip netns exec "$I" ip link set dev "$BAT" mtu $(( BASE_MTU + MTU_OVERHEAD )) 2>/dev/null
    fi

    ip netns exec "$I" ip link set "$BAT" up

    wait_until_interface_ready "$I" "$BAT"
}


setup_nodes() {
    echo -n "[+] Setting up nodes... "

    modprobe -r mac80211_hwsim 2>/dev/null
    sleep 1
    modprobe mac80211_hwsim radios="$NUM_NODES"

    ip link del vbr0 2>/dev/null

    ip link add vbr0 type bridge
    ip link set vbr0 up

    N=0
    for I in $NODES; do
        ip netns delete "$I" 2>/dev/null
        sleep 1
        ip netns add "$I"

        setup_wlan "$I" "wlan${N}"
        setup_eth "$I" "veth${N}"

        setup_bat "$I" "bat0" "wlp1s${I}"
        [ $START_UPPER_BATMAN -eq 0 ] || (
            MTU_OVERHEAD=0
            setup_bat "$I" "bat1"
        )

        N=$((N + 1))
    done

    echo "Done"
}


destroy_nodes() {
    echo -n "[+] Destroying nodes... "

    N=0
    for I in $NODES; do
        ip netns delete "$I"
        ip link del "veth${N}"
        N=$((N + 1))
    done 2>/dev/null

    ip link del vbr0

    modprobe -r mac80211_hwsim

    echo "Done"
}


generate_certificates() {
    DIR="${1:-.}"

    echo -n "[+] Generating certificates... "

    (
        export KEYPAIR_TYPE
        for I in $NODES; do
            export CERTIFICATE_FOLDERS="$DIR/certificates$I"
            INTERFACES="$(ip netns exec "$I" sh -c 'exec awk -F/ "{\$0=FILENAME;print \$5}" /sys/class/net/*/address')"
            ip netns exec "$I" bash "$DIR/generate_certificates.sh" $INTERFACES
        done
    ) >/dev/null 2>&1

    echo "Done"
}


remove_certificates() {
    DIR="${1:-.}"

    echo -n "[+] Removing certificates... "

    for I in $NODES; do
        rm -r "$DIR/certificates$I"
    done

    echo "Done"
}


PROCS=""

launch_lower_cbma() {
    DIR="${1:-.}"
    echo -n "[+] Launching lower CBMA... "

    rm -rf logs

    export LOG_LEVEL="$LOG_LEVEL"
    for I in $NODES; do
        export LOG_DIR="logs/logs$I"
        ip netns exec "$I" \
           python3 standalone.py -c "$DIR/certificates$I/$KEYPAIR_TYPE" \
                                 > ${CBMA_DEBUG:-/dev/null} 2>&1 &
        PROCS="${PROCS:+$PROCS }$!"
    done

    echo "Done"
}


launch_upper_cbma() {
    DIR="${1:-.}"
    echo -n "[+] Launching upper CBMA... "

    rm -rf logs

    export LOG_LEVEL="$LOG_LEVEL"
    for I in $NODES; do
        export LOG_DIR="logs/logs$I"
        ip netns exec "$I" \
           python3 standalone.py -i bat0 \
                                 -b bat1 \
                                 -u \
                                 -c "$DIR/certificates$I/$KEYPAIR_TYPE" \
                                 > ${CBMA_DEBUG:-/dev/null} 2>&1 &
        PROCS="${PROCS:+$PROCS }$!"
    done

    echo "Done"
}


wait_for_batman_neighbors() {
    BAT_IFACE="$1"

    if [ "$BAT_IFACE" = "bat0" ]; then
        bat_neighbors=$(( TOTAL_NUM_INTERFACES - TOTAL_NUM_INTERFACES / NUM_NODES ))
    else
        bat_neighbors=$(( NUM_NODES - 1 ))
    fi

    for I in $NODES; do
        printf "[+] Waiting for ${BAT_IFACE} neighbors in CBMA node ${I}... ${CBMA_DEBUG:+\n}${BAT_DEBUG:+\n}"
        ip netns exec "$I" sh > ${BAT_DEBUG:-/dev/null} 2>&1 <<- EOF
						while batctl meshif ${BAT_IFACE} n -H 2>/dev/null | awk -F'[[:space:]]*|s' '{print}\$4>1{NR=0;exit}END{exit(NR=='${bat_neighbors}')}'; do
								sleep 1
						done
				EOF
        if [ -n "$CBMA_DEBUG" -o -n "$BAT_DEBUG" ]; then
            echo
        else
            echo "Done"
        fi
    done
}


interrupt_handler() {
    RET=$?

    trap : INT EXIT QUIT KILL

    for P in $PROCS; do
        killall -n $P python3 2>/dev/null
    done

    sleep 1

    echo

    ip netns exec "max_nest_dev" 2>/dev/null

    destroy_nodes

    remove_certificates "$DIR"

    chmod -R a+rw logs

    echo "[+] Exiting"

    exit $RET
}


check_dependencies

(
    set +e

    DIR="$(dirname $0)"
    BASE_MTU="$(get_base_mtu_from_constants_rc "$DIR")"
    MTU_OVERHEAD="$(get_mtu_overhead_from_constants_rc "$DIR")"

    trap interrupt_handler INT EXIT QUIT KILL

    check_max_nest_dev

    . "${0%/*}/${CONSTANTS_RC}"
    MTU_OVERHEAD=$((MACSEC_OVERHEAD + BATMAN_OVERHEAD))

    setup_nodes

    generate_certificates "$DIR"

    launch_lower_cbma "$DIR"
    wait_for_batman_neighbors bat0

    if [ $START_UPPER_BATMAN -eq 1 ]; then
        launch_upper_cbma "$DIR"
        wait_for_batman_neighbors bat1
    fi

    echo -n "[+] All good! :)"

    if [ "$EXIT_AFTER_TESTS" = "0" ]; then
        printf "\n[+] Waiting for ${PROCS%% *}...\n\n"
        wait "${PROCS%% *}"
    fi
)

exit $?
