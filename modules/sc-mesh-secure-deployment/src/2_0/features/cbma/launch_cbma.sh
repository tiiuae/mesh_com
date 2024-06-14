#! /bin/bash -x


[ $EUID -ne 0 ] && { echo "[!] This script must be run as root" 1>&2; exit 1; }


NODES=`echo {A..C}`


check_dependencies() {
    echo -n "[+] Checking dependencies... "

    if ! python3 -c 'import OpenSSL' &>/dev/null; then
        echo -e "\n[!] FATAL: PyOpenSSL is missing!"
        exit 1
    fi

    for t in brctl ebtables batctl; do
        if ! type $t &>/dev/null; then
            echo -e "\n[!] FATAL: ebtables is missing!"
            exit 1
        fi
    done
    echo "Done"
}

setup_nodes() {
    echo -n "[+] Setting up nodes... "

    modprobe -r mac80211_hwsim 2>/dev/null
    sleep 1
    modprobe mac80211_hwsim radios=$(wc -w <<< "$NODES")

	  N=0
	  for I in $NODES; do
		    ip netns delete "$I" 2>/dev/null
		    sleep 1
		    ip netns add "$I"

        # TODO - Needed?
        # ip netns exec "$I" sysctl -q -w net.ipv6.conf.all.forwarding=1


		    PHY=`echo /sys/class/ieee80211/*/device/net/wlan$N | cut -d / -f 5`

	      iw dev "wlan$N" del 2>/dev/null
	      iw phy "$PHY" interface add "wlan$N" type mesh

		    sleep 1
		    iw phy "$PHY" set netns name "$I"

        ip netns exec "$I" iw dev "wlan$N" interface add "wlp1s0" type mesh
        ip netns exec "$I" ifconfig "wlp1s0" mtu 1560

		    sleep 1
		    ip netns exec "$I" ip link set dev "wlp1s0" address "00:20:91:0$I:0$I:0$I"
		    ip netns exec "$I" ip link set dev "wlp1s0" up
		    ip netns exec "$I" iw dev "wlp1s0" mesh join gold freq 2412

        ip netns exec "$I" iptables -P FORWARD ACCEPT
        ip netns exec "$I" ip6tables -P FORWARD ACCEPT

        # TODO - Finish setup for the eth1 interface
        ip link add "veth$N" type veth peer name "veth$N-ns"
        ip link set "veth$N-ns" netns "$I"

        ip netns exec "$I" ip link set "veth$N-ns" down
        ip netns exec "$I" ip link set "veth$N-ns" name eth1

		    ip netns exec "$I" ip link set dev eth1 address "00:20:91:${I}0:${I}0:${I}0"

        # TODO - IPv6
        # ip addr add "10.1.1.$(($N + 1))/24" dev "veth$N"
        # ip netns exec "$I" ip addr add "10.1.1.$(($N + 2))/24" dev eth1

        ip link set "veth$N" up
        ip netns exec "$I" ip link set eth1 up

        # ip netns exec "$I" ip route add default via "10.1.1.$(($N + 1))" dev eth1
        ip netns exec "$I" ip route add default dev eth1

		    N=$(( $N + 1 ))
	  done
    # TODO - Needed?
    # ip link set hwsim0 up

    echo "Done"
}

destroy_nodes() {
    echo -n "[+] Destroying nodes... "

	  N=0
	  for I in $NODES; do
		    ip netns delete "$I" 2>/dev/null
        ip link del "veth$N"
		    N=$(( $N + 1 ))
	  done

    modprobe -r mac80211_hwsim

    echo "Done"
}

PROCS=""

create_certificates() {
    echo -n "[+] Creating certificates... "

    (
        cd cert_generation/

	      for I in $NODES; do
            mkdir -p "certificates$I"
            cd "certificates$I"

            for D in wlp1s0 eth1; do
                mac=$(ip netns exec "$I" sh -c "tr -d : < '/sys/class/net/$D/address'")

                ip netns exec "$I" ../generate-csr.sh "$D"

                ../generate_certificates.sh "macsec_${mac}.csr" "certificates"

                cp "macsec_${mac}.key" certificates
            done

            cd ..
        done
    )

    echo "Done"
}

remove_certificates() {
    echo -n "[+] Removing certificates... "

	  for I in $NODES; do
        rm -r "cert_generation/certificates$I"
    done

    echo "Done"
}

launch_cbma() {
    echo -n "[+] Launching CBMA... "

	  for I in $NODES; do
        PROCS="${PROCS:+$PROCS }$!"
        ip netns exec "$I" python3 setup_cbma.py --certdir "cert_generation/certificates$I/certificates/" &
    done
    P=$!

    echo "Done"

    tail -f --pid=$P -q &>/dev/null
}

interrupt_handler() {
    trap : INT EXIT QUIT KILL

    for P in $PROCS; do
        killall -n $P python3 2>/dev/null
    done

    echo

    destroy_nodes

    remove_certificates

    exit $?
}


check_dependencies

trap interrupt_handler INT EXIT QUIT KILL

setup_nodes

create_certificates

launch_cbma
