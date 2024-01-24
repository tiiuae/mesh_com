#!/bin/sh


CBMA_DIR="/opt/mesh_com/modules/sc-mesh-secure-deployment/src/2_0/features/cbma"
DEBUG=0


###########################################################################

set -e

IPS="$@"


[ "$DEBUG" = "0" ] && unset DEBUG || DEBUG=/dev/fd/1


usage() {
    echo "Usage: $0 <ip1> <ip2> [<ip3> ...]" >&2
    exit 1
}

setup_ssh() {
    ip="$1"

    echo "[+] Setting up SSH configuration for ${ip}..."

    ln -fs /dev/fd/2 "/tmp/${ip}"

    echo y | ssh-keygen -q -N '' -f "/tmp/${ip}" 2>&1 >/dev/null | ssh-add - >/dev/null 2>&1

    ssh-copy-id -i "/tmp/${ip}.pub" "root@${ip}" >/dev/null 2>&1

    rm "/tmp/${ip}" "/tmp/${ip}.pub"

    echo "[+] SSH setup for ${ip} completed"
}

launch_cbma() {
    num="$1"
    ip="$2"

    echo -n "[+] Launching standalone CBMA on ${ip}... "

    command ssh -t -t "root@${ip}" sh <<- EOF > ${DEBUG:-/dev/null} 2>&1 &
        unset PS1

				cd "$CBMA_DIR"

				python3 standalone_cbma.py &

        pid=$!
        trap "kill $pid" INT KILL QUIT EXIT
        wait $pid
		EOF
    PROCS="${PROCS:+$PROCS }$!"

    echo "done"
}

handle_termination() {
    trap : INT KILL QUIT EXIT

    echo -ne "\n[!] Cleaning up processes... "

    for p in $PROCS; do
        kill -2 $p 2>/dev/null || :
    done
    echo "done"

    ssh-agent -k >/dev/null 2>&1

    echo "[!] Exiting"
    exit 0
}

wait_for_neighbors() {
    bat_neighbors="$1"

    for ip in $IPS; do
        echo -n "[+] Waiting for all neighbors in CBMA of ${ip}... "
        command ssh "root@${ip}" sh <<- EOF
						while batctl meshif bat0 n -H 2>/dev/null | awk -F'[[:space:]]*|s' '\$4>1{NR=0;exit}END{exit(NR=='${bat_neighbors}')}'; do
								sleep 1
						done
				EOF
        echo "done"
    done

    echo -n "[+] All good! :)"
}


[ $# -gt 1 ] || usage

trap handle_termination INT KILL QUIT EXIT

eval $(ssh-agent -s) >/dev/null

for ip in $IPS; do
    setup_ssh "$ip"
done

for ((n=1; $#; ++n)); do
    launch_cbma $n "$1"
    shift
done

sleep 3

wait_for_neighbors $(( n - 2 ))
