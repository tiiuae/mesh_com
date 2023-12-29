#!/bin/sh


MDM_AGENT_DIR="/opt/mesh_com/modules/sc-mesh-secure-deployment/src/nats"
MDM_AGENT_CERTS="${MDM_AGENT_DIR}/certs"

MDM_SERVER_DIR="~/mdm-server"
MDM_SERVER_BIND_IP="0.0.0.0"
MDM_SERVER_PORT="5000"

LAUNCH_MDM_SERVER=0
DEBUG=0


###########################################################################

set -e

MDM_SERVER_IP="$1"
shift

MDM_AGENT_IPS="$@"


[ "$DEBUG" != "1" ] || unset DEBUG


usage() {
    echo "Usage: $0 <mdm-server-ip> <csl1-ip> [<csl2-ip> ...]" >&2
    exit 1
}

setup_ssh() {
    ip="$1"

    echo -n "[+] Setting up SSH setup for ${ip}... "

    command ssh "root@${ip}" "cat ~/.ssh/id_rsa" | ssh-add - >/dev/null 2>&1

    echo "done"
}

launch_server() {
    echo -n "[+] Launching the MDM server on ${MDM_SERVER_IP}... "

    command ssh -t -t "root@${MDM_SERVER_IP}" sh <<- EOF ${DEBUG+>/dev/null 2>&1} &
        unset PS1

				cd $MDM_SERVER_DIR

				. venv/bin/activate

        docker compose up -d mongo

				python3 MDMServer.py -i "${MDM_SERVER_BIND_IP}" -p "${MDM_SERVER_PORT}" -d enabled &

        pid=$!
        trap "docker compose down; kill $pid" INT KILL QUIT EXIT
        wait $pid
		EOF
    PROCS="${PROCS:+$PROCS }$!"

    echo "done"
}

launch_client() {
    num="$1"
    ip="$2"

    echo -n "[+] Launching the MDM client on ${ip}... "

    command ssh -t -t "root@${ip}" sh <<- EOF ${DEBUG+>/dev/null 2>&1} &
        unset PS1

				cd $MDM_AGENT_DIR

				python3 comms_nats_controller.py -s "${MDM_SERVER_IP}" -p "${MDM_SERVER_PORT}" -a mdm -c "${MDM_AGENT_CERTS}/clients/csl${num}.local.crt" -k "${MDM_AGENT_CERTS}/clients/csl${num}.local.key" -r ${MDM_AGENT_CERTS}/clients/certs/ca.crt &

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

wait_for_cbma() {
    bat_neighbors="$1"

    [ $bat_neighbors -ne 0 ] || { echo "[+] Waiting forever..."; wait "${PROCS##* }"; }

    for ip in $MDM_AGENT_IPS; do
        echo -n "[+] Waiting for all neighbors in upper CBMA of ${ip}... "
        command ssh "root@${ip}" sh <<- EOF
						while batctl meshif bat1 n -H 2>/dev/null | awk -F'[[:space:]]*|s' '\$4>1{NR=0;exit}END{exit(NR=='${bat_neighbors}')}'; do
								sleep 1
						done
				EOF
        echo "done"
    done

    echo -n "[+] All good! :)"
}


[ $# -eq 0 ] && usage

trap handle_termination INT KILL QUIT EXIT

eval $(ssh-agent -s) >/dev/null

for ip in $MDM_AGENT_IPS; do
    setup_ssh "$ip"
done

if [ "$LAUNCH_MDM_SERVER" = "1" ]; then
    setup_ssh "$MDM_SERVER_IP"
    launch_server "$MDM_SERVER_IP"
    sleep 3
fi

for ((n=1; $#; ++n)); do
    launch_client $n "$1" "$MDM_SERVER_IP"
    shift
done

sleep 3

wait_for_cbma $(( n - 2 ))
