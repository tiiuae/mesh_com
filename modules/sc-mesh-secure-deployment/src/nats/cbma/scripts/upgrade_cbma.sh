#!/bin/sh

# OPT_CBMA='/opt/mesh_com/modules/sc-mesh-secure-deployment/src/nats/cbma'
OPT_CBMA='/opt/nats/cbma'
IFACE_GLOB='/sys/class/net/[!bdls][tals][!a]*'

SSH_OPTIONS='-o UserKnownHostsFile=/dev/null -o StrictHostKeychecking=no'


usage() {
    echo "[!] Usage: $0 </path/to/cbma> <IP> [<path/to/device_id_rsa]"
    echo
    echo "    Example: $0 ./cbma 192.168.13.37"
}

stop_mdm_agent() {
    IP="$1"
    ID_RSA="$2"

    echo "[+] Stopping MDM Agent"
    command ssh ${ID_RSA:+-i "$ID_RSA"} $SSH_OPTIONS "root@$IP" bash <<-EOF || exit 4
			/opt/S90mdm_agent stop &>/dev/null
			sleep 2
			ps aux | awk '/python.*mdm_agent/&&!/awk/{system("kill -9 " \$1)}' &>/dev/null
			{ ls -1d /sys/class/net/[ul]m* | cut -d '/' -f 5 | xargs -n1 ip link del; } &>/dev/null
			ebtables -t nat -F
			ebtables -t nat -L | awk -F' *|,' '/ [lu]mb[^,]+,/{system("ebtables -t nat -X " \$3)}' || :
		EOF
}

cleanup_logs() {
    IP="$1"
    ID_RSA="$2"

    echo "[+] Cleaning up logs"
    command ssh ${ID_RSA:+-i "$ID_RSA"} $SSH_OPTIONS "root@$IP" bash <<-EOF || exit 5
			rm -f /opt/mdm_agent.log
			rm -rf /var/log/cbma || :
		EOF
}

cleanup_mdm_agent_files() {
    IP="$1"
    ID_RSA="$2"

    echo "[+] Removing MDM Agent downloaded certs and uploaded check"
    command ssh ${ID_RSA:+-i "$ID_RSA"} $SSH_OPTIONS "root@$IP" bash <<-EOF || exit 6
			rm -f /opt/certs_uploaded
			rm -rf /opt/certs || :
			rm -rf /opt/mdm || :
		EOF
}

upgrade_cbma() {
    CBMA="./$1"
    IP="$2"
    ID_RSA="$3"

    echo "[+] Upgrading CBMA"
    command ssh ${ID_RSA:+-i "$ID_RSA"} $SSH_OPTIONS "root@$IP" "rm -rf '$OPT_CBMA' >/dev/null 2>&1 || :" || exit 7

    tar -C "${CBMA%/*}" -zcf - "${CBMA##*/}" | command ssh ${ID_RSA:+-i "$ID_RSA"} $SSH_OPTIONS "root@$IP" "tar -C '${OPT_CBMA%/*}' -zxf - >/dev/null 2>&1" || exit 8
}

upgrade_birthcerts() {
    IP="$1"
    ID_RSA="$2"

    echo "[+] Upgrading birth certs"
    command ssh ${ID_RSA:+-i "$ID_RSA"} $SSH_OPTIONS "root@$IP" bash <<-EOF || exit 9
			set -e
			TMP="\$(mktemp -d)"
			trap "rm -rf '\$TMP'" INT EXIT QUIT TERM KILL
			cd "\$TMP"
			export PATH="/usr/local/bin:\$PATH"
			GENERATE_CERTIFICATES="$OPT_CBMA/scripts/generate_certificates.sh"
			for iface in $IFACE_GLOB; do
				for key_type in ecdsa eddsa rsa; do
					echo "[+] Generating \$key_type certificates for \${iface##*/}"
					KEYPAIR_TYPE="\$key_type" bash "\$GENERATE_CERTIFICATES" "\${iface##*/}" >/dev/null
				done
			done
			cp -r certificates/* /opt/crypto/
			for key_type in ecdsa eddsa rsa; do
				for cert in intermediate.crt root.crt; do
					cp "certificates/\$key_type/\$cert" "/opt/mspki/\$key_type/\$cert"
				done
				for cert in filebased.crt filebased.key; do
					cp "certificates/\$key_type/\$cert" "/opt/mspki/\$key_type/security_officers/\$cert"
				done
			done
		EOF
}

start_mdm_agent() {
    IP="$1"
    ID_RSA="$2"

    echo "[+] Starting MDM Agent"
    command ssh ${ID_RSA:+-i "$ID_RSA"} $SSH_OPTIONS "root@$IP" "/opt/S90mdm_agent start >/dev/null 2>&1" || exit 10
}

if [ $# -lt 2 ]; then
    usage
    exit 1
fi

CBMA="$1"
IP="$2"
ID_RSA="$3"

if ! ping -W 2 -c 1 "$IP" >/dev/null 2>&1; then
    echo "[!] Host '$IP' is not reachable"
    usage
    exit 2
fi
if [ ! -f "$CBMA/VERSION" ]; then
    echo "[!] $CBMA is not a valid CBMA directory"
    usage
    exit 3
fi

stop_mdm_agent "$IP" "$ID_RSA"
cleanup_logs "$IP" "$ID_RSA"
cleanup_mdm_agent_files "$IP" "$ID_RSA"
# upgrade_cbma "$CBMA" "$IP" "$ID_RSA"
# upgrade_birthcerts "$IP" "$ID_RSA"
start_mdm_agent "$IP" "$ID_RSA"
