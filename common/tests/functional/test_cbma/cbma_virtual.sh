#! /bin/bash -x


NODES=`echo {A..C}`

KEYPAIR_TYPE="ecdsa"    # Can be ecdsa, eddsa, or rsa

DEBUG=0


#####################################

[ $EUID -ne 0 ] && { echo "[!] This script must be run as root" 1>&2; exit 1; }
[ "$DEBUG" = "0" ] && unset DEBUG || DEBUG=/dev/fd/1

############ CERTIFICATES ############
# Shamelessly stolen from 99_at_boot #
######################################

generate_filebased_private_key()
{
	  OUTPUT="$1"

	  case "$KEYPAIR_TYPE" in
		    ecdsa)
			      openssl genpkey -out "$OUTPUT" -algorithm EC -pkeyopt 'ec_paramgen_curve:P-256' -pkeyopt ec_param_enc:named_curve
			      ;;
		    eddsa)
			      openssl genpkey -out "$OUTPUT" -algorithm 'ed25519'
			      ;;
		    rsa)
			      openssl genpkey -out "$OUTPUT" -algorithm RSA -pkeyopt 'rsa_keygen_bits:3072'
			      ;;
		    *)
			      >&2 echo "Error: unsupported '$KEYPAIR_TYPE' algorithm"
			      exit 1
	  esac
}

filebased_security_officer_signs_csr()
{
	  CSR="$1"
	  CRT="$2"

	  UPPERCASE_KEYPAIR_TYPE=`echo "$KEYPAIR_TYPE" | tr '[:lower:]' '[:upper:]'`
	  TEMPSOCA_CRT="TEMPORARY_${UPPERCASE_KEYPAIR_TYPE}_SECURITY_OFFICER_CA_CRT"
	  TEMPSOCA_KEY="TEMPORARY_${UPPERCASE_KEYPAIR_TYPE}_SECURITY_OFFICER_CA_KEY"
    SERIAL=$( echo `date '+%Y%m%d%H%M%S%N'``dd if=/dev/urandom bs=9 count=1 status=none | od -tx1 -An -w99999 -v | tr -d ' '` | cut -c -40 )
    openssl x509 -in "$CSR" -out "$CRT" -sha256 -days 90 -req -set_serial "0x$SERIAL" -CA "${!TEMPSOCA_CRT}" -CAkey "${!TEMPSOCA_KEY}" -extensions default.extensions -extfile <( cat <<- EOF
		[ default ]
		openssl_conf            = default.openssl_conf
		extensions              = default.extensions

		[ default.openssl_conf ]
		oid_section             = default.openssl_conf.oid_section

		[ default.openssl_conf.oid_section ]
		# If worth the risk of breaking some buggy implementations, we might use some
		# custom OIDs to build our own Certificate Transparency equivalent for example
		# but for now this section is overloading the useless TSA default to clear it.

		[ default.extensions ]
		basicConstraints        = critical,CA:FALSE
		keyUsage                = critical,digitalSignature,keyEncipherment,dataEncipherment,keyAgreement
		extendedKeyUsage        = critical,serverAuth,clientAuth,ipsecIKE
		subjectKeyIdentifier    = hash
		authorityKeyIdentifier  = keyid:always
		#certificatePolicies    = @default.extensions.certificatePolicies
		subjectAltName          = $SUBJECT_ALT_NAME

		#[ default.extensions.certificatePolicies ]
		#policyIdentifier       = 1.3.6.1.4.1.53429
		#CPS                    = https://pki.ssrc.tii.ae/pki/mspki/

		[ default.extensions.policy ]
		commonName              = provided
		subjectAltName          = provided
	EOF
	) && rm "$CSR"
}


issue_filebased_certificate()
{
	  IDENTITY="$1"
	  IDENTITY_TYPE="$2"

	  FILEBASED_DIR="$CRYPTO/$KEYPAIR_TYPE/birth/filebased"
	  mkdir -p "$FILEBASED_DIR"
	  PRIVATE_KEY="$FILEBASED_DIR/private.key"
	  [ -s "$PRIVATE_KEY" ] || generate_filebased_private_key "$PRIVATE_KEY"

	  mkdir -p "$FILEBASED_DIR/$IDENTITY_TYPE"
	  CSR="$FILEBASED_DIR/$IDENTITY_TYPE/$IDENTITY.csr"
	  CRT="$FILEBASED_DIR/$IDENTITY_TYPE/$IDENTITY.crt"

	  case "$IDENTITY_TYPE" in
		    MAC)	SUBJECT_ALT_NAME="otherName:1.3.6.1.1.1.1.22;UTF8:$IDENTITY";;
		    *)	SUBJECT_ALT_NAME="$IDENTITY_TYPE:$IDENTITY"
	  esac
	  openssl req -key "$PRIVATE_KEY" -new -sha256 -out "$CSR" -config <( cat <<- EOF
		[ default ]
		openssl_conf            = default.openssl_conf
		extensions              = default.extensions

		[ default.openssl_conf ]
		oid_section             = default.openssl_conf.oid_section

		[ default.openssl_conf.oid_section ]
		# If worth the risk of breaking some buggy implementations, we might use some
		# custom OIDs to build our own Certificate Transparency equivalent for example
		# but for now this section is overloading the useless TSA default to clear it.

		[ default.extensions ]

		###############################################################################

		[ req ]
		encrypt_key             = no
		default_md              = sha256
		string_mask             = utf8only
		prompt                  = no
		distinguished_name      = req.distinguished_name
		req_extensions          = req.req_extensions

		[ req.distinguished_name ]
		commonName              = "$COMMON_NAME"

		[ req.req_extensions ]
		subjectAltName          = "$SUBJECT_ALT_NAME"
	EOF
	) && filebased_security_officer_signs_csr "$CSR" "$CRT"
}

# Assumes the MAC address is all lowercase and colon-separated the UNIX way
certify_mac_address()
{
	MAC_ADDRESS="$1"

	[ "$MAC_ADDRESS" != '00:00:00:00:00:00' ] || return 0
	[ "$MAC_ADDRESS" != 'ff:ff:ff:ff:ff:ff' ] || return 0
	issue_filebased_certificate "$MAC_ADDRESS" 'MAC'
}

flip_locally_administered_bit()
{
	MAC_ADDRESS="$1"

	NIBBLE1=`echo "$MAC_ADDRESS" | cut -c 1`
	LOCALLY_ADMINISTERED_FLIPPED_NIBBLE2=`echo "$MAC_ADDRESS" | cut -c 2 | tr '[0123456789abcdef]' '[23016745ab89efcd]'`
	REST_OF_MAC_ADDRESS=`echo "$MAC_ADDRESS" | cut -c 3-`
	echo "$NIBBLE1$LOCALLY_ADMINISTERED_FLIPPED_NIBBLE2$REST_OF_MAC_ADDRESS"
}

generate_subject_name()
{
	if [ -s /etc/machine-id ]; then
		cat /etc/machine-id
	else
		sha256sum /proc/cpuinfo | cut -c -32
	fi
}

#########################################


check_dependencies() {
    echo -n "[+] Checking dependencies... "

    if ! python3 -c 'import OpenSSL' &>/dev/null; then
        echo -e "\n[!] FATAL: PyOpenSSL is missing!"
        exit 1
    fi

    for t in brctl ebtables batctl; do
        if ! type $t &>/dev/null; then
            echo -e "\n[!] FATAL: $t is missing!"
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
        SUBJECT_NAME=`generate_subject_name`
        COMMON_NAME="$SUBJECT_NAME"

	      UKEYPAIR_TYPE=`echo "$KEYPAIR_TYPE" | tr '[:lower:]' '[:upper:]'`

	      for I in $NODES; do
            mkdir -p "certificates$I/$KEYPAIR_TYPE"
            CRYPTO="certificates$I"

	          eval "TEMPORARY_${UKEYPAIR_TYPE}_SECURITY_OFFICER_CA_CRT=$CRYPTO/$KEYPAIR_TYPE/filebased.crt"
	          eval "TEMPORARY_${UKEYPAIR_TYPE}_SECURITY_OFFICER_CA_KEY=$CRYPTO/$KEYPAIR_TYPE/filebased.key"
            eval "FILEBASED_CRT=\$TEMPORARY_${UKEYPAIR_TYPE}_SECURITY_OFFICER_CA_CRT"
            eval "FILEBASED_KEY=\$TEMPORARY_${UKEYPAIR_TYPE}_SECURITY_OFFICER_CA_KEY"

            awk '/^#'"$KEYPAIR_TYPE"'/{m=1}m&&/^-+BEGIN CERTIFICATE/{++c}c==1' "$SCRIPT_PATH" > "$FILEBASED_CRT"
            sed -n '/^#key-'"$KEYPAIR_TYPE"'/,/^#/{//!p}' "$SCRIPT_PATH" > "$FILEBASED_KEY"

            for D in wlp1s0 eth1; do
                MAC_ADDRESS="$(ip netns exec "$I" cat /sys/class/net/$D/address)"

                certify_mac_address "$MAC_ADDRESS"
                certify_mac_address `flip_locally_administered_bit "$MAC_ADDRESS"`
            done

            sed -n '/^#'"$KEYPAIR_TYPE"'/,/^#/{//!p}' "$SCRIPT_PATH" > "$CRYPTO/$KEYPAIR_TYPE/certificate_chain.crt"
        done
    ) >/dev/null 2>&1

    echo "Done"
}

remove_certificates() {
    echo -n "[+] Removing certificates... "

	  for I in $NODES; do
        rm -r "certificates$I"
    done

    echo "Done"
}

patch_cbma() {
    echo -n "[+] Patching CBMA to work with this script... "

    cp setup_cbma.py setup_cbma.py.bak

    sed -i '/^ .*wait_for_interface_to_be_pingable/,/)/{s/^\( *\))/&\n\1time.sleep(3)/;s/^/#/}' setup_cbma.py

    echo "Done"
}

unpatch_cbma() {
    echo -n "[+] Reverting CBMA patch... "

    mv setup_cbma.py.bak setup_cbma.py

    echo "Done"
}

launch_cbma() {
    echo -n "[+] Launching CBMA... "

    rm -rf logs

	  for I in $NODES; do
        PROCS="${PROCS:+$PROCS }$!"
        ip netns exec "$I" \
           python3 standalone_cbma.py -d "certificates$I/$KEYPAIR_TYPE/birth/filebased" \
                                      -c "certificates$I/$KEYPAIR_TYPE/certificate_chain.crt" \
                                      > ${DEBUG:-/dev/null} 2>&1 &
    done

    echo "Done"
}

wait_for_neighbors() {
    bat_neighbors=$(($(wc -w <<< "$NODES") - 1))

	  for I in $NODES; do
        echo -n "[+] Waiting for batman neighbors in CBMA node ${I}... "
        ip netns exec "$I" sh <<- EOF
						while batctl meshif bat0 n -H 2>/dev/null | awk -F'[[:space:]]*|s' '\$4>1{NR=0;exit}END{exit(NR=='${bat_neighbors}')}'; do
								sleep 1
						done
				EOF
        echo "done"
    done

    echo -n "[+] All good! :)"
}


interrupt_handler() {
    trap : INT EXIT QUIT KILL

    for P in $PROCS; do
        killall -n $P python3 2>/dev/null
    done

    echo

    destroy_nodes

    remove_certificates

    unpatch_cbma

    echo "[+] Exiting"

    exit $?
}


check_dependencies

(
    SCRIPT_PATH=$(realpath $0)

    cd $(dirname $0)/
    cd ../../../../modules/sc-mesh-secure-deployment/src/2_0/features/cbma/

    trap interrupt_handler INT EXIT QUIT KILL

    setup_nodes

    create_certificates

    patch_cbma

    launch_cbma

    wait_for_neighbors
)

exit $?


#ecdsa
-----BEGIN CERTIFICATE-----
MIIChDCCAimgAwIBAgIUICMSGBIhB1B4dplxS0IKheqCm0kwCgYIKoZIzj0EAwIw
LDEqMCgGA1UEAwwhTWVzaCBTaGllbGQgRUNEU0EgSW50ZXJtZWRpYXRlIENBMB4X
DTIzMTIxODA4MjEwN1oXDTI0MDMxNzA4MjEwN1owOjE4MDYGA1UEAwwvTWVzaCBT
aGllbGQgRUNEU0EgZmlsZWJhc2VkIFNlY3VyaXR5IE9mZmljZXIgQ0EwWTATBgcq
hkjOPQIBBggqhkjOPQMBBwNCAATlNHiIcni0oHbwMCuKnSl0dMVBSZWwyW32VxIk
PImcV3EDkniclXvMYtkvuBYuQG3POdrLZ4Xc6ZEeKe2UjooEo4IBGTCCARUwDwYD
VR0TBAgwBgEB/wIBADALBgNVHQ8EBAMCAYYwHQYDVR0OBBYEFHUFswBoC5zQwEbl
tapYy4tEopLVMB8GA1UdIwQYMBaAFBQ41TaxXavzMIMj8G9bx78rUfWcMIG0BgNV
HR4BAf8EgakwgaaggaMwCIIGLmxvY2FsMBGCD21lc2hzaGllbGQuY29ycDASghAu
bWVzaHNoaWVsZC5jb3JwMBGBD21lc2hzaGllbGQuY29ycDAlgSMubWVzaHNoaWVs
ZC5jb3JwO0ROUzptZXNoc2hpZWxkLmxhbjARgg8ubWVzaHNoaWVsZC5sYW4wEIEO
bWVzaHNoaWVsZC5sYW4wEYEPLm1lc2hzaGllbGQubGFuMAoGCCqGSM49BAMCA0kA
MEYCIQC6Xv2We9gczupT+fGYQ8xRmDaTX2lKibV6FVlaKm/vWgIhAKG4Cf+9s1kC
AHOCIYUPVp+Si36n2OtGLpoxuR4thDOd
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIICbDCCAhOgAwIBAgIUICMSGBIgVhN4g3NHj00iXpkLx9kwCgYIKoZIzj0EAwIw
JDEiMCAGA1UEAwwZTWVzaCBTaGllbGQgRUNEU0EgUm9vdCBDQTAeFw0yMzEyMTgw
ODIwNTZaFw0yNDEyMTcwODIwNTZaMCwxKjAoBgNVBAMMIU1lc2ggU2hpZWxkIEVD
RFNBIEludGVybWVkaWF0ZSBDQTBZMBMGByqGSM49AgEGCCqGSM49AwEHA0IABIYY
bTdPpty/3XNh3cZhsrAGg1QzwfNckFR/rffNOJqC1TjV99ty4bsK5ZXS1qVLg9UP
g3afAQEn4cZXKHnqarWjggEZMIIBFTAPBgNVHRMECDAGAQH/AgEBMAsGA1UdDwQE
AwIBhjAdBgNVHQ4EFgQUFDjVNrFdq/MwgyPwb1vHvytR9ZwwHwYDVR0jBBgwFoAU
UgdBTWtr24yoAERhHEoIWKXSgeAwgbQGA1UdHgEB/wSBqTCBpqCBozAIggYubG9j
YWwwEYIPbWVzaHNoaWVsZC5jb3JwMBKCEC5tZXNoc2hpZWxkLmNvcnAwEYEPbWVz
aHNoaWVsZC5jb3JwMCWBIy5tZXNoc2hpZWxkLmNvcnA7RE5TOm1lc2hzaGllbGQu
bGFuMBGCDy5tZXNoc2hpZWxkLmxhbjAQgQ5tZXNoc2hpZWxkLmxhbjARgQ8ubWVz
aHNoaWVsZC5sYW4wCgYIKoZIzj0EAwIDRwAwRAIge5TpzNgMlqHtJPOS5EsZhv5H
tLuHHkmG3sEWSESB9ncCIB1dN9Fn1o6eL2cWD6j4CKLPW2t1uZL3OR4+sUTeSh+s
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIBnjCCAUOgAwIBAgIUXkxay40nm3a6NjBpm/Pdyo3OMUMwCgYIKoZIzj0EAwIw
JDEiMCAGA1UEAwwZTWVzaCBTaGllbGQgRUNEU0EgUm9vdCBDQTAeFw0yMzEyMTgw
ODE5MzZaFw0yNDEyMTcwODE5MzZaMCQxIjAgBgNVBAMMGU1lc2ggU2hpZWxkIEVD
RFNBIFJvb3QgQ0EwWTATBgcqhkjOPQIBBggqhkjOPQMBBwNCAAT/XRnruO2Hy0IV
uIaiIjChPs8rUKVNzzwmKcVkoH1Ia3yeqfSAN+ZtbxR7g9ylP2gWJdlHUoAaxGqU
7h+2NzjGo1MwUTAPBgNVHRMECDAGAQH/AgECMB0GA1UdDgQWBBRSB0FNa2vbjKgA
RGEcSghYpdKB4DAfBgNVHSMEGDAWgBRSB0FNa2vbjKgARGEcSghYpdKB4DAKBggq
hkjOPQQDAgNJADBGAiEAmJItBxksDvgkF1WQAiI1VyAu9/T/6T5v7P3CcHAxtZsC
IQDBiypHdhVhX4wqXLn49DJBStBTxT3bDaT2AHGf9oxWAg==
-----END CERTIFICATE-----
#eddsa
-----BEGIN CERTIFICATE-----
MIICQzCCAfWgAwIBAgIUICMSGBIhEyAAU5g1ZYwEepsM5EYwBQYDK2VwMCwxKjAo
BgNVBAMMIU1lc2ggU2hpZWxkIEVkRFNBIEludGVybWVkaWF0ZSBDQTAeFw0yMzEy
MTgwODIxMTNaFw0yNDAzMTcwODIxMTNaMDoxODA2BgNVBAMML01lc2ggU2hpZWxk
IEVkRFNBIGZpbGViYXNlZCBTZWN1cml0eSBPZmZpY2VyIENBMCowBQYDK2VwAyEA
gE43rK1IlZb0AALtLQfSHJlcatElVz9wBztRUuTdHdKjggEZMIIBFTAPBgNVHRME
CDAGAQH/AgEAMAsGA1UdDwQEAwIBhjAdBgNVHQ4EFgQUgVA2gtEpSqeQlSAttTtU
DTsqz7IwHwYDVR0jBBgwFoAUsM4T/E5Ev9k4iRiPjeraWbtX4Q0wgbQGA1UdHgEB
/wSBqTCBpqCBozAIggYubG9jYWwwEYIPbWVzaHNoaWVsZC5jb3JwMBKCEC5tZXNo
c2hpZWxkLmNvcnAwEYEPbWVzaHNoaWVsZC5jb3JwMCWBIy5tZXNoc2hpZWxkLmNv
cnA7RE5TOm1lc2hzaGllbGQubGFuMBGCDy5tZXNoc2hpZWxkLmxhbjAQgQ5tZXNo
c2hpZWxkLmxhbjARgQ8ubWVzaHNoaWVsZC5sYW4wBQYDK2VwA0EA/sslQ4LCCzjR
xNhdc+Thm5VqnTPH+JY9Z9w69nUqSm0wTZdmULg+AN0sMGcs5zuTBy8Rc7Zu0fML
mj2THbcZAA==
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIICLTCCAd+gAwIBAgIUICMSGBIgViiQiQGXt4BKUi6X8N4wBQYDK2VwMCQxIjAg
BgNVBAMMGU1lc2ggU2hpZWxkIEVkRFNBIFJvb3QgQ0EwHhcNMjMxMjE4MDgyMDU2
WhcNMjQxMjE3MDgyMDU2WjAsMSowKAYDVQQDDCFNZXNoIFNoaWVsZCBFZERTQSBJ
bnRlcm1lZGlhdGUgQ0EwKjAFBgMrZXADIQA03dy4MGFgaJhlgWTvhQI7+IVFrCF7
UA9PRg7fDEKOJaOCARkwggEVMA8GA1UdEwQIMAYBAf8CAQEwCwYDVR0PBAQDAgGG
MB0GA1UdDgQWBBSwzhP8TkS/2TiJGI+N6tpZu1fhDTAfBgNVHSMEGDAWgBSrgEbK
VxKuWtOo4Z3mvUbKqM1yrzCBtAYDVR0eAQH/BIGpMIGmoIGjMAiCBi5sb2NhbDAR
gg9tZXNoc2hpZWxkLmNvcnAwEoIQLm1lc2hzaGllbGQuY29ycDARgQ9tZXNoc2hp
ZWxkLmNvcnAwJYEjLm1lc2hzaGllbGQuY29ycDtETlM6bWVzaHNoaWVsZC5sYW4w
EYIPLm1lc2hzaGllbGQubGFuMBCBDm1lc2hzaGllbGQubGFuMBGBDy5tZXNoc2hp
ZWxkLmxhbjAFBgMrZXADQQBq6Ube9zki+OKSCioZk+w0LPoW+vLizV5Q610KTmgK
rUsKcVd7UID/4yFFU7XtQtQQth9KYLRzb9sWUrRM42wE
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIBXTCCAQ+gAwIBAgIUcFhNdVa3yZgEvnDH8QzZVpCXrr4wBQYDK2VwMCQxIjAg
BgNVBAMMGU1lc2ggU2hpZWxkIEVkRFNBIFJvb3QgQ0EwHhcNMjMxMjE4MDgxOTM2
WhcNMjQxMjE3MDgxOTM2WjAkMSIwIAYDVQQDDBlNZXNoIFNoaWVsZCBFZERTQSBS
b290IENBMCowBQYDK2VwAyEAQZYoOCo077XjO/UqYvh+U6L0lB2U1BTM2n30dWlJ
DuejUzBRMA8GA1UdEwQIMAYBAf8CAQIwHQYDVR0OBBYEFKuARspXEq5a06jhnea9
RsqozXKvMB8GA1UdIwQYMBaAFKuARspXEq5a06jhnea9RsqozXKvMAUGAytlcANB
ANWBoQOwX5nvbLXlqW0PrIy7Lb10KALk4FZ468EpzxLpOIVgKs2Raxv4AbJWDUV4
t3bkR3qgPZpMKtJoPOQKTwo=
-----END CERTIFICATE-----
#rsa
-----BEGIN CERTIFICATE-----
MIIFCzCCA3OgAwIBAgIUICMSGBIhGFEZZhE9uMiAsP+Nm90wDQYJKoZIhvcNAQEL
BQAwKjEoMCYGA1UEAwwfTWVzaCBTaGllbGQgUlNBIEludGVybWVkaWF0ZSBDQTAe
Fw0yMzEyMTgwODIxMThaFw0yNDAzMTcwODIxMThaMDgxNjA0BgNVBAMMLU1lc2gg
U2hpZWxkIFJTQSBmaWxlYmFzZWQgU2VjdXJpdHkgT2ZmaWNlciBDQTCCAaIwDQYJ
KoZIhvcNAQEBBQADggGPADCCAYoCggGBAOpuSFOWR5scCtPF5jrGn17CY2zp6OGT
za5iGzg7Y0rOhBI2YVxUvv1bl1wEfeCGj9c9L0fcU36C8DI34j8tbF74kbYWSFxl
9RIl+bRK737OYnqiPjOAWG1jXRa/2xhG3+1b73Wv5wV50jn/+iXOPH7G78ScJXbA
KSwyDiR0ND1PQsAREj6aMC4vmcVcylcBY9GK+MIHmlcspYowMaSp5htHrRkormrj
ywh0OYqfzn/8DkSk9dmtzFFgMKUibgBUI05fqvm0WsIdGPo37OM/QHlqNQ2McNf6
0iMs+iVOgxuiTiSEDG/neRRB5CBnLCasrl9h2tyeUd0Xbrhwcwx0q73ej53qrtVZ
NrpT8SLXNz1LA3kv9rQRcQZT+nloIQSkQhs/ZuAUZgB/wktU/7H7DYulVHFYWa98
z9DiP2GBC9SKwQSKJuiSkkloCmEBvUZ2xCbsXKMzjOeigETqfVmJ7dxMVjADxwQt
JBlbCNsRKfy99SOk7SafkjCGSLPp0YtI4wIDAQABo4IBGTCCARUwDwYDVR0TBAgw
BgEB/wIBADALBgNVHQ8EBAMCAYYwHQYDVR0OBBYEFE0It5e0tX9ur5DB00yiDEYx
bsk9MB8GA1UdIwQYMBaAFAzZSODJmyBVyvLB2R3afbS1yJVgMIG0BgNVHR4BAf8E
gakwgaaggaMwCIIGLmxvY2FsMBGCD21lc2hzaGllbGQuY29ycDASghAubWVzaHNo
aWVsZC5jb3JwMBGBD21lc2hzaGllbGQuY29ycDAlgSMubWVzaHNoaWVsZC5jb3Jw
O0ROUzptZXNoc2hpZWxkLmxhbjARgg8ubWVzaHNoaWVsZC5sYW4wEIEObWVzaHNo
aWVsZC5sYW4wEYEPLm1lc2hzaGllbGQubGFuMA0GCSqGSIb3DQEBCwUAA4IBgQCA
JN/ZWx4fblsmRb6m+CIchO7Yty33F3m3oqLWGYXg5M+4hJp1NipN+KBqbsqvmvpy
ajEnPNPejEei0OMA8oQoimut3D7iH0RrnGmG68pLTCw6HiVPuCXC6WHJTG+ju9Ap
tiDgTY5UZ2Vd3I3EnhRF3Gbe2YE8Pidh7qXs+x4HbeEz89ZDas8NCQxBgS/lNEa5
pxJFa5NLa0FKuly+aJiTBZB5zKtkIKltrtGwugLnq66Uz2VobUtzhd/K1f4Oe7bf
clgVUG0CosEUTviHJR0uZ+82EgVpPMEfsxxmOw2ZldbE4Rm9+BbpgIPWRmBYkq3+
K+c9zXQMy+trdUMCOmiF3AzizpaKKOyzA3lcLjfMjX9KeJFFkBeXVELIGKXtTnOZ
pCiAORqgPPDME59/VYHYkC7/6V78h6IbM/KfWkK9tWurkI4BylH8iOwK6DlDS8Tb
RFU/NInn1Z/kxIGOO95P7dD85gKFHdwJj7k8Tm4y6/PTe1lis0yaIsxjZATUVFc=
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIE9TCCA12gAwIBAgIUICMSGBIgVkMzGCBEnqUlTMvLy7YwDQYJKoZIhvcNAQEL
BQAwIjEgMB4GA1UEAwwXTWVzaCBTaGllbGQgUlNBIFJvb3QgQ0EwHhcNMjMxMjE4
MDgyMDU2WhcNMjQxMjE3MDgyMDU2WjAqMSgwJgYDVQQDDB9NZXNoIFNoaWVsZCBS
U0EgSW50ZXJtZWRpYXRlIENBMIIBojANBgkqhkiG9w0BAQEFAAOCAY8AMIIBigKC
AYEArBHA65qw4Pe405pU7HEu0F6gyBGIfbg7VddoHpcxzQzwr0IzcVfKZzyjwppB
OuzNZeeFwKygCspF0BdKxc+oFqauoIEiPBjY7jfqPVgJreCtgRgaVRSeRiYIwwtf
qJGZICqqfT4BYs74T99VW1pcPUEFEXxI3ciPQ5HI62nHQ86xaRJIJsL2cg0IehPq
B6gnw0QhV96qiiamCE0CjjSSJYHXS4O4+yE81A46MnXozCZCUIlK5VBthvfF00xk
/hrxEyzwImpgxTRPGmz0yjSvPGN4lLw7GK3LLuHmqcnjBwCmqBcOCJ7UVvZ3gI3P
z569LiDSMaqAdyOMcY1ycA3o+b5ZwWBNZRsCqCPootpscMeNOe2VkPaRKiUt4dN3
FPaaF4ceY2/3Shs9VFChpMZIRvz3n2sgEuWQctjRsxg5GRGUklLGFYWzUNYQzWmz
JcV3t2Ub0ZkfMpzQZt6fsBBXPwsJjwVZK63irfDdliwVwGFGcAZX6vRm021WHdqo
c/xdAgMBAAGjggEZMIIBFTAPBgNVHRMECDAGAQH/AgEBMAsGA1UdDwQEAwIBhjAd
BgNVHQ4EFgQUDNlI4MmbIFXK8sHZHdp9tLXIlWAwHwYDVR0jBBgwFoAUS9UGcQXG
bNf+tgQ7192EuyFi53UwgbQGA1UdHgEB/wSBqTCBpqCBozAIggYubG9jYWwwEYIP
bWVzaHNoaWVsZC5jb3JwMBKCEC5tZXNoc2hpZWxkLmNvcnAwEYEPbWVzaHNoaWVs
ZC5jb3JwMCWBIy5tZXNoc2hpZWxkLmNvcnA7RE5TOm1lc2hzaGllbGQubGFuMBGC
Dy5tZXNoc2hpZWxkLmxhbjAQgQ5tZXNoc2hpZWxkLmxhbjARgQ8ubWVzaHNoaWVs
ZC5sYW4wDQYJKoZIhvcNAQELBQADggGBACm7abOcN2iHoLa5JVFgV2lIYaByZSkG
0XURw8gP1ZqydxpTDfteItfA1b1KDPWAuB7whhDUcSVZ4lmWsvT/fgUO0aOyfX6K
6oi3v/XlWyWRuazbfMPewOnoZyFwWM/Rtwjtc3qQQ2KQkyG695Uu1TrpaRHbZFTA
30aYY44SWGjHwIxAPFSCbUWM1EGm7mabDsbQDkWORkaR4FqcbGyMqf21tC6Zy1dv
HyKRwR14XTM5AcF5qBegqmvWQDcdkjOa0KvP+eV0gamTKEp2NUpqcJx1vZmKNGBt
Vgpn6oRpOuSQUhLKgWsLCx7fo5EaSi+mLJd6lb0muAenlLIWGr+gV1WdoP4Dn/+6
q2/7oia/lXcmGY5KNjn+m7heVPQj88ZeabIemiYOPhI5yMPTAJR+4zVEObWqTLVW
eag2nM8kb5QPzkKVgsPL0YAWQYoHoi1i+S8CLaIhtg+rUdRIEmN0opZnOfYXW2LZ
0bfJ27EyCY7d1K0nz3LUMISyIAkKbZdZ3Q==
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIEJTCCAo2gAwIBAgIUEn7hrJLPWUZ+LXsZRheBg5c5gDkwDQYJKoZIhvcNAQEL
BQAwIjEgMB4GA1UEAwwXTWVzaCBTaGllbGQgUlNBIFJvb3QgQ0EwHhcNMjMxMjE4
MDgxOTM2WhcNMjQxMjE3MDgxOTM2WjAiMSAwHgYDVQQDDBdNZXNoIFNoaWVsZCBS
U0EgUm9vdCBDQTCCAaIwDQYJKoZIhvcNAQEBBQADggGPADCCAYoCggGBAKOx9H5l
ZhnuxFlD1vC1nUXgqc9T0+us+W+r1m/9wwFtQOtK+ZpyfPNot6glaIA9dDJdrKvN
0jhkszd2wR/hs0WYDZ0jv3y7zTuvaqZbJgKejBtPoH1B5crd5U/KP2WjaHkMopdQ
5PhJAvm/GpaQqLT0aCwCwgw9UvbEF5SGCcnRAZ/h5Kp71Q//wn1FITYA0E9M+XP9
FtluQ9HgPJdl6FaJ+ziIsyAvCQ1ul561TpXsr2rtH57WOqJuXJvmS6GeIJ0u8B1x
rkFbGsncTp3/AC6CZfH8u6UFwVZ1PQAKrl2lzB6JHUHQ1NZvFnP+TU61qtXbvjnH
xAwsXfcpa5sILYmTNYLYlTzFPaI7sEH6sufxQpwHAEEIKgvPx9Bb7GaWXtBCu0AK
PfUl0bNF2EJkA6hKR7Ec9C6JjVG3dBYL9+8Bm2a4BQov4SGY5wKO+viNZRhtd8Sw
pRI2SEf/d6/bJk+nHgJ1PD73MuGx6RmGCf7t5VQCSCMfaQ8iHguHt1VG+wIDAQAB
o1MwUTAPBgNVHRMECDAGAQH/AgECMB0GA1UdDgQWBBRL1QZxBcZs1/62BDvX3YS7
IWLndTAfBgNVHSMEGDAWgBRL1QZxBcZs1/62BDvX3YS7IWLndTANBgkqhkiG9w0B
AQsFAAOCAYEAHVD7mrnbOSTLGE0TrUXSAY5xZc1S3LshGwwZ6416dgEFhRdmWc0q
8K9yoTPN4WI0lAGliIDgPaOOz0Xi+2213tsg52QeTxJ6B8T6MKbYUG1YKEdbgKED
8DDivbYssVhIBhUj9shxKMsU1IosZWoFtxpECCAuVJ3v0FVxZIK3CI/FviaqkDPy
qLf5EW2hxvEJ9v6p9UWNAu+ZUNvRGEbV9aCDZ3g68EpYCOLKXjnYX1+9DEtbkG5L
c8XXXLgTstTgaA4vG7a4ZTnyW91EPDjuzZMV1Ym3v6ys4Co0uWjhZ/YoLo6n9FmS
XFXQLOPKPRA/fWW5LP6P7NFId9qcV9UE3o4fgUDD7ZmaKieMSpLLur3dv5CLVDLT
QeYnPF6IVjTHERGfXFXuon9fLoORnYMaLjvjE4HgWXxZRd2YYxJL4b8xIkZVoDgS
NYjXOJqgNFNjYlj4spuV8xh0Out9mImLk6fDnfvjoS+zS1wK8pm8D/HoAvbp8V/x
Vv3ECoymmEcw
-----END CERTIFICATE-----
#key-ecdsa
-----BEGIN PRIVATE KEY-----
MIGHAgEAMBMGByqGSM49AgEGCCqGSM49AwEHBG0wawIBAQQg5stNP9mMK6K+2huZ
GC4nxvDFKXlxx7ajH+nhbW0F5/KhRANCAATlNHiIcni0oHbwMCuKnSl0dMVBSZWw
yW32VxIkPImcV3EDkniclXvMYtkvuBYuQG3POdrLZ4Xc6ZEeKe2UjooE
-----END PRIVATE KEY-----
#key-eddsa
-----BEGIN PRIVATE KEY-----
MC4CAQAwBQYDK2VwBCIEIIH5zf/4yoblUNoUm7SBb6bt7MuzjAQ77EMj26HoQ4L3
-----END PRIVATE KEY-----
#key-rsa
-----BEGIN PRIVATE KEY-----
MIIG/gIBADANBgkqhkiG9w0BAQEFAASCBugwggbkAgEAAoIBgQDqbkhTlkebHArT
xeY6xp9ewmNs6ejhk82uYhs4O2NKzoQSNmFcVL79W5dcBH3gho/XPS9H3FN+gvAy
N+I/LWxe+JG2FkhcZfUSJfm0Su9+zmJ6oj4zgFhtY10Wv9sYRt/tW+91r+cFedI5
//olzjx+xu/EnCV2wCksMg4kdDQ9T0LAERI+mjAuL5nFXMpXAWPRivjCB5pXLKWK
MDGkqeYbR60ZKK5q48sIdDmKn85//A5EpPXZrcxRYDClIm4AVCNOX6r5tFrCHRj6
N+zjP0B5ajUNjHDX+tIjLPolToMbok4khAxv53kUQeQgZywmrK5fYdrcnlHdF264
cHMMdKu93o+d6q7VWTa6U/Ei1zc9SwN5L/a0EXEGU/p5aCEEpEIbP2bgFGYAf8JL
VP+x+w2LpVRxWFmvfM/Q4j9hgQvUisEEiibokpJJaAphAb1GdsQm7FyjM4znooBE
6n1Zie3cTFYwA8cELSQZWwjbESn8vfUjpO0mn5Iwhkiz6dGLSOMCAwEAAQKCAYBP
r0nVY7UWQ6tofZG6882zM4cBO+cFWiylNELFU+Z1xIm++Q4SCNpcmzfxNrOk0Ihv
JKSAHCOjUpWbtEQqrmFyTLKWvq88/QkW8KKWhowuwa9WuPREpEEqeYZoV7F86IZw
E3prOe3gfh6XyWs5U4dbnnpcjajsWvgDxkXpXThuiJULXSIi9l0fLd3qaJZ8iYPo
84CHz3bGX+4q8SPGsVt+dVggv6O9wI1AY+U1OgnCliYxbhMFOV7EYFpYn0HdybsJ
+Z0wxlwu9xrKlqmB9yl0osEveIzAh+87nCxEPb/awm9OIhy+thNnfQuGEGbZmNfJ
YwKlrbLS0lNzwaVKUdRjW/GklUCQGuWGEbDUrNCVXTBggnDONeancjPyy02K0xbX
74TZjLBe50FB8LUge7zcfX3adh5kHhFguaZNwGec24LYYnqp/hdAuVdnqo8CPImw
9lssUfxyszIryRlYqJa+u71Z/kHO8wOA3LabX61/Aa/LllTmZFkQlWcZcnB1Py0C
gcEA91IEaxoHCZjws1DzyAe+9eMnT/Fhe/JKm+k6kc7Xo05DeEpObmOfVOcPn/tv
6ww7vPQU6jrnGXFPuepzZrA67AWYsJewQ1h9rvINKSQZ95HdcbuyuhGEjC+I3UzF
aw/E70YUWuNuLdzFYoxza39AboK9+ylSiI5wof/SYTtmMg7ngsm6zY6FKrdW1P2n
PY+lwMwBjUy98U2z3NwxMRM5jtpyj5mpX3E7fe5ESV28AU9d+zu8jnhaxSiEQIHW
fgUHAoHBAPKodlaSxv4aMWqkho8caiLB4I66N8Iqa/zJt4neSCI4v2Np45kZ9mzD
TB6xvr/p9Q88msftbHKVRsi6hCZfRlxQ00vHD6198uyJG57wwoS96jkLP6djOKis
waySaQy9OdWi+XT/qyi4pftq/0LqupeJkB/YW4Rye6yMZZh8bMOM5r2yEXx52LIh
cQOqJHiTOn2vLoSPsk+KnGS1Zm/o5pZmzeGW+hFeiYeyTNVuWmXVbCtMtoTMRRQO
hFVTWo8iRQKBwQDoCGzYxsx/dpuswHBW5N2F9520ZPRewwy4hfJLRdIOBqlKO/yl
mJWtpDKGl/GChjGtrxnIP4b92c9Az/OBDinZsc51L3tRU75m2fT6+kHagWFPCRg/
dZvzoc04QoSdU2giTI9gTivan1wYf0jyO6sJg2QWNciXEXweoNKSIpZYtJ8Dl5vI
qsPJNcUjgu7a1OBfp4Z1OOCjeZEbzwHf1veTTONaUoITZzq3hUlPw79VQ08bHApl
aAlRHBgoJShFhxUCgcA4Drrn3RijcyGfZUrAnOvYA7oAvvxKmso/p8oCy5sWbM2q
BbhSkjDfh256WM2cQEwgrPuE3FCFV/72OEB3LONwSCvIAcCZb9NC2ZnL6grF56nI
a9TUspNQRDipyVkDXGOT+wZuBPJhgea2Yk1+BAP/JgRplu3tdxUjE3wU8gmeU1qq
0AxCGVP6MseM9S1fvYNxrZqtfRTQU1E14sI7cC506GL/joVOpexXsNUXKVsw01Ed
36b6nsYGkCP6Ijt1fZECgcEA6y+5WK2fdMyMH/6pVIB/6LxBOkr/pQoRYedt31R6
IRkTFzBKdzAqFJB3D+o5jwevjhp54bLtjsV4wZbygcFRTnytKmxv6VBcBL0Imt78
nK0gXGfwM2jLTiLLdMgNxyhlhwYc+OmJx6QAdxJHmuoleNViuljHA4thQTGnaj8b
AHvJFP7LLh0sx8xh3odN2gNiwwfozkwMU6YsfOQaIZa0rDIxtTdHjbPWDCnci8pP
+YWHJK04/HJXp2bP8u8ZdfzX
-----END PRIVATE KEY-----
