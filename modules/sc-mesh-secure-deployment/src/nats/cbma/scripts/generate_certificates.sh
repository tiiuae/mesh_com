#!/bin/bash -x


DEFAULT_KEYPAIR_TYPE="ecdsa"    # Can be ecdsa, eddsa, or rsa

DEBUG=0


#####################################


usage() {
    echo "[+] Usage: $0 <interface> [<interface> ...]"
    echo
}

if [ $# -eq 0 ]; then
    usage
    exit 1
fi

INTERFACES="$@"

for I in $INTERFACES; do
    if [ ! -e /sys/class/net/$I/address ]; then
        echo "[!] Invalid '$I' interface"
        echo
        usage
        exit 2
    fi
done


[ "$DEBUG" = "0" ] && unset DEBUG || DEBUG=/dev/fd/1

[ -n "$CERTIFICATE_FOLDERS" ] || CERTIFICATE_FOLDERS='certificates'

KEYPAIR_TYPE="${KEYPAIR_TYPE:-$DEFAULT_KEYPAIR_TYPE}"


############ CERTIFICATES #############
# Shamelessly adapted from 99_at_boot #
#######################################

generate_filebased_private_key() {
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

filebased_security_officer_signs_csr() {
    CSR="$1"
    CRT="$2"

    cts=$(date +%s)
    jan_1_1970_ts=$(TZ="GMT" date -d "1970-01-01 00:00:01" +%s)
    diff=$((jan_1_1970_ts - cts))
    UPPERCASE_KEYPAIR_TYPE="${KEYPAIR_TYPE^^}"
    TEMPSOCA_CRT="TEMPORARY_${UPPERCASE_KEYPAIR_TYPE}_SECURITY_OFFICER_CA_CRT"
    TEMPSOCA_KEY="TEMPORARY_${UPPERCASE_KEYPAIR_TYPE}_SECURITY_OFFICER_CA_KEY"
    SERIAL=$( echo "$(date '+%Y%m%d%H%M%S%N')$(dd if=/dev/urandom bs=9 count=1 status=none | od -tx1 -An -w99999 -v | tr -d ' ')" | cut -c -40 )
    end_date=$(openssl x509 -in "${!TEMPSOCA_CRT}" -noout -enddate | cut -d= -f2)
    end_ts=$(date -d "$end_date" +%s)
    enddiff=$((end_ts - jan_1_1970_ts))
    days=$(( (enddiff / (60 * 60 * 24)) + 1))
    faketime -f "${diff}s" openssl x509 -in "$CSR" -out "$CRT" -sha256 -days "$days" -req -set_serial "0x$SERIAL" -CA "${!TEMPSOCA_CRT}" -CAkey "${!TEMPSOCA_KEY}" -extensions default.extensions -extfile <( cat <<- EOF
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


issue_filebased_certificate() {
    IDENTITY="$1"
    IDENTITY_TYPE="$2"

    FILEBASED_DIR="$CRYPTO/$KEYPAIR_TYPE/birth/filebased"
    PRIVATE_KEY="$FILEBASED_DIR/private.key"

    mkdir -p "$FILEBASED_DIR"
    [ -s "$PRIVATE_KEY" ] || generate_filebased_private_key "$PRIVATE_KEY"

    mkdir -p "$FILEBASED_DIR/$IDENTITY_TYPE"
    CSR="$FILEBASED_DIR/$IDENTITY_TYPE/$IDENTITY.csr"
    CRT="$FILEBASED_DIR/$IDENTITY_TYPE/$IDENTITY.crt"

    case "$IDENTITY_TYPE" in
        MAC) SUBJECT_ALT_NAME="otherName:1.3.6.1.1.1.1.22;UTF8:$IDENTITY";;
        *) SUBJECT_ALT_NAME="$IDENTITY_TYPE:$IDENTITY"
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
certify_mac_address() {
    MAC_ADDRESS="$1"

    [ "$MAC_ADDRESS" != '00:00:00:00:00:00' ] || return 0
    [ "$MAC_ADDRESS" != 'ff:ff:ff:ff:ff:ff' ] || return 0

    echo "[+] Generating certificates for $MAC_ADDRESS"

    issue_filebased_certificate "$MAC_ADDRESS" 'MAC'
    issue_filebased_certificate "$(flip_locally_administered_bit "$MAC_ADDRESS")" 'MAC'
}

flip_locally_administered_bit() {
    MAC_ADDRESS="$1"

    NIBBLE1="$(echo "$MAC_ADDRESS" | cut -c 1)"
    LOCALLY_ADMINISTERED_FLIPPED_NIBBLE2="$(echo "$MAC_ADDRESS" | cut -c 2 | tr '[0123456789abcdef]' '[23016745ab89efcd]')"
    REST_OF_MAC_ADDRESS="$(echo "$MAC_ADDRESS" | cut -c 3-)"

    echo "${NIBBLE1}${LOCALLY_ADMINISTERED_FLIPPED_NIBBLE2}${REST_OF_MAC_ADDRESS}"
}

generate_subject_name() {
    if [ -s /etc/machine-id ]; then
        cat /etc/machine-id
    else
        sha256sum /proc/cpuinfo | cut -c -32
    fi
}

#########################################


check_dependencies() {
    echo -n "[+] Checking dependencies... "

    for t in openssl faketime; do
        if ! type $t &>/dev/null; then
            echo -e "\n[!] FATAL: $t is missing!"
            exit 1
        fi
    done
    echo "Done"
}

create_certificates() {
    echo -n "[+] Creating certificates... "

    (
        echo

        SUBJECT_NAME="$(generate_subject_name)"
        COMMON_NAME="$SUBJECT_NAME"

        UKEYPAIR_TYPE="${KEYPAIR_TYPE^^}"

        for P in $CERTIFICATE_FOLDERS; do
            CRYPTO="$P"

            echo "[+] Generating certificates under '$CRYPTO'"

            mkdir -p "$CRYPTO/$KEYPAIR_TYPE/ca"

            eval "TEMPORARY_${UKEYPAIR_TYPE}_SECURITY_OFFICER_CA_CRT=$CRYPTO/$KEYPAIR_TYPE/filebased.crt"
            eval "TEMPORARY_${UKEYPAIR_TYPE}_SECURITY_OFFICER_CA_KEY=$CRYPTO/$KEYPAIR_TYPE/filebased.key"
            eval "FILEBASED_CRT=\$TEMPORARY_${UKEYPAIR_TYPE}_SECURITY_OFFICER_CA_CRT"
            eval "FILEBASED_KEY=\$TEMPORARY_${UKEYPAIR_TYPE}_SECURITY_OFFICER_CA_KEY"

            sed -n '/^#key-'"$KEYPAIR_TYPE"'/,/^#/{//!p}' "$SCRIPT_PATH" > "$FILEBASED_KEY"
            awk '/^#/{c=m=/^#'"$KEYPAIR_TYPE"'/}m&&/^-+BEGIN CERTIFICATE/{++c}c==2' "$SCRIPT_PATH" > "$FILEBASED_CRT"
            awk '/^#/{c=m=/^#'"$KEYPAIR_TYPE"'/}m&&/^-+BEGIN CERTIFICATE/{++c}c==3' "$SCRIPT_PATH" > "$CRYPTO/$KEYPAIR_TYPE/intermediate.crt"
            awk '/^#/{c=m=/^#'"$KEYPAIR_TYPE"'/}m&&/^-+BEGIN CERTIFICATE/{++c}c==4' "$SCRIPT_PATH" > "$CRYPTO/$KEYPAIR_TYPE/root.crt"
            sed -n '/^#'"$KEYPAIR_TYPE"'/,/^#/{//!p}' "$SCRIPT_PATH" > "$CRYPTO/$KEYPAIR_TYPE/certificate_chain.crt"

            # TODO - CA certs need to be added to the folder
            openssl rehash "$CRYPTO/$KEYPAIR_TYPE/ca"

            for I in $INTERFACES; do
                MAC_ADDRESS="$(cat /sys/class/net/$I/address)"

                [ -n "$MAC_ADDRESS" ] || { echo "[!] $I doesn't have a MAC address"; exit 3; }

                certify_mac_address "$MAC_ADDRESS"
            done
        done
    ) > ${DEBUG:-/dev/null} 2>&1

    echo "Done"
}

remove_certificates() {
    echo -n "[+] Removing certificates... "

    for P in $CERTIFICATE_FOLDERS; do
        rm -fr "$P"
    done

    echo "Done"
}

interrupt_handler() {
    trap : INT EXIT QUIT KILL

    echo

    echo "[!] Operation interrupted"

    remove_certificates

    ret_code=$?

    echo "[+] Exiting"

    exit $ret_code
}


check_dependencies

(
    SCRIPT_PATH=$(realpath $0)

    trap interrupt_handler INT QUIT KILL

    create_certificates || remove_certificates

    echo "[+] Exiting"
)

exit $?


#ecdsa
-----BEGIN CERTIFICATE-----
MIICgzCCAimgAwIBAgIUICQDGBZDQFRnNEgyStoQOeYbbVgwCgYIKoZIzj0EAwIw
LDEqMCgGA1UEAwwhTWVzaCBTaGllbGQgRUNEU0EgSW50ZXJtZWRpYXRlIENBMB4X
DTcwMDEwMTAwMDAwMVoXDTI0MDkxNDAwMDAwMVowOjE4MDYGA1UEAwwvTWVzaCBT
aGllbGQgRUNEU0EgZmlsZWJhc2VkIFNlY3VyaXR5IE9mZmljZXIgQ0EwWTATBgcq
hkjOPQIBBggqhkjOPQMBBwNCAAR87Z+CCTFQ01K7ilOYOefNKlj5rfwyi7CFmVvJ
9hzuiPwjnpGEXCpZPnm+f/px4CvNM5T0TWZlOrqR89vOTr+eo4IBGTCCARUwDwYD
VR0TBAgwBgEB/wIBADALBgNVHQ8EBAMCAYYwHQYDVR0OBBYEFKoM4ReytDaXKelb
efciMef//GB7MB8GA1UdIwQYMBaAFJrZ86BJkwJn5kNZusQWWnwzrg3KMIG0BgNV
HR4BAf8EgakwgaaggaMwCIIGLmxvY2FsMBGCD21lc2hzaGllbGQuY29ycDASghAu
bWVzaHNoaWVsZC5jb3JwMBGBD21lc2hzaGllbGQuY29ycDAlgSMubWVzaHNoaWVs
ZC5jb3JwO0ROUzptZXNoc2hpZWxkLmxhbjARgg8ubWVzaHNoaWVsZC5sYW4wEIEO
bWVzaHNoaWVsZC5sYW4wEYEPLm1lc2hzaGllbGQubGFuMAoGCCqGSM49BAMCA0gA
MEUCIQCZaqgmByr+0992bef17mjE90b+7LsR8a2FOR1cQO2TqQIgZmar3NDns6ya
vU7Nxzctcu4Jyv9Xc0WUAW4HdPPvMmg=
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIICbTCCAhOgAwIBAgIUICQDGBZBRjIjg3NTw69vUk4eQMwwCgYIKoZIzj0EAwIw
JDEiMCAGA1UEAwwZTWVzaCBTaGllbGQgRUNEU0EgUm9vdCBDQTAeFw03MDAxMDEw
MDAwMDFaFw0yNTAzMTgwMDAwMDFaMCwxKjAoBgNVBAMMIU1lc2ggU2hpZWxkIEVD
RFNBIEludGVybWVkaWF0ZSBDQTBZMBMGByqGSM49AgEGCCqGSM49AwEHA0IABHrg
s1VVwlIkV6+r2RqlVCOo3NMqTnYeKO49MCPVWW3av86Q+b5w1fOJVwpxwMO/aXCV
qzhxDtZB8sIO4l9zHgejggEZMIIBFTAPBgNVHRMECDAGAQH/AgEBMAsGA1UdDwQE
AwIBhjAdBgNVHQ4EFgQUmtnzoEmTAmfmQ1m6xBZafDOuDcowHwYDVR0jBBgwFoAU
DWy4d2tulKsAfaZGx11qlKVewugwgbQGA1UdHgEB/wSBqTCBpqCBozAIggYubG9j
YWwwEYIPbWVzaHNoaWVsZC5jb3JwMBKCEC5tZXNoc2hpZWxkLmNvcnAwEYEPbWVz
aHNoaWVsZC5jb3JwMCWBIy5tZXNoc2hpZWxkLmNvcnA7RE5TOm1lc2hzaGllbGQu
bGFuMBGCDy5tZXNoc2hpZWxkLmxhbjAQgQ5tZXNoc2hpZWxkLmxhbjARgQ8ubWVz
aHNoaWVsZC5sYW4wCgYIKoZIzj0EAwIDSAAwRQIgOTCXwmHkvEUllOSI4hS8avR1
59n+aoZYh4iSI7E4u8ECIQCiHweiexlUXD+mBTjPvbZBywKtF1eC0fOJTXkX20vh
6A==
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIBnTCCAUOgAwIBAgIUXWtrcIDlT6T5dqtcbAfe6GE6BXYwCgYIKoZIzj0EAwIw
JDEiMCAGA1UEAwwZTWVzaCBTaGllbGQgRUNEU0EgUm9vdCBDQTAeFw03MDAxMDEw
MDAwMDFaFw0yNTAzMTgwMDAwMDFaMCQxIjAgBgNVBAMMGU1lc2ggU2hpZWxkIEVD
RFNBIFJvb3QgQ0EwWTATBgcqhkjOPQIBBggqhkjOPQMBBwNCAAQlN6/LuPmyf1X8
jJK0iIgVvryb8VdfAm0A+GqLSyq/Nk+/nDqJ/gty79euBuQxiRozVYOR3lPxAcSz
h/qJso1Wo1MwUTAPBgNVHRMECDAGAQH/AgECMB0GA1UdDgQWBBQNbLh3a26UqwB9
pkbHXWqUpV7C6DAfBgNVHSMEGDAWgBQNbLh3a26UqwB9pkbHXWqUpV7C6DAKBggq
hkjOPQQDAgNIADBFAiEAvgjBFgkRfsWT9TtgmBt1ww9e3/sQnPwOzdWhMhrJavoC
IBjptI5+H1hdA87GSee8VxYcr+Tghl6B2BDp/zXyGn5z
-----END CERTIFICATE-----
#eddsa
-----BEGIN CERTIFICATE-----
MIICQzCCAfWgAwIBAgIUICQDGBZDQncEkUMWssMg5hMaobcwBQYDK2VwMCwxKjAo
BgNVBAMMIU1lc2ggU2hpZWxkIEVkRFNBIEludGVybWVkaWF0ZSBDQTAeFw03MDAx
MDEwMDAwMDFaFw0yNDA5MTQwMDAwMDFaMDoxODA2BgNVBAMML01lc2ggU2hpZWxk
IEVkRFNBIGZpbGViYXNlZCBTZWN1cml0eSBPZmZpY2VyIENBMCowBQYDK2VwAyEA
7nTADnHaEk1kfxu00fPx0QntzkWbqP5QLXy5okJ6zAujggEZMIIBFTAPBgNVHRME
CDAGAQH/AgEAMAsGA1UdDwQEAwIBhjAdBgNVHQ4EFgQU2GD1q2DD2obhD55fPQtR
pPvBzegwHwYDVR0jBBgwFoAUKjbojsBR2TSV60gGz4vs/I04tpQwgbQGA1UdHgEB
/wSBqTCBpqCBozAIggYubG9jYWwwEYIPbWVzaHNoaWVsZC5jb3JwMBKCEC5tZXNo
c2hpZWxkLmNvcnAwEYEPbWVzaHNoaWVsZC5jb3JwMCWBIy5tZXNoc2hpZWxkLmNv
cnA7RE5TOm1lc2hzaGllbGQubGFuMBGCDy5tZXNoc2hpZWxkLmxhbjAQgQ5tZXNo
c2hpZWxkLmxhbjARgQ8ubWVzaHNoaWVsZC5sYW4wBQYDK2VwA0EAqyBgbmHXirZH
0lH1rjuMc8lFDMUb+ZMcGLpPSdvz+bTg3YktArv3lfdB9nLWQMwk1xLDtPfqHXCS
0C6TfSiLCg==
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIICLTCCAd+gAwIBAgIUICQDGBZBRjd5dzNBVj0pzx6aAM0wBQYDK2VwMCQxIjAg
BgNVBAMMGU1lc2ggU2hpZWxkIEVkRFNBIFJvb3QgQ0EwHhcNNzAwMTAxMDAwMDAx
WhcNMjUwMzE4MDAwMDAxWjAsMSowKAYDVQQDDCFNZXNoIFNoaWVsZCBFZERTQSBJ
bnRlcm1lZGlhdGUgQ0EwKjAFBgMrZXADIQA0WbmSuXQjuR8NUlFHB6p53yr0AyZc
P/kDySGeF/qdvqOCARkwggEVMA8GA1UdEwQIMAYBAf8CAQEwCwYDVR0PBAQDAgGG
MB0GA1UdDgQWBBQqNuiOwFHZNJXrSAbPi+z8jTi2lDAfBgNVHSMEGDAWgBQKGgHV
jcSp6exuI7RJUffupMf2HTCBtAYDVR0eAQH/BIGpMIGmoIGjMAiCBi5sb2NhbDAR
gg9tZXNoc2hpZWxkLmNvcnAwEoIQLm1lc2hzaGllbGQuY29ycDARgQ9tZXNoc2hp
ZWxkLmNvcnAwJYEjLm1lc2hzaGllbGQuY29ycDtETlM6bWVzaHNoaWVsZC5sYW4w
EYIPLm1lc2hzaGllbGQubGFuMBCBDm1lc2hzaGllbGQubGFuMBGBDy5tZXNoc2hp
ZWxkLmxhbjAFBgMrZXADQQAwdyWffENxKiBVk1DTMfBx3NbiA8kCjdJll9cFuxeF
qw9qgb/IOI9IuZLjreQ90/TJSyw4w/i7Fu+2T2gdwZ0P
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIBXTCCAQ+gAwIBAgIUd+ecwYO8huaj25vPjNpiTOcflmowBQYDK2VwMCQxIjAg
BgNVBAMMGU1lc2ggU2hpZWxkIEVkRFNBIFJvb3QgQ0EwHhcNNzAwMTAxMDAwMDAx
WhcNMjUwMzE4MDAwMDAxWjAkMSIwIAYDVQQDDBlNZXNoIFNoaWVsZCBFZERTQSBS
b290IENBMCowBQYDK2VwAyEAeGsWqieq2lmokQ9z1rg8hJjtZ0QO2RWfahvfPFsM
68ejUzBRMA8GA1UdEwQIMAYBAf8CAQIwHQYDVR0OBBYEFAoaAdWNxKnp7G4jtElR
9+6kx/YdMB8GA1UdIwQYMBaAFAoaAdWNxKnp7G4jtElR9+6kx/YdMAUGAytlcANB
ACUpjssjBLv4rmJpmjHaGQVsY6+VX2MEW5/L5MPy6v3nvGt2iPqPvK3g1IAj2WGj
sBbF0WSDgOVJyImVzCYGDgo=
-----END CERTIFICATE-----
#rsa
-----BEGIN CERTIFICATE-----
MIIFCzCCA3OgAwIBAgIUICQDGBZDRHg1MAdiN+KIAmbVTKAwDQYJKoZIhvcNAQEL
BQAwKjEoMCYGA1UEAwwfTWVzaCBTaGllbGQgUlNBIEludGVybWVkaWF0ZSBDQTAe
Fw03MDAxMDEwMDAwMDFaFw0yNDA5MTQwMDAwMDFaMDgxNjA0BgNVBAMMLU1lc2gg
U2hpZWxkIFJTQSBmaWxlYmFzZWQgU2VjdXJpdHkgT2ZmaWNlciBDQTCCAaIwDQYJ
KoZIhvcNAQEBBQADggGPADCCAYoCggGBAK4IScvEd4Scs6irGjefxTimLNMMU/gP
1PzHY5tPzXB8SJnjUOuZK6ejut4wh7Op5sFpQGG6GLiggfPZKCDBaUos+sMZAvOq
D2O2gdMjJQbVwWU8NCn+yq17kENsEdYHJ2WcNgK+AKcsCzTGf+naMU8B8tKFd1ug
td/01YW6eqo83xmpm7O8h6AL5cHhkW8Zya1yL6RMzhMA3GyXIxrU1RFRcKkF2qpB
5ZWAWY1dltsE84ZPqlcPhdcmrTXt+ZcU9PfGGLMg7Q4vRkN0YCkV+VIvzS/g4MHU
Q8ubvd0z0k029EL7KuPiA3GlhQobplYiVCFIDMZ3H2Y2S4dcjyMxl1hDjTQvlIw8
m+pQymuPcnE4u6DFkrYq0U6t7OE8kgZbUTTxojDm+VSCa/wNvpdsERAn+Hh5H69A
ibTVYBEKvLEbuiLhopxcWdrEHkjB6gd1I60T0X8vtQ6sPRQ/f9XkHWcebP0NlW9E
HoaQEKpG8bpBQ+OPm+Px/QTZShGIM81LqwIDAQABo4IBGTCCARUwDwYDVR0TBAgw
BgEB/wIBADALBgNVHQ8EBAMCAYYwHQYDVR0OBBYEFFDOB3PMPwgPwcLTdQ7ane9E
XUkgMB8GA1UdIwQYMBaAFO0Qfq2662ti1u8KZjeBEtllUPOkMIG0BgNVHR4BAf8E
gakwgaaggaMwCIIGLmxvY2FsMBGCD21lc2hzaGllbGQuY29ycDASghAubWVzaHNo
aWVsZC5jb3JwMBGBD21lc2hzaGllbGQuY29ycDAlgSMubWVzaHNoaWVsZC5jb3Jw
O0ROUzptZXNoc2hpZWxkLmxhbjARgg8ubWVzaHNoaWVsZC5sYW4wEIEObWVzaHNo
aWVsZC5sYW4wEYEPLm1lc2hzaGllbGQubGFuMA0GCSqGSIb3DQEBCwUAA4IBgQAZ
8/yt88Li3kb5t8KUPCo1K+lqpGWSB/dkQRxSM95TXV4ZiXT9MUxJAd43n8sXBtlm
KBjwfXsdjGVBQcx8Ifnqyt5QVBL+GgdBVeJwVXgwPFFoz/iEizTCbZ+cy4xsXaqy
SoROvM53bn3M2ocBxrIV800lJOpMh3WzCgKJb1zejgVH1EyaUx8nAtPLDzaW0N8Q
3+5g1RI7QG11eJEYBDLucLbHqdgFuAY6jQygS3U6Cm4riXzqKiho/IXDuySvMaoJ
9MHfIy2OuRI6qfB0FueDsy9AQuHwOXmLfKOMh1nfrJ3nOdUnR6jcTQw77fql6zQI
+5/JotHogFVipM7W3qM+Ay78eT3UJoF6DB4U3F1gMqEhlPXRww+cs/+Qt0CuClSi
PKgwPpeVlPpOM5noV6c+hYa1eqXtGTaG9DPTctB3iUOjYpcmMzvB01+/uPSxF13M
358RWz2Zzs5XzbH2ZV7NAHA2o2S3LBtqw1D7HMQt3y/l/2L3WvqSYkGjVH1O/Yg=
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIE9TCCA12gAwIBAgIUICQDGBZBRkKBZ4QoAG/CrFm5urgwDQYJKoZIhvcNAQEL
BQAwIjEgMB4GA1UEAwwXTWVzaCBTaGllbGQgUlNBIFJvb3QgQ0EwHhcNNzAwMTAx
MDAwMDAxWhcNMjUwMzE4MDAwMDAxWjAqMSgwJgYDVQQDDB9NZXNoIFNoaWVsZCBS
U0EgSW50ZXJtZWRpYXRlIENBMIIBojANBgkqhkiG9w0BAQEFAAOCAY8AMIIBigKC
AYEAn4mwHISMAQ9AOw9oa98YcA92sYNVfhhBYXLgyMnNYOwQtvxmbg2TN7S8WdQp
VZAGdIF7auC2/2Ke6OH6Ht2HwIXwv8CcigO5Idl26wctba9KTe8BgYbpBhb2KU/n
s7LhU6XNkU2pXPkeNQ/YHa7qQnVoTtnzn8g1ooaKLTtfWaeKZQdP80JrdGK259kG
IFOPTzYOBxep2lxKWvL1rAR1LhYGp55exsAfPPJMDvEmpW8cmZeD0C9z5BmZWXGc
HnQ0P/xWvTIAG2Sef4al/P5pCoxuxeDPzaXKkOe0UVIQY7Kl0bjhOi6gvWzObapP
Kd5UoyT0qmQ1VU4yUSFNyF/0eZdvwKprKErvLJ9AOmGlpV1F62e7JxbjjTDXoO23
i3i/M5jAUs9SxFMdOuFDFcMl2Wn3ABxCQhKJNURUo/F2y1zf9fQHImSbirb9QNj3
A33bs3wxgc+0iLu6+cOIv+CSVvzdmv68vgfE+8Y0VnM7Dp2u10JBofJfltu3zdwb
0hqFAgMBAAGjggEZMIIBFTAPBgNVHRMECDAGAQH/AgEBMAsGA1UdDwQEAwIBhjAd
BgNVHQ4EFgQU7RB+rbrra2LW7wpmN4ES2WVQ86QwHwYDVR0jBBgwFoAUZzKtU6Z0
wXqJ5WssX2JDjSoMllYwgbQGA1UdHgEB/wSBqTCBpqCBozAIggYubG9jYWwwEYIP
bWVzaHNoaWVsZC5jb3JwMBKCEC5tZXNoc2hpZWxkLmNvcnAwEYEPbWVzaHNoaWVs
ZC5jb3JwMCWBIy5tZXNoc2hpZWxkLmNvcnA7RE5TOm1lc2hzaGllbGQubGFuMBGC
Dy5tZXNoc2hpZWxkLmxhbjAQgQ5tZXNoc2hpZWxkLmxhbjARgQ8ubWVzaHNoaWVs
ZC5sYW4wDQYJKoZIhvcNAQELBQADggGBABexaRQkvPerVH8OeM4odG+tWwlz7ssj
GkA427NRTNXn90BMf8T5w6bkZjus0SkHkDBA1I8YVoDe7I4J4cJHS4CJNkuxa3Vk
DonKOrAQ8x6C+hPFj4PjEHQymDdlRGF0HuVl9A+JILY2wKW5LWIU+vJ8VtXrEy3H
y2rj0VQCj+DIDVeYdyhjNcpJAkt7KaR5JRFnHFmKPNPLW0ZzD9bi461YMKY+oCtP
9NkL0VlE3l/rAme/zsUgKK5xgQbgN9dUPgkzyp1oFdZ45mMUXQA/8lQZscREi1ku
qZqJMUpzsPXdA3mxyoNtToVm88zptzTNVgkgO1dj9Wb+HHz+3MLuWKLg6HhlO8pA
sjC7x6xz3YR6TW5w90MbMg4x0S+Erp14Tf7y2TfHNQTPgU2hQ/G0evyz+3Rjlj1K
a849VIEDm/mG8xFt49mA1EyJvBQmOAbYYiYA8+fTI2ToVDK6iaxS2jSHxj3GiOWV
veYsLr/x0XOaTUT8mTUnpk9Uip556XbMeQ==
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIEJTCCAo2gAwIBAgIUaMz2xtLsR7emsSF3mGXrKlJAXS4wDQYJKoZIhvcNAQEL
BQAwIjEgMB4GA1UEAwwXTWVzaCBTaGllbGQgUlNBIFJvb3QgQ0EwHhcNNzAwMTAx
MDAwMDAxWhcNMjUwMzE4MDAwMDAxWjAiMSAwHgYDVQQDDBdNZXNoIFNoaWVsZCBS
U0EgUm9vdCBDQTCCAaIwDQYJKoZIhvcNAQEBBQADggGPADCCAYoCggGBAOne87LI
PDb2+x+PQZP4bpGTcO+9RIURydHt7cYVO73Y1ALyxHqqaHsIMhUW9DbU69OZDyFY
NFNOfMfhgrvUCugLtqtkK66F2eqx4f+/aW84NjSmhRqeR77K2OFSvWM48ds5AzkT
Hf/KMxN2eo5JD4vJz7sOgt/b3kGp3TssC2gBt1NGtA377JP/c1BszuFGa555p+Ba
ulByNYq0R+iXaPBM3MPq0hnMFWpdDlkCWmTyqdeTi1CiecbfiBdBkA8WzsPS+ME2
xXEttFDngIDnkN4urMWJFrv7dvnKOz+6OeIy4ZzGnd6jVJfpx5au2ysFM9RWjXn6
Q5lukPjpfnVK/KczqssQLzl/sU7g0OhHTd6G7Vl1Ml6OFgwa4Zj1OtgwZZx4WDLD
R+7yI0l8W1RFz5XjcmYpGKrjnE9eTs2RwzWgo0dFVpi8gbzcvr3hW1PwojFKOf7k
T4/23HNL/sPK221rNqDBcqMiD2Wz6zGJIspZayepG1WCmVvUh3utjGByJQIDAQAB
o1MwUTAPBgNVHRMECDAGAQH/AgECMB0GA1UdDgQWBBRnMq1TpnTBeonlayxfYkON
KgyWVjAfBgNVHSMEGDAWgBRnMq1TpnTBeonlayxfYkONKgyWVjANBgkqhkiG9w0B
AQsFAAOCAYEAUWpmCUupdCsN0HrsXo+ib378xvNwycZi6wm5hF9cicrrfNGTdwAV
rdvrPMdJudeJBdkTJ834Tht7JzLrc4zDDKViOMbK/GRjZNJcCL1CRp1REGq77WPz
iqsuwxRdPfhvbjrXOW5o927dM+0JYA5ar9agjaRhvjeSqwZrRQzY2OSFVTVy46dt
/0TBTksGJyudKI93L1pZ+hxHyoF1U8JsSSwltiP/KqFQGAk8hsTrrWO6B0FmE87O
Wkkfc3RXkAxF/7Ia5qY6DwhjxvnFQ2amiyHZYFn0h+dbw032/Z2uFCUOgeTwRn9n
8I0NxobttYW8VAf/Mna+tQDZSyFO8dOHBBFCygC+Dsegqg9vwjpZQInbLgtMD8QW
i3JDhfN3UiO5VJrQiq6RnAB/wM7NZrgbsAXHTxA3sJgiFxJUa5FV8yHtuCg0EH9b
xS+H3dCsCdAtdzQvoqMlJJjW9C55mLe7zl/gI4PGC+miTLdpmqQl5fXx/chwfaS7
hf+GHSZjVBr/
-----END CERTIFICATE-----
#key-ecdsa
-----BEGIN PRIVATE KEY-----
MIGHAgEAMBMGByqGSM49AgEGCCqGSM49AwEHBG0wawIBAQQgbqbVENsv4/gPBadV
Xq7GSHnZjCx4hxsSPwRP/jncUQahRANCAAR87Z+CCTFQ01K7ilOYOefNKlj5rfwy
i7CFmVvJ9hzuiPwjnpGEXCpZPnm+f/px4CvNM5T0TWZlOrqR89vOTr+e
-----END PRIVATE KEY-----
#key-eddsa
-----BEGIN PRIVATE KEY-----
MC4CAQAwBQYDK2VwBCIEIKUcLijimIccrEUPyLNFWrwoStob6DUu2AJKO0Vw0j/X
-----END PRIVATE KEY-----
#key-rsa
-----BEGIN PRIVATE KEY-----
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
p0EaWnQNoQG7DMOHnkOL7a0=
-----END PRIVATE KEY-----
