#!/bin/bash


DEFAULT_KEYPAIR_TYPE="ecdsa"    # Can be ecdsa, eddsa, or rsa
DEFAULT_DOMAIN="meshshield"

DEFAULT_DEBUG=0


#####################################


usage() {
    echo "[+] Usage: $0 <interface | fqdns> [<interface> ...]"
    echo
}

if [ $# -eq 0 ]; then
    usage
    exit 1
fi

if [[ "${@^^}" =~ 'FQDNS' ]]; then
    set -- ${@//[Ff][Qq][Dd][Nn][Ss]/}
    GENERATE_FQDNS=1
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


[ "${DEBUG:-$DEFAULT_DEBUG}" = "0" ] && unset DEBUG || DEBUG=/dev/fd/1

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

certify_fqdn() {
    FQDN="$1"

    echo "[+] Generating certificates for $FQDN"

    issue_filebased_certificate "$FQDN" 'DNS'
}

process_all_fqdns() {
    certify_fqdn "$SUBJECT_NAME.local"
    certify_fqdn "$SUBJECT_NAME.$DEFAULT_DOMAIN.corp"
    certify_fqdn "$SUBJECT_NAME.$DEFAULT_DOMAIN.lan"
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

            [ "$GENERATE_FQDNS" != "1" ] || process_all_fqdns

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
MIIChDCCAimgAwIBAgIUICQHIxAoMIJhEGBw/4E6M9Sok5AwCgYIKoZIzj0EAwIw
LDEqMCgGA1UEAwwhTWVzaCBTaGllbGQgRUNEU0EgSW50ZXJtZWRpYXRlIENBMB4X
DTcwMDEwMTAwMDAwMVoXDTI1MDExOTAwMDAwMVowOjE4MDYGA1UEAwwvTWVzaCBT
aGllbGQgRUNEU0EgZmlsZWJhc2VkIFNlY3VyaXR5IE9mZmljZXIgQ0EwWTATBgcq
hkjOPQIBBggqhkjOPQMBBwNCAATELOCMEN9E3YPOid3oFk0ZdlXFUPJvu+2WgZg4
SMLQzAP+ogbUMk2/6UvosxzW4yyyDUn0U8aAAKFBugucpHlYo4IBGTCCARUwDwYD
VR0TBAgwBgEB/wIBADALBgNVHQ8EBAMCAYYwHQYDVR0OBBYEFBe3wocw3vLgPcS7
G3D+t86yeg8oMB8GA1UdIwQYMBaAFN3StJaYPJpu/gu4KX//SnIbtflEMIG0BgNV
HR4BAf8EgakwgaaggaMwCIIGLmxvY2FsMBGCD21lc2hzaGllbGQuY29ycDASghAu
bWVzaHNoaWVsZC5jb3JwMBGBD21lc2hzaGllbGQuY29ycDAlgSMubWVzaHNoaWVs
ZC5jb3JwO0ROUzptZXNoc2hpZWxkLmxhbjARgg8ubWVzaHNoaWVsZC5sYW4wEIEO
bWVzaHNoaWVsZC5sYW4wEYEPLm1lc2hzaGllbGQubGFuMAoGCCqGSM49BAMCA0kA
MEYCIQDpso6bEwdGltNIc26WXLiBD4ot+Hch0qFDhItSxtCdqgIhAOQRP2J6mL3h
4QefQVfxmOPrFHTQ/ZmVVoSQbIpQE5bx
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIICbTCCAhOgAwIBAgIUICQHIxAoFigkZwEgqHCdjupJs8QwCgYIKoZIzj0EAwIw
JDEiMCAGA1UEAwwZTWVzaCBTaGllbGQgRUNEU0EgUm9vdCBDQTAeFw03MDAxMDEw
MDAwMDFaFw0yNTA3MjMwMDAwMDFaMCwxKjAoBgNVBAMMIU1lc2ggU2hpZWxkIEVD
RFNBIEludGVybWVkaWF0ZSBDQTBZMBMGByqGSM49AgEGCCqGSM49AwEHA0IABFhM
lEwUDE7RdySAA+YVMqpXEESPckHEerrYDLjAJ6LZ4mdTqWBNluVj1e7IdzrCagwJ
Do5THFp8YlJdDO7tWRCjggEZMIIBFTAPBgNVHRMECDAGAQH/AgEBMAsGA1UdDwQE
AwIBhjAdBgNVHQ4EFgQU3dK0lpg8mm7+C7gpf/9Kchu1+UQwHwYDVR0jBBgwFoAU
Bwdh0I2hfBka6kIOVF7h1gyfNXIwgbQGA1UdHgEB/wSBqTCBpqCBozAIggYubG9j
YWwwEYIPbWVzaHNoaWVsZC5jb3JwMBKCEC5tZXNoc2hpZWxkLmNvcnAwEYEPbWVz
aHNoaWVsZC5jb3JwMCWBIy5tZXNoc2hpZWxkLmNvcnA7RE5TOm1lc2hzaGllbGQu
bGFuMBGCDy5tZXNoc2hpZWxkLmxhbjAQgQ5tZXNoc2hpZWxkLmxhbjARgQ8ubWVz
aHNoaWVsZC5sYW4wCgYIKoZIzj0EAwIDSAAwRQIhAO2byWbtG/sh4mOrjdNu9a0I
BUR7EGSic5xT2UFt11dXAiB/Cc4OhWowLWFZhygnNJzGaR0QcmZQYb8Uo1iOw3SO
CA==
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIBnjCCAUOgAwIBAgIUEBsgoYH0sHymeBKJT6BsoE/KWqwwCgYIKoZIzj0EAwIw
JDEiMCAGA1UEAwwZTWVzaCBTaGllbGQgRUNEU0EgUm9vdCBDQTAeFw03MDAxMDEw
MDAwMDFaFw0yNTA3MjMwMDAwMDFaMCQxIjAgBgNVBAMMGU1lc2ggU2hpZWxkIEVD
RFNBIFJvb3QgQ0EwWTATBgcqhkjOPQIBBggqhkjOPQMBBwNCAAQGQr17URxfNbi5
wHGsh5e97H65LvoNoElebspA5hirX+m1sTgTQvcrry/gwQTwfWkq6lBNCwSXF90Q
/b57Hj13o1MwUTAPBgNVHRMECDAGAQH/AgECMB0GA1UdDgQWBBQHB2HQjaF8GRrq
Qg5UXuHWDJ81cjAfBgNVHSMEGDAWgBQHB2HQjaF8GRrqQg5UXuHWDJ81cjAKBggq
hkjOPQQDAgNJADBGAiEApWnB7MnZLThzGlSR6p1q0Dc/5TL1EtRUP7++yxZpyLsC
IQDyjv/z5VTaVNZBfJWSYL82RkfLgeZjt7rK7DE9up/v1g==
-----END CERTIFICATE-----
#eddsa
-----BEGIN CERTIFICATE-----
MIICQzCCAfWgAwIBAgIUICQHIxAoMmBoJSGSOrFUCMlkAGcwBQYDK2VwMCwxKjAo
BgNVBAMMIU1lc2ggU2hpZWxkIEVkRFNBIEludGVybWVkaWF0ZSBDQTAeFw03MDAx
MDEwMDAwMDFaFw0yNTAxMTkwMDAwMDFaMDoxODA2BgNVBAMML01lc2ggU2hpZWxk
IEVkRFNBIGZpbGViYXNlZCBTZWN1cml0eSBPZmZpY2VyIENBMCowBQYDK2VwAyEA
J5uOBUapmCIf0f8H9Zbnu+3B5uGTCDVVwsDVu8cGP3yjggEZMIIBFTAPBgNVHRME
CDAGAQH/AgEAMAsGA1UdDwQEAwIBhjAdBgNVHQ4EFgQUDCRpKDnrqWtR/nCJrAAW
1vidTJUwHwYDVR0jBBgwFoAURkLl3H4L3Yc0jCiqyPtX8XZ0EuwwgbQGA1UdHgEB
/wSBqTCBpqCBozAIggYubG9jYWwwEYIPbWVzaHNoaWVsZC5jb3JwMBKCEC5tZXNo
c2hpZWxkLmNvcnAwEYEPbWVzaHNoaWVsZC5jb3JwMCWBIy5tZXNoc2hpZWxkLmNv
cnA7RE5TOm1lc2hzaGllbGQubGFuMBGCDy5tZXNoc2hpZWxkLmxhbjAQgQ5tZXNo
c2hpZWxkLmxhbjARgQ8ubWVzaHNoaWVsZC5sYW4wBQYDK2VwA0EAIrslHs3imcOs
ZNZhTsWCKYlCfOy5/in76RybZE+cGqfFqz+e3msoU5QXlAbP4hKapnZIAZ9DYBH/
FEjYR1XdAQ==
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIICLTCCAd+gAwIBAgIUICQHIxAoFkl2gGCB6DSVQH3xX8swBQYDK2VwMCQxIjAg
BgNVBAMMGU1lc2ggU2hpZWxkIEVkRFNBIFJvb3QgQ0EwHhcNNzAwMTAxMDAwMDAx
WhcNMjUwNzIzMDAwMDAxWjAsMSowKAYDVQQDDCFNZXNoIFNoaWVsZCBFZERTQSBJ
bnRlcm1lZGlhdGUgQ0EwKjAFBgMrZXADIQA8b96J9N5EOa48/PnxhRrxkptin23C
22SCahRkmK4YA6OCARkwggEVMA8GA1UdEwQIMAYBAf8CAQEwCwYDVR0PBAQDAgGG
MB0GA1UdDgQWBBRGQuXcfgvdhzSMKKrI+1fxdnQS7DAfBgNVHSMEGDAWgBT5c9GS
iId8CQh7Vg4XQtLjfSCHNTCBtAYDVR0eAQH/BIGpMIGmoIGjMAiCBi5sb2NhbDAR
gg9tZXNoc2hpZWxkLmNvcnAwEoIQLm1lc2hzaGllbGQuY29ycDARgQ9tZXNoc2hp
ZWxkLmNvcnAwJYEjLm1lc2hzaGllbGQuY29ycDtETlM6bWVzaHNoaWVsZC5sYW4w
EYIPLm1lc2hzaGllbGQubGFuMBCBDm1lc2hzaGllbGQubGFuMBGBDy5tZXNoc2hp
ZWxkLmxhbjAFBgMrZXADQQByKJPLnogg9Kj0Mu62ny9E2gFhdG5sbMvoAA0LVmqR
G6LEW2ydf0W47XXa+yN0/kHQcq/nXJkZyP7qsyP/1FoL
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIBXTCCAQ+gAwIBAgIUJzSdOTfyLRmGypvO4v9fH1iJ7+cwBQYDK2VwMCQxIjAg
BgNVBAMMGU1lc2ggU2hpZWxkIEVkRFNBIFJvb3QgQ0EwHhcNNzAwMTAxMDAwMDAx
WhcNMjUwNzIzMDAwMDAxWjAkMSIwIAYDVQQDDBlNZXNoIFNoaWVsZCBFZERTQSBS
b290IENBMCowBQYDK2VwAyEARXFxzqCRAjgprlHHYELjYf9zyAF5rwVDlfVCGHRZ
ddajUzBRMA8GA1UdEwQIMAYBAf8CAQIwHQYDVR0OBBYEFPlz0ZKIh3wJCHtWDhdC
0uN9IIc1MB8GA1UdIwQYMBaAFPlz0ZKIh3wJCHtWDhdC0uN9IIc1MAUGAytlcANB
AHeR/g9TXCBxW7pGGybvVhQUVxz9VUophygTLOkKbyRin+LeoCNZmkLgMBwr/zwk
1nnvWCa026541BKZzOIeNQo=
-----END CERTIFICATE-----
#rsa
-----BEGIN CERTIFICATE-----
MIIFCzCCA3OgAwIBAgIUICQHIxAoNFQlEmBEmgA7NDgI4mYwDQYJKoZIhvcNAQEL
BQAwKjEoMCYGA1UEAwwfTWVzaCBTaGllbGQgUlNBIEludGVybWVkaWF0ZSBDQTAe
Fw03MDAxMDEwMDAwMDJaFw0yNTAxMTkwMDAwMDJaMDgxNjA0BgNVBAMMLU1lc2gg
U2hpZWxkIFJTQSBmaWxlYmFzZWQgU2VjdXJpdHkgT2ZmaWNlciBDQTCCAaIwDQYJ
KoZIhvcNAQEBBQADggGPADCCAYoCggGBAKmLWkFFaRLM0iNOpB+YYirfRm+51+s3
LWoEtu3LdvNhYQuSA2d2TIktsvaIhXIYwsYx68cMaj5TnS/6nFajxmZ35kSx0kPM
zTz5JHRhhF8tPLpq4Hs1TuIvz0f112iwx5nYvDWpSpC31xYd2vhBTDYWuilriDqB
Twnyps/NY/lYjf9EaG1dFNv176Yx2YYO4b7+uvYOmmEzjK8f3R4usIbVOCF7m17A
/jifpqSWPeYPx7jve4JlBSOpBPZiKOpjaviDzzqubqAlTDVqgvf0It3Tp04eQAYu
i4cWw4ZnSryFJzeLWVA4DWM4AiXXm6x9nyGD47St+DXtmJgey17LB8akd7yabRXz
AOri6UESTr5CVfxr6Q+K9VKLrFuc+rJtxs055ThcQJ7IItI2L0YtQBdBvuaTGgNx
krU8y7dLCMHGBGQplO0FtmScW/54PF9RTocpYMqX53iWErJqJIUD+pxFOcYdunhd
i+nIaJ5FfbTP1nnRpgEKH5DozluSzZLYewIDAQABo4IBGTCCARUwDwYDVR0TBAgw
BgEB/wIBADALBgNVHQ8EBAMCAYYwHQYDVR0OBBYEFNPOKuUyDKTUji7buzI+d9Fq
dTvxMB8GA1UdIwQYMBaAFD1CUi1EcSYOWH5NDWOxPwT9AoFFMIG0BgNVHR4BAf8E
gakwgaaggaMwCIIGLmxvY2FsMBGCD21lc2hzaGllbGQuY29ycDASghAubWVzaHNo
aWVsZC5jb3JwMBGBD21lc2hzaGllbGQuY29ycDAlgSMubWVzaHNoaWVsZC5jb3Jw
O0ROUzptZXNoc2hpZWxkLmxhbjARgg8ubWVzaHNoaWVsZC5sYW4wEIEObWVzaHNo
aWVsZC5sYW4wEYEPLm1lc2hzaGllbGQubGFuMA0GCSqGSIb3DQEBCwUAA4IBgQCZ
aro1l5QZ75IeABG1iTwDLygtwicOZ1W+IbIHKq3KnzgTLOhdazwIbZVks1EGq7EX
BZ5ZCLm1edzjgixL0cjRG9Kj9e3Ryfk3DDKk2/6bBkymztFQaSlwfsEbNxJQaqGa
WzkDWEyI6bKQbint4lFNtqx+LZIwmWq/HMonWILVcrCy8lT95X49fNc+Q3tYIj07
waCZTUtRCTmHIoaUk5vE219v5jIAXfFNe57ki39/v8a9VRaMWorDIg3KyLtMfnRf
VgQ19d1EY/+3Jv+JjPpmka1Dr9uNzqedsZrLUq7R2GaGhA0Jz5L0TDhLcNsQSNfw
dZMnAAnXAdGVoPPlQ5nCwKf/Q3ndVA0yxvRM0L2w2dRkU2Z6merewx2bPfFPRR9V
75r+d6UUTk+kxwKa1qMsma4sqEhlre8DtmCGY/ljAzOnS309NAwC6L/n9zzyZ0XC
ZmcrYXqjnialsc66O8ALWFLJnEiuUv9X+OZC0jICBWGGJ6oUzEbL9g8U7jC+5j4=
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIE9TCCA12gAwIBAgIUICQHIxAoFnFnl4ONdMd1VKQoNRAwDQYJKoZIhvcNAQEL
BQAwIjEgMB4GA1UEAwwXTWVzaCBTaGllbGQgUlNBIFJvb3QgQ0EwHhcNNzAwMTAx
MDAwMDAxWhcNMjUwNzIzMDAwMDAxWjAqMSgwJgYDVQQDDB9NZXNoIFNoaWVsZCBS
U0EgSW50ZXJtZWRpYXRlIENBMIIBojANBgkqhkiG9w0BAQEFAAOCAY8AMIIBigKC
AYEAvCV929n+BSCJ60ahRd5UbuLjqvKc934zcTSLaa3wx24UlvvP0VRI5lbjGu05
F/WnEAonb8hXkzwx0h/z7jlEZ5XAAD+P84E2X155EUIcPZB1AM6PuCO8BhsQSny5
MaLBxhdZhq8iZsI1qa8cZZoj5SevsPnIOZSIUPUwO+4ldhetDGQjb0Zp5sDAg0JN
2OVRqGr5Byi/Tkrq2qW2x/aMmFMU1204JNLRpWN2tj9RF8wNphgiHClvnFODDrx3
3m12G3aMm+/ATCiRnteHtXOKlXURlxV9WMo2K9AkHZe1rvwWArQwfYaesTPqOSnX
uO6gEDZqjfuyOegN9cdTk51h9kzfp7x+WXSQoUA+3mdCqulydMlXrjCXWC3+0Lrs
4zS1Y6+e9quhiekZnyYmbkAj9JKtoZLntopUVPugX3bkHQNYPHMV8rDcFjotsEpV
xzGiPh/iugEiBcURvJQtyi9uwbxPVd92DQpOxnQoP0RfXDmzGmbaqz8IrTgy1bZs
g5vPAgMBAAGjggEZMIIBFTAPBgNVHRMECDAGAQH/AgEBMAsGA1UdDwQEAwIBhjAd
BgNVHQ4EFgQUPUJSLURxJg5Yfk0NY7E/BP0CgUUwHwYDVR0jBBgwFoAUwApw1cqg
5Kd6gsg7VEcPzRi9jEUwgbQGA1UdHgEB/wSBqTCBpqCBozAIggYubG9jYWwwEYIP
bWVzaHNoaWVsZC5jb3JwMBKCEC5tZXNoc2hpZWxkLmNvcnAwEYEPbWVzaHNoaWVs
ZC5jb3JwMCWBIy5tZXNoc2hpZWxkLmNvcnA7RE5TOm1lc2hzaGllbGQubGFuMBGC
Dy5tZXNoc2hpZWxkLmxhbjAQgQ5tZXNoc2hpZWxkLmxhbjARgQ8ubWVzaHNoaWVs
ZC5sYW4wDQYJKoZIhvcNAQELBQADggGBALqaZXqO27Z0rQKUCkOU5Y8uAQKI0n6O
avGc7KxHutwc1ndFRPfbR62gwbWIHsk49GYw/CqdvbW85Mus7WyN0fWufh3/cmV8
8fbylOAPjSZMmIg33ft8zMCjYpbpkKO3+ynsZ7VkmSZhRoGfObkrybPcEgpYuvOd
hxJ30j+rfwFiBilZKV9wgvFn1cxSkoZZFAp+S+DvOPzx6ir4BP8FQzgrnw3LI3Rh
Q7YBkbg5E3TOG5TyUasCuRthlkWWWgIjjEzQWd8vLimB9MJGVAR0xeysQP1dA2fT
RQZ2wmeY4yFAZ0bazsCuNIGyxDfgAfCPOdx+raNMeqAYpKz4KksYHiSFHKK5CYaF
D++CZu+mbEuOImBzjqCZpb/awBbzqzLwiIecRJrEPOktWEOLPYxWjbrQGAgUasW4
ac5elLxxYLUzt46aF7fpvwVvfoZUyPJmhbREVmBFuunk8rxnWZj1s2e4hGCHX4Ya
uNKLx14b3YN6ZX8wiyhpqfoNZRMQ9QxzaQ==
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIEJTCCAo2gAwIBAgIUGC+qMYf5mGK4cR2ubIxv9MQCfp8wDQYJKoZIhvcNAQEL
BQAwIjEgMB4GA1UEAwwXTWVzaCBTaGllbGQgUlNBIFJvb3QgQ0EwHhcNNzAwMTAx
MDAwMDAyWhcNMjUwNzIzMDAwMDAyWjAiMSAwHgYDVQQDDBdNZXNoIFNoaWVsZCBS
U0EgUm9vdCBDQTCCAaIwDQYJKoZIhvcNAQEBBQADggGPADCCAYoCggGBAMiW9gtI
n6dBXAtXG5DV1JwgEhGE/czwiWk7PIH9hr/NzLAb+aODagiLTU44kVL5VhgMsVFC
FSGeeyl9QwtX5XNv9hINqWj14ux0+082XLl9J2HHarpJif8uCreZ5XyzbATTgx3m
c1mJtAgM5VsQF5xYxT5mdg5heh5Xx+nC/xzp6lWCJAOIFFxmLKteUSbg72J4M4aA
jkfwJEKoTY5ZUFQkPXJ0rmC980XNkuRsr4x/yj9xjhODFEsrs1iiSh7NU9eaEO5w
tROwFRW3oLFNvgxLI4gMU+rlD+vZzlULerQvqiwZaVmbNpwl0ltTc11+NjSw8qPY
TuIGdt4cWUEZ6BqjyQTXTzCqvV0Dxi6dA2P5m5FL14NIxox2ihi4nLpDuq+OaYM/
y8y91fdZhRmFvotNa4bOfLkncvAuA4HeSoc4LSFk9hWCjrZOZ2YfvcxnYCHppxHn
SlVsE6FFrp9DSx/DWjkqMARROo/HLlVX8SIWMr0N5XfMsz2Pqm1bop7y3wIDAQAB
o1MwUTAPBgNVHRMECDAGAQH/AgECMB0GA1UdDgQWBBTACnDVyqDkp3qCyDtURw/N
GL2MRTAfBgNVHSMEGDAWgBTACnDVyqDkp3qCyDtURw/NGL2MRTANBgkqhkiG9w0B
AQsFAAOCAYEAcQo6DP/PPzTr73VACMlmrNB1ycyUceAHN577cBfl47VOo9hVnQNm
PN7caaYuMi32p0feKMTaw1n04cmutZlfj8q6ztJOpwxpZGug9pV5nq69zEFBiwwK
B7/QS/Zyp+sK4NMw1Oa+WfB7d9oWFmvviPgrxOdwLmqzU8DKl/Y50mYnbA/jCTZL
yZqRpnue87Jtv+kVw57aBzX9zBXgCzz9jfTb8IJxbHcuu/C5iHNxn93FQR1tAWI2
ELhFOOHnY3hw9C9LLszAmpRjPVHypKjv0mHjnyZ9EobBjW9ehkgn8GFUlxox83ik
sUoRaSBU3C+em6qLxrIz2db3GawqpRTYci1T4aezf5qiRObCFt2p5X2zTajOpNKX
yor4hirLFcP5omGsLVOf9SzPYP1dqMWiv/2IZ/bpp+Yh2LcqPr9UmR9aOoDOfGLt
qUV23bVa+o4nHCx195y0z8Bmxrm7zWo2MThMgDANNQZ2YQpHmknJEHUIw8/j//YP
FJK8H+tq1ZJv
-----END CERTIFICATE-----
#key-ecdsa
-----BEGIN PRIVATE KEY-----
MIGHAgEAMBMGByqGSM49AgEGCCqGSM49AwEHBG0wawIBAQQgZDHfgDpvipNNZH/F
wqyXaL8rJ0Slbbkk5u9w3rbWElGhRANCAATELOCMEN9E3YPOid3oFk0ZdlXFUPJv
u+2WgZg4SMLQzAP+ogbUMk2/6UvosxzW4yyyDUn0U8aAAKFBugucpHlY
-----END PRIVATE KEY-----
#key-eddsa
-----BEGIN PRIVATE KEY-----
MC4CAQAwBQYDK2VwBCIEICINcL9XJm0hh0/dQ4e2aU6PKr3lSSxcPmHBcPjpKrmO
-----END PRIVATE KEY-----
#key-rsa
-----BEGIN PRIVATE KEY-----
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
xJtlkt8CPw0tiuMIdnBy8P+T
-----END PRIVATE KEY-----
