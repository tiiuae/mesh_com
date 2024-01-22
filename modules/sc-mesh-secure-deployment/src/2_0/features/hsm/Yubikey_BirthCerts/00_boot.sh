#! /bin/bash

CRYPTO='./crypto'
HWINFO='./hwinfo'
MSPKI='./mspki'
LIBSOFTHSM2='/usr/lib/softhsm/libsofthsm2.so'
LIBYUBIKEY='/usr/local/lib/libykcs11.so'
SOFTHSM2_PIN='To be protected by SB and FDE'
SOFTHSM2_SO_PIN='May not even need to be here'
IPV6_ULA_PREFIXES='
	fd1d:8a65:4c32:e330
	fdbd:5175:25cd:bbf3
	fd2e:868e:e806:a27a
	fde3:049c:9f74:4742
	fd1e:0156:d99b:f7a6
	fd4c:d61a:7cac:1bda
	fd70:8347:8f1e:f1d5
	fddf:38d8:ad39:8410
	fd72:9c2f:9a73:9c6a
	fdcd:8b8f:5718:72ec
	fd8d:bbf3:2ee6:c338
	fd55:2dfa:c984:9efb
	fd07:4c4f:b7e0:1f8b
	fd0d:3f43:859f:ccd8
	fd7d:92af:abbc:cd4f
	fd22:e964:ba05:e603
	fdf7:4177:ea03:30fe
	fd54:4bad:e5fb:2b23
	fdd8:84dc:fe30:7bf2
	fdbb:1ef7:9d6f:e05d
'

ECDSA_ROOT_CA_CRT="$MSPKI/ecdsa/root.crt"
RSA_ROOT_CA_CRT="$MSPKI/rsa/root.crt"

ECDSA_INTERMEDIARY_CA_CRT="$MSPKI/ecdsa/intermediary.crt" 
RSA_INTERMEDIARY_CA_CRT="$MSPKI/rsa/intermediary.crt" 

TEMPORARY_ECDSA_SECURITY_OFFICER_CA_CRT="$MSPKI/ecdsa/security_officer.crt"
TEMPORARY_RSA_SECURITY_OFFICER_CA_CRT="$MSPKI/rsa/security_officer.crt"

PIN="12345678"
create_softhsm2()
{
	KEYPAIR_TYPE="$CURRENT_KEYPAIR_TYPE"

	case "$KEYPAIR_TYPE" in
		ecdsa)	HSM_KEYPAIR_TYPE='EC:prime256v1';;
		rsa)	HSM_KEYPAIR_TYPE='RSA:2048';;
		*)	>&2 echo "Unsupported keypair type '$KEYPAIR_TYPE'"; exit 255;;
	esac

	HSM_LABEL='BirthKeys'
	TOKENS_DIR="$CRYPTO/$KEYPAIR_TYPE/birth/softhsm2"
	mkdir -p "$TOKENS_DIR"
	export SOFTHSM2_CONF="$TOKENS_DIR/softhsm2.conf"
	cat > "$SOFTHSM2_CONF" <<- EOF
		directories.tokendir = $TOKENS_DIR
		objectstore.backend = file

		# ERROR, WARNING, INFO, DEBUG
		log.level = INFO

		# If CKF_REMOVABLE_DEVICE flag should be set
		slots.removable = false

		# Enable and disable PKCS#11 mechanisms using slots.mechanisms.
		slots.mechanisms = ALL

		# If the library should reset the state on fork
		library.reset_on_fork = false
	EOF

	# FIXME: PINs/passwords shouldn't be passed on the CLI as it makes them easily visible in the process list
	softhsm2-util --init-token --free --label "$KEYPAIR_TYPE" --pin "$SOFTHSM2_PIN" --so-pin "$SOFTHSM2_SO_PIN"
	pkcs11-tool --modul "$LIBSOFTHSM2" --label "$KEYPAIR_TYPE" --pin "$SOFTHSM2_PIN" --keypairgen --key-type "$HSM_KEYPAIR_TYPE" --id 01
	export -n SOFTHSM2_CONF
}

generate_filebased_private_key()
{
        OUTPUT="$1"
        KEYPAIR_TYPE="$CURRENT_KEYPAIR_TYPE"

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
        KEYPAIR_TYPE="$CURRENT_KEYPAIR_TYPE"

        UPPERCASE_KEYPAIR_TYPE=`echo "$KEYPAIR_TYPE" | tr '[:lower:]' '[:upper:]'`
        TEMPSOCA_CRT="TEMPORARY_${UPPERCASE_KEYPAIR_TYPE}_SECURITY_OFFICER_CA_CRT"
        TEMPSOCA_KEY="TEMPORARY_${UPPERCASE_KEYPAIR_TYPE}_SECURITY_OFFICER_CA_KEY"
        SERIAL=$( echo `date '+%Y%m%d%H%M%S%N'``dd if=/dev/urandom bs=9 count=1 status=none | xxd -p` | cut -c -40 )
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
        )

        # Intentional: keeping the CSRs for future use
        # ) && rm "$CSR"
}




yubikey_security_officer_signs_csr()
{
	CSR="$1"
	CRT="$2"
	KEYPAIR_TYPE="$CURRENT_KEYPAIR_TYPE"
	UPPERCASE_KEYPAIR_TYPE=`echo "$KEYPAIR_TYPE" | tr '[:lower:]' '[:upper:]'`        
        case "$UPPERCASE_KEYPAIR_TYPE" in 
                ECDSA)          SECURITY_OFFICER_KEY="pkcs11:object=PIV AUTH key";;
                RSA)            SECURITY_OFFICER_KEY="pkcs11:object=SIGN key";;
                *)      >&2 echo "Unsupported keypair type '$KEYPAIR_TYPE'"; exit 255;;
        esac
	TEMPSOCA_CRT="TEMPORARY_${UPPERCASE_KEYPAIR_TYPE}_SECURITY_OFFICER_CA_CRT"
	SERIAL=$( echo `date '+%Y%m%d%H%M%S%N'``dd if=/dev/urandom bs=9 count=1 status=none | xxd -p` | cut -c -40 )
        openssl x509 -req -CAkeyform engine -engine pkcs11 -in "$CSR" -out "$CRT" -sha256 -days 90 -req -set_serial "0x$SERIAL" -CA "${!TEMPSOCA_CRT}" -CAkey "$SECURITY_OFFICER_KEY" -passin "pass:$PIN" -extensions default.extensions -extfile <( cat <<- EOF
                [ default ]
                openssl_conf            = default.openssl_conf
                extensions              = default.extensions
                [ default.openssl_conf ]
                oid_section             = default.openssl_conf.oid_section
                engines                 = default.openssl_conf.engines
                [ default.openssl_conf.oid_section ]
                [ default.openssl_conf.engines ]
                pkcs11                  = default.openssl_conf.engines.pkcs11
                [ default.openssl_conf.engines.pkcs11 ]
                engine_id               = pkcs11
                MODULE_PATH             = $LIBYUBIKEY
                PIN                     = "$PIN"
                init                    = 0
                [ default.extensions ]
                basicConstraints        = critical,CA:FALSE
                keyUsage                = critical,digitalSignature,keyEncipherment,dataEncipherment,keyAgreement
                extendedKeyUsage        = critical,serverAuth,clientAuth,ipsecIKE
                subjectKeyIdentifier    = hash
                authorityKeyIdentifier  = keyid:always
                subjectAltName          = "$COMMON_NAME"
                [ default.extensions.policy ]
                commonName              = provided
                subjectAltName          = provided
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
	)
	# ) && rm "$CSR"
}

issue_filebased_certificate()
{
        IDENTITY="$1"
        IDENTITY_TYPE="$2"
        KEYPAIR_TYPE="$CURRENT_KEYPAIR_TYPE"

        FILEBASED_DIR="$CRYPTO/$KEYPAIR_TYPE/birth/filebased"
        mkdir -p "$FILEBASED_DIR"
        PRIVATE_KEY="$FILEBASED_DIR/private.key"
        [ -s "$PRIVATE_KEY" ] || generate_filebased_private_key "$PRIVATE_KEY"

        mkdir -p "$FILEBASED_DIR/$IDENTITY_TYPE"
        CSR="$FILEBASED_DIR/$IDENTITY_TYPE/$IDENTITY.csr"
        CRT="$FILEBASED_DIR/$IDENTITY_TYPE/$IDENTITY.crt"

        case "$IDENTITY_TYPE" in
                MAC)    SUBJECT_ALT_NAME="otherName:1.3.6.1.1.1.1.22;UTF8:$IDENTITY";;
                *)      SUBJECT_ALT_NAME="$IDENTITY_TYPE:$IDENTITY"
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
        ) && root_signs_csr "$CSR" "$CRT"
}


issue_softhsmv2_certificate()
{
	IDENTITY="$1"
	IDENTITY_TYPE="$2"
	KEYPAIR_TYPE="$CURRENT_KEYPAIR_TYPE"

	# FIXME: `openssl req` fails to find the EdDSA privkey for some reason
	[ 'eddsa' = "$KEYPAIR_TYPE" ] && return 0

	TOKENS_DIR="$CRYPTO/$KEYPAIR_TYPE/birth/softhsm2"
	export SOFTHSM2_CONF="$TOKENS_DIR/softhsm2.conf"

	mkdir -p "$TOKENS_DIR/$IDENTITY_TYPE"
	CSR="$TOKENS_DIR/$IDENTITY_TYPE/$IDENTITY.csr"
	CRT="$TOKENS_DIR/$IDENTITY_TYPE/$IDENTITY.crt"

	openssl req -key "pkcs11:model=SoftHSM%20v2;manufacturer=SoftHSM%20project;token=$KEYPAIR_TYPE;object=$KEYPAIR_TYPE;type=private" -keyform engine -engine pkcs11 -new -sha256 -out "$CSR" -config <( cat <<- EOF
		[ default ]
		openssl_conf            = default.openssl_conf
		extensions              = default.extensions

		[ default.openssl_conf ]
		oid_section             = default.openssl_conf.oid_section
		engines                 = default.openssl_conf.engines

		[ default.openssl_conf.oid_section ]
		# If worth the risk of breaking some buggy implementations, we might use some
		# custom OIDs to build our own Certificate Transparency equivalent for example
		# but for now this section is overloading the useless TSA default to clear it.

		[ default.openssl_conf.engines ]
		pkcs11                  = default.openssl_conf.engines.pkcs11

		[ default.openssl_conf.engines.pkcs11 ]
		engine_id               = pkcs11
		MODULE_PATH             = $LIBSOFTHSM2
		PIN                     = "$SOFTHSM2_PIN"
		init                    = 0

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
	) && yubikey_security_officer_signs_csr "$CSR" "$CRT"
	export -n SOFTHSM2_CONF
}



issue_certificate()
{
	issue_filebased_certificate $@
	issue_softhsmv2_certificate $@
}



flip_locally_administered_bit()
{
	MAC_ADDRESS="$1"

	NIBBLE1=`echo "$MAC_ADDRESS" | cut -c 1`
	LOCALLY_ADMINISTERED_FLIPPED_NIBBLE2=`echo "$MAC_ADDRESS" | cut -c 2 | tr '[0123456789abcdef]' '[23016745ab89efcd]'`
	REST_OF_MAC_ADDRESS=`echo "$MAC_ADDRESS" | cut -c 3-`
	echo "$NIBBLE1$LOCALLY_ADMINISTERED_FLIPPED_NIBBLE2$REST_OF_MAC_ADDRESS"
}


mac_to_eui64()
{
	MAC_ADDRESS="$1"

	BYTE1=`flip_locally_administered_bit "$MAC_ADDRESS" | cut -c 1,2`
	BYTE2=`echo "$MAC_ADDRESS" | cut -c 4,5`
	BYTE3=`echo "$MAC_ADDRESS" | cut -c 7,8`
	BYTE4=`echo "$MAC_ADDRESS" | cut -c 10,11`
	BYTE5=`echo "$MAC_ADDRESS" | cut -c 13,14`
	BYTE6=`echo "$MAC_ADDRESS" | cut -c 16,17`
	echo "$BYTE1$BYTE2:${BYTE3}ff:fe${BYTE4}:$BYTE5$BYTE6"
}


certify_ipv6_address()
{
	IPV6_ADDRESS="$1"

	issue_certificate "$IPV6_ADDRESS" 'IP'
}



process_all_ipv6_derivatives()
{
	MAC_ADDRESS="$1"

	EUI64=`mac_to_eui64 "$MAC_ADDRESS"`
	certify_ipv6_address "fe80::$EUI64"
	for PREFIX in $IPV6_ULA_PREFIXES; do
		certify_ipv6_address "$PREFIX:$EUI64"
	done
}



# Assumes the MAC address is all lowercase and colon-separated the UNIX way
certify_mac_address()
{
	MAC_ADDRESS="$1"

	[ "$MAC_ADDRESS" != '00:00:00:00:00:00' ] || return 0
	[ "$MAC_ADDRESS" != 'ff:ff:ff:ff:ff:ff' ] || return 0
	issue_certificate "$MAC_ADDRESS" 'MAC'
	process_all_ipv6_derivatives "$MAC_ADDRESS"
}



process_all_mac_addresses()
{
	find /sys/devices/ -name address | while read ADDRESS_PATH; do
		MAC_ADDRESS=`tr '[:upper:]' '[:lower:]' < "$ADDRESS_PATH"`
		if [ -n "$MAC_ADDRESS" ] && echo "$MAC_ADDRESS" | grep -Eq '^[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}$'; then
			certify_mac_address "$MAC_ADDRESS"
			certify_mac_address `flip_locally_administered_bit "$MAC_ADDRESS"`
		else
			>&2 echo "Error: $ADDRESS_PATH contains '$MAC_ADDRESS' which does not look like a MAC address"
		fi
	done
}



# TODO: find some more human-readable way (e.g. BIP39)
generate_subject_name()
{
	if [ -s /etc/machine-id ]; then
		cat /etc/machine-id
	else
		sha256sum /proc/cpuinfo | cut -c -32
	fi
}



certify_fqdn()
{
	FQDN="$1"

	issue_certificate "$FQDN" 'DNS'
}




process_all_fqdns()
{
	certify_fqdn "$SUBJECT_NAME.local"
	certify_fqdn "$SUBJECT_NAME.meshshield.corp"
	certify_fqdn "$SUBJECT_NAME.meshshield.lan"
}



# TODO: make sure we collect everything that in the future we might wish we had :-)
collect_other_hardware_info()
{
	TREE="$HWINFO/tree"

	mkdir -p "$HWINFO" "$TREE"
	for COMMAND in 'iw phy' 'lspci' 'lsusb' 'lshw' 'ifconfig -a' 'dmesg'; do
		$COMMAND > "$HWINFO/$COMMAND.txt" || true
	done
	for FILE in /etc/machine-id /proc/{cpu,mem}info /sys/firmware/devicetree/base/serial-number; do
		[ -e "$FILE" ] && mkdir -p "$TREE/$FILE" && rmdir "$TREE/$FILE" && cp "$FILE" "$TREE/$FILE"
	done
	return 0
}




# TODO: maybe JSON instead of a tarball, to help MDM?
# Make sure to include the SO certs to help MDM rebuild the chain; maybe include the whole mspki/ folder?
create_bundle()
{
	tar jcvf "$BUNDLE" --null -T <( find "$CRYPTO" -type f -name \*.crt -print0; find "$HWINFO" "$MSPKI" -type f -print0 )
}



# sign tarball/JSON ready for future upload to MDM server, both by the Security
# Officer for authenticity and by the device for non-repudiation.
#
# Actually: keeping the CSRs achieves pretty much the same, might come in handy
# for future cross-signing requirements, and allows a file naming scheme like
# .rsa-sig instead of the uglier .securityofficer-rsa-sig and .device-rsa-sig
sign_bundle()
{
	KEYPAIR_TYPE="$CURRENT_KEYPAIR_TYPE"

	UPPERCASE_KEYPAIR_TYPE=`echo "$KEYPAIR_TYPE" | tr '[:lower:]' '[:upper:]'`
	#TEMPSOCA_KEY="TEMPORARY_${UPPERCASE_KEYPAIR_TYPE}_SECURITY_OFFICER_CA_KEY"
        case "$UPPERCASE_KEYPAIR_TYPE" in 
                ECDSA)          SECURITY_OFFICER_KEY="pkcs11:object=PIV AUTH key";;
                RSA)            SECURITY_OFFICER_KEY="pkcs11:object=SIGN key";;
                *)      >&2 echo "Unsupported keypair type '$KEYPAIR_TYPE'"; exit 255;;
        esac

	#openssl dgst -sha512 -sign "${!TEMPSOCA_KEY}" -out "$BUNDLE.securityofficer-$KEYPAIR_TYPE-sig" "$BUNDLE" # lacks Ed25519 support
	openssl pkeyutl -engine pkcs11 -keyform engine -sign -inkey "$SECURITY_OFFICER_KEY" -out "$BUNDLE.$KEYPAIR_TYPE-sig" -rawin -in "$BUNDLE"
        #openssl pkeyutl -engine pkcs11 -keyform engine -inkey "$SECURITY_OFFICER_KEY" -passin pass:"$PIN" -sign -in data.txt -out data.sig
}

SUBJECT_NAME=`generate_subject_name`

# FIXME: format it as a Distinguished Name?
COMMON_NAME="$SUBJECT_NAME"

for KEYPAIR_TYPE in ecdsa rsa; do
	CURRENT_KEYPAIR_TYPE="$KEYPAIR_TYPE"
	create_softhsm2
	process_all_fqdns
	process_all_mac_addresses
	collect_other_hardware_info
done

BUNDLE="./yubikey_at_birth.tar.bz2"
create_bundle
for KEYPAIR_TYPE in ecdsa rsa; do
        CURRENT_KEYPAIR_TYPE="$KEYPAIR_TYPE"
        sign_bundle
done

