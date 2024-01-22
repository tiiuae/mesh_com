#! /bin/bash -x

set -e



security_officer_csr()
{
	UPPERCASE_ALGORITHM="$1"
	ALGORITHM_VARIANT="$2"

	LOWERCASE_ALGORITHM=`echo "$UPPERCASE_ALGORITHM" | tr '[:upper:]' '[:lower:]'`
	FILEBASE="mspki/${LOWERCASE_ALGORITHM}"
	INTERMEDIATE_KEY="$FILEBASE/intermediate.key"
	INTERMEDIATE_CRT="$FILEBASE/intermediate.crt"
	#SECURITY_OFFICER_KEY="$FILEBASE/security_officer.key"
	SECURITY_OFFICER_CSR="$FILEBASE/security_officer.csr"
	SECURITY_OFFICER_CRT="$FILEBASE/security_officer.crt"
        pin="654321"
	case "$LOWERCASE_ALGORITHM" in
		ecdsa)
			#openssl ecparam -out "$SECURITY_OFFICER_KEY" -check -name "$ALGORITHM_VARIANT" -genkey
			#ALGORITHM_VARIANT='P-256'
			#openssl genpkey -out "$SECURITY_OFFICER_KEY" -algorithm EC -pkeyopt "ec_paramgen_curve:$ALGORITHM_VARIANT" -pkeyopt ec_param_enc:named_curve -aes-256-cbc
			SECURITY_OFFICER_KEY="pkcs11:model=PKCS%2315%20emulated;manufacturer=piv_II;serial=00000000;token=YubiKey%20PIV%20Slot%209a;id=%04;object=CARD%20AUTH%20key;type=private";;
				#eddsa)
			#openssl genpkey -out "$SECURITY_OFFICER_KEY" -algorithm "$ALGORITHM_VARIANT" -aes-256-cbc
			#;;
		rsa)
		        SECURITY_OFFICER_KEY="pkcs11:model=PKCS%2315%20emulated;manufacturer=piv_II;serial=00000000;token=YubiKey%20PIV%20Slot%209a;id=%01;object=PIV%20AUTH%20key;type=private";;
			#openssl genrsa -out "$SECURITY_OFFICER_KEY" "$ALGORITHM_VARIANT"
			#openssl genpkey -out "$SECURITY_OFFICER_KEY" -algorithm RSA -pkeyopt "rsa_keygen_bits:$ALGORITHM_VARIANT" -aes-256-cbc
		*)
			>&2 echo "Error: unsupported '$LOWERCASE_ALGORITHM' algorithm"
	esac

	#openssl genpkey -out "$SECURITY_OFFICER_KEY" -algorithm "$ALGORITHM_VARIANT"
	
	openssl req -keyform engine -engine pkcs11 -out "$SECURITY_OFFICER_CSR" -new -key "$SECURITY_OFFICER_KEY" -passin "pass:$pin" -config <( cat <<- EOF
		[ default ]
		openssl_conf		= default.openssl_conf
		extensions		= default.extensions

		[ default.openssl_conf ]
		oid_section		= default.openssl_conf.oid_section

		[ default.openssl_conf.oid_section ]
		# If worth the risk of breaking some buggy implementations, we might use some
		# custom OIDs to build our own Certificate Transparency equivalent for example
		# but for now this section is overloading the useless TSA default to clear it.

		[ default.extensions ]

		###############################################################################

		[ req ]
		encrypt_key		= yes
		default_md		= sha256
		string_mask		= utf8only
		prompt			= no
		distinguished_name	= req.distinguished_name

		[ req.distinguished_name ]
		commonName		= Mesh Shield $UPPERCASE_ALGORITHM Intermediate CA
	EOF
	)

	SERIAL=$( echo `date '+%Y%m%d%H%M%S%N'``dd if=/dev/urandom bs=9 count=1 status=none | xxd -p` | cut -c -40 )
	openssl x509 -in "$SECURITY_OFFICER_CSR" -out "$SECURITY_OFFICER_CRT" -sha256 -days 365 -req -set_serial "0x$SERIAL" -CA "$INTERMEDIATE_CRT" -CAkey "$INTERMEDIATE_KEY" -extensions default.extensions -extfile <(
		cat <<- EOF
			[ default ]
			openssl_conf		= default.openssl_conf
			extensions		= default.extensions

			[ default.openssl_conf ]
			oid_section		= default.openssl_conf.oid_section

			[ default.openssl_conf.oid_section ]
			# If worth the risk of breaking some buggy implementations, we might use some
			# custom OIDs to build our own Certificate Transparency equivalent for example
			# but for now this section is overloading the useless TSA default to clear it.

			[ default.extensions ]
			basicConstraints	= CA:TRUE,pathlen:0
			keyUsage		= digitalSignature,cRLSign,keyCertSign
			subjectKeyIdentifier	= hash
			authorityKeyIdentifier	= keyid:always
			issuerAltName		= email:mspki@tii.ae
			nameConstraints		= critical,permitted;DNS:mesh.lan,permitted;DNS:.mesh.lan,permitted;email:mesh.lan,permitted;email:.mesh.lan
			#certificatePolicies	= @default.extensions.certificatePolicies

			#[ default.extensions.certificatePolicies ]
			#policyIdentifier	= 1.3.6.1.4.1.53429
			#CPS			= https://pki.ssrc.tii.ae/pki/mspki/

			[ default.extensions.policy ]
			commonName		= supplied
			emailAddress		= optional
		EOF
	) ##&& rm "$SECURITY_OFFICER_CSR"
}



security_officer_csr ECDSA secp256r1
security_officer_csr RSA 2048
echo 'All good!'
