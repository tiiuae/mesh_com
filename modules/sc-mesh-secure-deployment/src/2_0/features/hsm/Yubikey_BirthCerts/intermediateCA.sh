#! /bin/bash -x

PKI='mspki'



set -e



create_intermediate()
{
	UPPERCASE_ALGORITHM="$1"
	ALGORITHM_VARIANT="$2"

	LOWERCASE_ALGORITHM=`echo "$UPPERCASE_ALGORITHM" | tr '[:upper:]' '[:lower:]'`
	FILEBASE="$PKI/${LOWERCASE_ALGORITHM}"
	ROOT_KEY="$FILEBASE/root.key"
	ROOT_CRT="$FILEBASE/root.crt"
	INTERMEDIATE_KEY="$FILEBASE/intermediate.key"
	INTERMEDIATE_CSR="$FILEBASE/intermediate.csr"
	INTERMEDIATE_CRT="$FILEBASE/intermediate.crt"

	case "$LOWERCASE_ALGORITHM" in
		ecdsa)
			#openssl ecparam -out "$INTERMEDIATE_KEY" -check -name "$ALGORITHM_VARIANT" -genkey
			ALGORITHM_VARIANT='P-256'
			openssl genpkey -out "$INTERMEDIATE_KEY" -algorithm EC -pkeyopt "ec_paramgen_curve:$ALGORITHM_VARIANT" -pkeyopt ec_param_enc:named_curve -aes-256-cbc
			;;
		#eddsa)
			#openssl genpkey -out "$INTERMEDIATE_KEY" -algorithm "$ALGORITHM_VARIANT" -aes-256-cbc
			#;;
		rsa)
			#openssl genrsa -out "$INTERMEDIATE_KEY" "$ALGORITHM_VARIANT"
			openssl genpkey -out "$INTERMEDIATE_KEY" -algorithm RSA -pkeyopt "rsa_keygen_bits:$ALGORITHM_VARIANT" -aes-256-cbc
			;;
		*)
			>&2 echo "Error: unsupported '$LOWERCASE_ALGORITHM' algorithm"
	esac

	#openssl genpkey -out "$INTERMEDIATE_KEY" -algorithm "$ALGORITHM_VARIANT"
	openssl req -out "$INTERMEDIATE_CSR" -new -key "$INTERMEDIATE_KEY" -config <( cat <<- EOF
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
}



sign_intermediate()
{
	ROOT_TYPE="$1"
	INTERMEDIATE_TYPE="$2"

	ROOT_KEY="$PKI/$ROOT_TYPE/root.key"
	ROOT_CRT="$PKI/$ROOT_TYPE/root.crt"
	INTERMEDIATE_CSR="$PKI/$INTERMEDIATE_TYPE/intermediate.csr"
	if [ "$ROOT_TYPE" = "$INTERMEDIATE_TYPE" ]; then
		INTERMEDIATE_CRT="$PKI/$INTERMEDIATE_TYPE/intermediate.crt"
	else
		INTERMEDIATE_CRT="$PKI/$INTERMEDIATE_TYPE/intermediate-$ROOT_TYPE.crt"
	fi

	SERIAL=$( echo `date '+%Y%m%d%H%M%S%N'``dd if=/dev/urandom bs=9 count=1 status=none | xxd -p` | cut -c -40 )
	openssl x509 -in "$INTERMEDIATE_CSR" -out "$INTERMEDIATE_CRT" -sha256 -days 365 -req -set_serial "0x$SERIAL" -CA "$ROOT_CRT" -CAkey "$ROOT_KEY" -extensions default.extensions -extfile <(
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
			basicConstraints	= CA:TRUE,pathlen:1
			keyUsage		= digitalSignature,cRLSign,keyCertSign
			subjectKeyIdentifier	= hash
			authorityKeyIdentifier	= keyid:always
			#issuerAltName		= email:mspki@tii.ae
			nameConstraints		= critical,permitted;DNS:.local,permitted;DNS:meshshield.corp,permitted;DNS:.meshshield.corp,permitted;email:meshshield.corp,permitted;email:.meshshield.corp;DNS:meshshield.lan,permitted;DNS:.meshshield.lan,permitted;email:meshshield.lan,permitted;email:.meshshield.lan
			#certificatePolicies	= @default.extensions.certificatePolicies

			#[ default.extensions.certificatePolicies ]
			#policyIdentifier	= 1.3.6.1.4.1.53429
			#CPS			= https://pki.ssrc.tii.ae/pki/mspki/

			[ default.extensions.policy ]
			commonName		= supplied
			emailAddress		= optional
		EOF
	)
#	) && rm "$INTERMEDIATE_CSR"
}



create_intermediate ECDSA secp256r1
#create_intermediate EdDSA ed25519
create_intermediate RSA 2048

for ROOT_TYPE in ecdsa rsa; do
	for INTERMEDIATE_TYPE in ecdsa rsa; do
		sign_intermediate "$ROOT_TYPE" "$INTERMEDIATE_TYPE"
	done
done

echo 'All good!'
