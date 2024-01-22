#! /bin/bash -x

set -e



create_root()
{
	UPPERCASE_ALGORITHM="$1"
	ALGORITHM_VARIANT="$2"
	#INTERMEDIATE_CAS_NUMBER=
	LOWERCASE_ALGORITHM=`echo "$UPPERCASE_ALGORITHM" | tr '[:upper:]' '[:lower:]'`
	FILEBASE="mspki/${LOWERCASE_ALGORITHM}/root"
	KEY_FILE="$FILEBASE.key"
	CRT_FILE="$FILEBASE.crt"

	case "$LOWERCASE_ALGORITHM" in
		ecdsa)
			openssl ecparam -out "$KEY_FILE" -check -name "$ALGORITHM_VARIANT" -genkey
			;;
		eddsa)
			openssl genpkey -out "$KEY_FILE" -algorithm "$ALGORITHM_VARIANT"
			;;
		rsa)
			openssl genrsa -out "$KEY_FILE" "$ALGORITHM_VARIANT"
			;;
		*)
			>&2 echo "Error: unsupported '$LOWERCASE_ALGORITHM' algorithm"
	esac

	openssl req -out "$CRT_FILE" -new -key "$KEY_FILE" -x509 -days 365 -config <( cat <<- EOF
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
		encrypt_key		= no
		default_md		= sha256
		string_mask		= utf8only
		x509_extensions		= req.x509_extensions
		prompt			= no
		distinguished_name	= req.distinguished_name

		[ req.distinguished_name ]
		commonName		= Mesh Shield $UPPERCASE_ALGORITHM Root CA

		[ req.x509_extensions ]
		basicConstraints	= CA:TRUE,pathlen:0
		subjectKeyIdentifier	= hash
		authorityKeyIdentifier  = keyid:always
		#issuerAltName		= email:mspki@tii.ae
		#certificatePolicies	= @req.x509_extensions.certificatePolicies
		#subjectInfoAccess	= OCSP;URI:http://root.ecdsa.mspki.ocsp.mesh.lan/

		#[ req.x509_extensions.certificatePolicies ]
		#policyIdentifier	= 1.3.6.1.4.1.53429
		#CPS			= https://pki.ssrc.tii.ae/pki/mspki/
	EOF
	)
}



create_root ECDSA secp256r1
create_root EdDSA ed25519
create_root RSA 3072
echo 'All good!'
