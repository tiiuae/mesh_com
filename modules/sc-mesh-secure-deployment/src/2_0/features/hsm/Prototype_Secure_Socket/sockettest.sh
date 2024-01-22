#!/bin/bash
	KEYPAIR_TYPE="rsa"
	LIBSOFTHSM2='/usr/lib/softhsm/libsofthsm2.so'
        case "$KEYPAIR_TYPE" in
                ecdsa)  HSM_KEYPAIR_TYPE='EC:prime256v1';;
                eddsa)  HSM_KEYPAIR_TYPE='EC:edwards25519';;
                rsa)    HSM_KEYPAIR_TYPE='RSA:3072';;
                *)      >&2 echo "Unsupported keypair type '$KEYPAIR_TYPE'"; exit 255;;
        esac
	SOFTHSM2_PIN="123456"
	SOFTHSM2_SO_PIN="12345678"
        HSM_LABEL='CBMA'
	HSM_CERT='hsmcert.pem'
        TOKENS_DIR="/tmp/cbma/$KEYPAIR_TYPE/softhsm2/"
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
        softhsm2-util --delete-token --token "$KEYPAIR_TYPE"
	#Creating SOFTHSM token and generating private key
        softhsm2-util --init-token --free --label "$KEYPAIR_TYPE" --pin "$SOFTHSM2_PIN" --so-pin "$SOFTHSM2_SO_PIN"
        pkcs11-tool --modul "$LIBSOFTHSM2" --label "$KEYPAIR_TYPE" --pin "$SOFTHSM2_PIN" --keypairgen --key-type "$HSM_KEYPAIR_TYPE" --id 01
        #Getting PKCS11URI:GNUTLS_PIN="$SOFTHSM2_PIN" p11tool --login --list-all "pkcs11:token=$KEYPAIR_TYPE"
        openssl req -new -x509 -days 7300 -sha512 -extensions v3_ca  -engine pkcs11 -keyform engine -key "pkcs11:model=SoftHSM%20v2;manufacturer=SoftHSM%20project;token=$KEYPAIR_TYPE;object=$KEYPAIR_TYPE;type=private" -passin "pass:$SOFTHSM2_PIN" -out $HSM_CERT
        #Importing HSM_Cert inside hsm
	pkcs11-tool --module $LIBSOFTHSM2 --login --pin $SOFTHSM2_PIN --write-object $HSM_CERT --type cert --label "HSM Cert"
   	TOKEN_LABEL=$KEYPAIR_TYPE
   	#Running SecureSocket script using private key & cert stored in hsm
	python securesocket.py --cert-path pkcs11:token=$TOKEN_LABEL --key-path pkcs11:token=$TOKEN_LABEL --pin $SOFTHSM2_PIN --cert-verify none
    	# In another terminal check that the server returns the certificate
        #openssl s_client -connect 127.0.0.1:4433 2>/dev/null | openssl x509 -outform PEM | tee /dev/fd/2 | openssl x509 -text -noout 
        softhsm2-util --delete-token --token "$KEYPAIR_TYPE"
        #rm -rf $TOKENS_DIR
   	#rm -rf *.pem
