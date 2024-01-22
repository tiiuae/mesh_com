#!/bin/bash
        KEYPAIR_TYPE="rsa"
	LIBSOFTHSM2='/usr/lib/softhsm/libsofthsm2.so'
        case "$KEYPAIR_TYPE" in
                ecdsa)  HSM_KEYPAIR_TYPE='EC:prime256v1';;
                eddsa)  HSM_KEYPAIR_TYPE='EC:edwards25519';;
                rsa)    HSM_KEYPAIR_TYPE='RSA:3072';;
                *)      >&2 echo "Unsupported keypair type '$KEYPAIR_TYPE'"; exit 255;;
        esac
	#Should be changed to env variables
        SOFTHSM2_PIN="123456"
	SOFTHSM2_SO_PIN="12345678"
        HSM_LABEL='CBMA'
        TOKENS_DIR="/tmp/cbma/$KEYPAIR_TYPE/softhsm2/"
	rootCACert='root.crt'
	rootCAKey='root.key'
	HSM_CERT='hsm.pem'
	Client_CSR='client.csr'
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
        #Deleting token for testing purposes 
	softhsm2-util --delete-token --token "$KEYPAIR_TYPE"
        #Creating softhsm token
	softhsm2-util --init-token --free --label "$KEYPAIR_TYPE" --pin "$SOFTHSM2_PIN" --so-pin "$SOFTHSM2_SO_PIN"
        #Generating Keys inside HSM
	pkcs11-tool --modul "$LIBSOFTHSM2" --label "$KEYPAIR_TYPE" --pin "$SOFTHSM2_PIN" --keypairgen --key-type "$HSM_KEYPAIR_TYPE" --id 01
        #Creating Client CSR using private key inside softhsm
        openssl req -new -engine pkcs11 -keyform engine -key "pkcs11:model=SoftHSM%20v2;manufacturer=SoftHSM%20project;token=$KEYPAIR_TYPE;object=$KEYPAIR_TYPE;type=private" -passin "pass:$SOFTHSM2_PIN" -out $Client_CSR
        #Using csr to generate a certificate signed by root CA
        sudo openssl x509 -in $Client_CSR -out $HSM_CERT -sha256 -days 90 -req -set_serial 1 -CA $rootCACert -CAkey $rootCAKey
        #Importing the client certificate into hsm 
	pkcs11-tool --module $LIBSOFTHSM2 --login --pin $SOFTHSM2_PIN --write-object $HSM_CERT --type cert --label "HSM Cert"
        #Executing cllient https requests script
        python3 mdmagentrequestsmodule.py    
