#! /bin/sh -x
BUNDLE='yk_at_birth.tar.bz2'
PIN="654321"
#for ALGORITHM in 'ecdsa' 'rsa'; do
 #       case ALGORITHM in
  #           ecdsa)          SO_KEY="pkcs11:object=Private key for Card Authentication";;
                 #pkcs11:object=Private key for PIV Authentication";;
   #          rsa)            SO_KEY="pkcs11:object=Private key for PIV Authentication";;

    #         *)      >&2 echo "Unsupported keypair type '$KEYPAIR_TYPE'"; exit 255;;
     #   esac
#openssl pkeyutl -verify -sigfile "$BUNDLE.$ALGORITHM-sig" -certin -inkey "mspki/$ALGORITHM/security_officers/filebased.crt" -in "$BUNDLE" -rawin
	#PKCS11_MODULE_PATH="/usr/local/lib/libykcs11.so" openssl pkeyutl -engine pkcs11 -keyform engine -verify -in  "$BUNDLE"  -sigfile "$BUNDLE.$ALGORITHM-sig" -inkey "$SO_KEY" -passin pass:"654321"
#done
ecdsa_SO_KEY="pkcs11:object=Private key for Card Authentication"
rsa_SO_KEY="pkcs11:object=Private key for PIV Authentication"
#for ALGORITHM in ecdsa rsa; do
 	     #SO_KEY="${ALGORITHM}_SO_KEY"
ALGORITHM='ecdsa'
#	          openssl pkeyutl -verify -sigfile "$BUNDLE.$ALGORITHM-sig" -certin -inke>   echo "$ecdsa_SO_KEY"
PKCS11_MODULE_PATH="/usr/local/lib/libykcs11.so" openssl pkeyutl -engine pkcs11 -keyform engine -verify -in  "$BUNDLE" -rawin -sigfile "$BUNDLE.$ALGORITHM-sig" -inkey "$ecdsa_SO_KEY" -passin "pass:$PIN" 
ALGORITHM='rsa'
PKCS11_MODULE_PATH="/usr/local/lib/libykcs11.so" openssl pkeyutl -engine pkcs11 -keyform engine -verify -in  "$BUNDLE" -rawin -sigfile "$BUNDLE.$ALGORITHM-sig" -inkey "$rsa_SO_KEY" -passin "pass:$PIN"
#done
