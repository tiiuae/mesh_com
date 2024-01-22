#! /bin/sh -x

BUNDLE='mdm_download_to_agent_certs.tar.bz2'

for ALGORITHM in ecdsa eddsa rsa; do
	openssl pkeyutl -verify -sigfile "$BUNDLE.$ALGORITHM-sig" -certin -inkey "mspki/$ALGORITHM/root.crt" -in "$BUNDLE" -rawin
done
