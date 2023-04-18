#!/bin/bash
###
### This file replaces the EC keys of the node with fake keys to simulate a rogue node

#add line at the beginning of the file
sed -i '1s/^/openssl_conf = openssl_init\n/' /etc/ssl/openssl.cnf

# getting the system aarch64 or x86_64
system=$(uname -m)

#rpbi
if [[ "$system" == "aarch64" ]]; then
printf "\n[openssl_init]\nengines = engine_section\n\n[engine_section]\npkcs11 = pkcs11_section\n\n[pkcs11_section]\nengine_id = pkcs11\ndynamic_path = /usr/lib/engines-1.1/pkcs11.so\nMODULE_PATH = /usr/lib/softhsm/libsofthsm2.so" >> /etc/ssl/openssl.cnf
fi

#intel
if [[ "$system" == "x86_64" ]]; then
printf "\n[openssl_init]\nengines = engine_section\n\n[engine_section]\npkcs11 = pkcs11_section\n\n[pkcs11_section]\nengine_id = pkcs11\ndynamic_path = /usr/lib/x86_64-linux-gnu/engines-1.1/pkcs11.so #libP11/libpkcs11.so\nMODULE_PATH = /usr/lib/x86_64-linux-gnu/softhsm/libsofthsm2.so" >> /etc/ssl/openssl.cnf
fi

#loading softhsm library
LIB='/usr/lib/softhsm/libsofthsm2.so'

pin=$1
#delete keys
keys=$(pkcs11-tool --module="$LIB" -O --login --pin "$pin")
if [ ${#keys} -ne 0  ]
then
  echo "Keys Found"
  echo "Deleting old keys"
  pkcs11-tool --module="$LIB" --login --pin "$pin" --delete-object --type privkey --id 01
  pkcs11-tool --module="$LIB" --login --pin "$pin" --delete-object --type pubkey --id 01
fi

mesh_if=wlp1s0
mesh_if_mac="$(ip -brief link | grep "$mesh_if" | awk '{print $3; exit}')"
uid=$(echo -n "$mesh_if_mac" | b2sum -l 32)
uid=${uid::-1};
LABEL="$uid"

#generate fake keys
echo "Generating new keys"
pkcs11-tool --keypairgen --key-type="EC:prime256v1"  --login --pin="$pin" --module="$LIB" --label="$LABEL" --id=01 #for EC
#export to der
pkcs11-tool --read-object --id 01 --type pubkey --module="$LIB" --output-file /etc/ssl/certs/mesh_cert.der

# Replacing root cert with fake mesh cert
#cp /etc/ssl/certs/mesh_cert.der /etc/ssl/certs/root_cert.der