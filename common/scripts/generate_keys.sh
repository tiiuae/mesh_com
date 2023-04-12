#!/bin/bash
###
### This file initialize the softhsm. It creates a random pin and store it in a file (store it in AES with the id)
### the ID of the device should be passed as parameter, otherwise uses 'myKey' as ID.
### to generate the ID based on mac (lines 164-176 of entrypoint):
###    mesh_if_mac="$(ip -brief link | grep "$mesh_if" | awk '{print $3; exit}')"
###   uid=$(echo -n $mesh_if_mac | b2sum -l 32)
###   uid=${uid::-1}; echo $uid

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


softhsm2_output=$(softhsm2-util --show-slot)
if [ ${#softhsm2_output} -ne 616 ]
 then
  echo "Token exists"
  token_label=$(echo "$softhsm2_output" | grep 'Label:' | sed 's/^.*: //')
  softhsm2-util --slot 1 --delete-token --token $token_label
else
  echo "No Token exists"
fi

#random pin 
pin=$((1 + RANDOM % 99999999999))

#delete keys
keys=$(pkcs11-tool --module="$LIB" -O --login --pin "$pin")
if [ ${#keys} -ne 0  ]
then
  echo "Keys Found"
  echo "Deleting old keys"
  pkcs11-tool --module="$LIB" --login --pin "$pin" --delete-object --type privkey --id 01
  pkcs11-tool --module="$LIB" --login --pin "$pin" --delete-object --type pubkey --id 01
fi

#intialize token
softhsm2-util --init-token --slot 0 --label secccoms --pin "$pin" --so-pin "$pin" ## this should be done before?


if [ -z "$1" ] # label or ID
   then
    mesh_if=wlp1s0
    mesh_if_mac="$(ip -brief link | grep "$mesh_if" | awk '{print $3; exit}')"
    uid=$(echo -n "$mesh_if_mac" | b2sum -l 32)
    uid=${uid::-1};
    LABEL="$uid"
   else
     LABEL="$1"
fi

#generate keys
echo "Generating new keys"
#pkcs11-tool --keypairgen --key-type="RSA:4096"  --login --pin=$pin --module=$LIB --label=$LABEL --id=01
pkcs11-tool --keypairgen --key-type="EC:prime256v1"  --login --pin="$pin" --module="$LIB" --label="$LABEL" --id=01 #for EC
#export to der
pkcs11-tool --read-object --id 01 --type pubkey --module="$LIB" --output-file /etc/ssl/certs/mesh_cert.der

output_path="/opt"

### Check if a directory does not exist ###
if [ ! -d "$output_path" ]
then
  mkdir -p "$output_path"
fi

echo "$pin" | openssl enc -aes-256-cbc -md sha256 -a -pbkdf2 -iter 100000 -salt -pass pass:$LABEL > "$output_path"/output.txt



#openssl aes-256-cbc -md sha256 -salt -a -pbkdf2 -iter 100000  -d  -k "$LABEL" -in "$output_path"/output.txt



