#!/bin/bash
###
### This file initialize the softhsm. It creates a random pin and store it in a file (store it in AES with the id)
### the ID of the device should be passed as parameter, otherwise uses $uid as ID.
### to generate the ID based on mac (lines 164-176 of entrypoint):
###    mesh_if_mac="$(ip -brief link | grep "$mesh_if" | awk '{print $3; exit}')"
###   uid=$(echo -n $mesh_if_mac | b2sum -l 32)
###   uid=${uid::-1}; echo $uid

#loading softhsm library
LIB='/usr/lib/softhsm/libsofthsm2.so'

# Set output path
output_path="/opt"

# list the available slots and check if the token label exists
token_label="secccoms"

### Check if a directory does not exist ###
if [ ! -d "$output_path" ]
then
  mkdir -p "$output_path"
fi

initialize_hsm()
{
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
}

# Parse command line options
while getopts "l:" opt; do
  case ${opt} in
    l )
      label=$OPTARG
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
    : )
      echo "Option -$OPTARG requires an argument" >&2
      exit 1
      ;;
  esac
done
shift $((OPTIND -1))



# this is an initial idea of label (passcode) that we can pass to the  CS. Currently is getting input or hash of the mac
if [ -z "$label" ] # label or ID
   then
    mesh_if=wlp1s0
    mesh_if_mac="$(ip -brief link | grep "$mesh_if" | awk '{print $3; exit}')"
    uid=$(echo -n "$mesh_if_mac" | b2sum -l 32)
    uid=${uid::-1};
    LABEL="$uid"
   else
    LABEL="$label"
fi

if [[ -f "$output_path/output.txt" ]]; then #pin exists load it
    #to decrypt
    pin=$(openssl aes-256-cbc -md sha256 -salt -a -pbkdf2 -iter 100000  -d  -k $LABEL -in "$output_path"/output.txt)
else
    #random pin
    pin=$(tr -dc '0-9' </dev/random | head -c 6)
fi

initialize_all()
{
  #intialize token
  pkcs11-tool --module="$LIB" --init-token --label secccoms --so-pin "$pin" # --pin "$pin"
  #init the pin
  pkcs11-tool --init-pin --login --pin "$pin" --so-pin "$pin" --module "$LIB"
}

hard_delete()
{
softhsm2_output=$(softhsm2-util --show-slot)
if [ ${#softhsm2_output} -ne 616 ]
 then
  echo "Token exists"
#  token_label=$(echo "$softhsm2_output" | grep 'Label:' | sed 's/^.*: //')
  serial=$(softhsm2-util --show-slot |grep Serial |awk '{print $3}')
  softhsm2-util --slot 1 --delete-token --serial "$serial"
#  softhsm2-util --slot 1 --delete-token --token "$token_label"
else
  echo "No Token exists"
fi
}

soft_delete()
{
if pkcs11-tool --module="$LIB" --list-slots | grep -q "Label: $token_label"; then
  echo "Token exists"
  # delete the existing token
  pkcs11-tool --module="$LIB" --login --pin "$pin" --delete-token --label "$token_label" #we need the pin from previous execution
  #delete keys
  keys=$(pkcs11-tool --module="$LIB" -O --login --pin "$pin")
  if [ ${#keys} -ne 0  ]
  then
    echo "Keys Found"
    echo "Deleting old keys"
    pkcs11-tool --module="$LIB" --login --pin "$pin" --delete-object --type privkey --id 01
    pkcs11-tool --module="$LIB" --login --pin "$pin" --delete-object --type pubkey --id 01
  fi
else
  echo "No Token exists"
fi

}

key_generation()
{

#generate keys
echo "Generating new keys"
#pkcs11-tool --keypairgen --key-type="RSA:4096"  --login --pin=$pin --module=$LIB --label=$LABEL --id=01
pkcs11-tool --keypairgen --key-type="EC:prime256v1"  --login --pin="$pin" --module="$LIB" --label="$LABEL" --id=01 #for EC
#export to der
pkcs11-tool --read-object --id 01 --type pubkey --module="$LIB" --output-file /etc/ssl/certs/mesh_cert.der
}

export_pin()
{ #needs to be improved $LABEL is known
### Check if a directory does not exist ###
if [ ! -d "$output_path" ]
then
  mkdir -p "$output_path"
fi

if test -f "$output_path"/output.txt; then
    rm "$output_path"/output.txt
fi
echo "$pin" | openssl enc -aes-256-cbc -md sha256 -a -pbkdf2 -iter 100000 -salt -pass pass:$LABEL > "$output_path"/output.txt
}

create_csr(){

#create the CSR
#key_label=$(pkcs11-tool --module $LIB --list-objects --login --pin $pin | awk '/Private Key/{getline; print $2}' | cut -d "\"" -f 2)

SUBJ="/C=AE/ST=Abu Dhabi/L=Abu Dhabi/O=TII/OU=SSRC/CN=*.tii.ae"


export OPENSSL_PIN="$pin"
openssl req -new -x509 -days 365 -subj "$SUBJ" -sha256 -engine pkcs11 -keyform engine -key 01 -passin env:OPENSSL_PIN  -out cert.pem
echo "Certificate Signing Request (CSR)"
openssl req -new -engine pkcs11 -keyform engine -key 01 -passin env:OPENSSL_PIN  -out /opt/mycsr.csr -subj "$SUBJ"
openssl x509 -req -CAkeyform engine -engine pkcs11 -in /opt/mycsr.csr -CA cert.pem -CAkey 01 -set_serial 1 -sha256  -passin env:OPENSSL_PIN  -out /opt/mycert.pem # self-signed cerificate
echo "self-signed certificate"
openssl x509 -in /opt/mycert.pem -noout -text -passin env:OPENSSL_PIN


}


initialize_hsm
initialize_all
#hard_delete
soft_delete
key_generation
export_pin
create_csr

