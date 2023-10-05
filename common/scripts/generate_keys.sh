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
slot_id=""
free_slot_id=""

### Check if a directory does not exist ###
if [ ! -d "$output_path" ]
then
  mkdir -p "$output_path"
fi

set_openssl_env()
{
  export OPENSSL_CONF=/opt/comms_openssl.cnf
  echo "OPENSSL_CONF=$OPENSSL_CONF"
}

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

create_and_init_token()
{
  #intialize token
  echo "pin: $pin"
  pkcs11-tool --slot "$free_slot_id" --module="$LIB" --init-token --label secccoms --so-pin "$pin" # --pin "$pin"
}

get_slot_id_by_token_label()
{
slots_info=$(pkcs11-tool --module "$LIB" --list-slots)

# Loop through slot information
while IFS= read -r line; do
    if [[ $line == "Slot "* ]]; then
        # Extract the slot ID from the line
        hex_id=$(echo "$line" | grep -o -P '\(0x[0-9a-fA-F]+\)' | sed 's/[\(\)]//g')
    elif [[ $line == "  token label"* ]]; then
        # Extract the token label from the line
        label=$(echo "$line" | awk -F ':' '{print $2}' | awk '{$1=$1;print}')

        # Check if the label matches the desired token label
        if [[ "$label" == "$token_label" ]]; then
            slot_id="$hex_id"
            break  # Exit the loop if the label is found
        fi
    fi
done <<< "$slots_info"
echo "slot_id: $slot_id"
}

get_next_free_slot_id()
{
slots_info=$(pkcs11-tool --module "$LIB" --list-slots)

# Loop through slot information
while IFS= read -r line; do
    if [[ $line == "Slot "* ]]; then
        # Extract the slot ID from the line
        hex_id=$(echo "$line" | grep -o -P '\(0x[0-9a-fA-F]+\)' | sed 's/[\(\)]//g')
    elif [[ $line == "  token state"* ]]; then
        # Extract the token label from the line
        state=$(echo "$line" | awk -F ':' '{print $2}' | awk '{$1=$1;print}')

        # Check if the label matches the desired token label
        if [[ "$state" == "uninitialized" ]]; then
            free_slot_id="$hex_id"
            break  # Exit the loop if the label is found
        fi
    fi
done <<< "$slots_info"
echo "free_slot_id: $free_slot_id"
}

set_user_pin()
{
  echo "Set user pin for slot $slot_id"
  pkcs11-tool --slot "$slot_id" --init-pin --login --pin "$pin" --so-pin "$pin" --module "$LIB"
}

hard_delete()
{
softhsm2_output=$(softhsm2-util --show-slot)
if [ ${#softhsm2_output} -ne 616 ]
 then
  echo "Token exists"
#  token_label=$(echo "$softhsm2_output" | grep 'Label:' | sed 's/^.*: //')
  serial=$(softhsm2-util --show-slot |grep Serial |awk '{print $3}')
  softhsm2-util --slot "$slot_id" --delete-token --serial "$serial"
#  softhsm2-util --slot 1 --delete-token --token "$token_label"
else
  echo "No Token exists"
fi
}

soft_delete()
{
if pkcs11-tool --slot "$slot_id" --module="$LIB" --list-slots | grep -q "$token_label"; then
  echo "Token exists"
  # delete the existing token
  pkcs11-tool --slot "$slot_id" --module="$LIB" --login --pin "$pin" --delete-token --label "$token_label" #we need the pin from previous execution
  #delete keys
  keys=$(pkcs11-tool --module="$LIB" -O --login --pin "$pin")
  if [ ${#keys} -ne 0  ]
  then
    echo "Keys Found"
    echo "Deleting old keys"
    pkcs11-tool --slot "$slot_id" --module="$LIB" --login --pin "$pin" --delete-object --type privkey --id 01
    pkcs11-tool --slot "$slot_id" --module="$LIB" --login --pin "$pin" --delete-object --type pubkey --id 01
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
pkcs11-tool --slot "$slot_id" --keypairgen --key-type="EC:prime256v1"  --login --pin="$pin" --module="$LIB" --label="$LABEL" --id=01 #for EC
#export to der
pkcs11-tool --slot "$slot_id" --read-object --id 01 --type pubkey --module="$LIB" --output-file /etc/ssl/certs/mesh_cert.der
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
# Create "ca" certificate
openssl req -new -x509 -days 365 -subj "$SUBJ" -sha256 -engine pkcs11 -keyform engine -key "pkcs11:token=$token_label;object=$LABEL" -passin env:OPENSSL_PIN  -out cert.pem

# Create CSR
echo "Certificate Signing Request (CSR)"
openssl req -new -engine pkcs11 -keyform engine -key "pkcs11:token=$token_label;object=$LABEL" -passin env:OPENSSL_PIN  -out /opt/mycsr.csr -subj "$SUBJ"
# Sign the CSR with ca certificate.
openssl x509 -req -CAkeyform engine -engine pkcs11 -in /opt/mycsr.csr -CA cert.pem -CAkey "pkcs11:token=$token_label;object=$LABEL" -set_serial 1 -sha256  -passin env:OPENSSL_PIN  -out /opt/mycert.pem # self-signed cerificate

echo "Self-signed certificate properties:"
openssl x509 -in /opt/mycert.pem -noout -text -passin env:OPENSSL_PIN
}


set_openssl_env
get_slot_id_by_token_label
if [[ -z "$slot_id" ]]; then
  get_next_free_slot_id
  # Create new token in case one didn't exist already
  create_and_init_token
  # Get slot id
  get_slot_id_by_token_label
  set_user_pin
  key_generation
fi
if [[ -n "$slot_id" ]]; then
  #hard_delete
  #soft_delete
  export_pin
  create_csr
fi

