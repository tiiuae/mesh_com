#!/bin/bash

# Function to derive IPv6 address from MAC address
function mac_to_ipv6() {
    # Remove any separators from the MAC address (e.g., colons, hyphens)
    mac_address="${1//[:\-]/}"
    mac_address="${mac_address,,}"

    # Split the MAC address into two equal halves
    first_half="${mac_address:0:6}"

    # Convert the first octet from hexadecimal to binary
    binary_first_octet=$(echo "obase=2; ibase=16; ${first_half:0:2}" | bc | xargs printf "%08d")

    # Invert the seventh bit (change 0 to 1 or 1 to 0)
    inverted_seventh_bit=$(( 1 - $(echo "${binary_first_octet:6:1}") ))

    # Convert the modified binary back to hexadecimal
    modified_first_octet=$(echo "obase=16; ibase=2; ${binary_first_octet:0:6}${inverted_seventh_bit}${binary_first_octet:7}" | bc)

    # Replace the original first octet with the modified one
    modified_mac_address="${modified_first_octet}${mac_address:2}"

    line="${modified_mac_address:0:6}fffe${modified_mac_address:6}"

    # Add "ff:fe:" to the middle of the new MAC address
    mac_with_fffe=$(echo "$line" | sed -r 's/(.{4})/\1:/g; s/:$//')

    echo "fe80::$mac_with_fffe"
}

# Read the user input for the network interface, defaulting to "wlp1s0" if no input is provided
#read -p "Enter the network interface (default: wlp1s0): " network_interface
#network_interface=${network_interface:-wlp1s0}

# Get the network interface from the command-line argument or use a default value
network_interface=${1:-wlp1s0}


# Parse the MAC address from the network interface using ip command
mac_address=$(ip link show $network_interface | awk '/ether/ {print $2}')

id=${mac_address//:/} #mac address with no colon

# Derive the IPv6 address from the MAC address (extended format)
ipv6_address=$(mac_to_ipv6 "$mac_address")

# Generate the EC private key
openssl ecparam -name prime256v1 -genkey -noout -out macsec_"$id".key  # for other certificates (eg ipsec) we need to verify if this exists

# Generate the 256-bit random number from the fingerprint of the public key
random=$(openssl ec -in macsec_"$id".key -pubout -outform DER | sha256sum | awk '{print substr($1, 1, 30)}')


# Derive the second IPv6 address (mesh IPv6) from the ID (extended format)
mesh_ipv6="fe80::$(echo "$random" | cut -c1-4):$(echo "$random" | cut -c5-8):$(echo "$random" | cut -c9-12)"


# Create a CSR configuration file with the custom SANs
cat > csr.conf <<EOF
[req]
default_bits = 256
prompt = no
encrypt_key = no
distinguished_name = dn
req_extensions = v3_req

[dn]
CN = $mac_address

[v3_req]
subjectAltName = @alt_names

[alt_names]
DNS.1 = csl$id.local
DNS.2 = cls$id.lan
IP.1 = $ipv6_address
IP.2 = $mesh_ipv6
EOF



# Create the CSR using the generated private key and the custom CSR configuration
openssl req -new -key macsec_"$id".key -out macsec_"$id".csr -config csr.conf
echo "CSR generated: " "macsec_$id.csr"
openssl req -in macsec_"$id".csr -text -noout