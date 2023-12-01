#!/bin/bash


if [ -z "$1" ]; then
    echo "Please provide the CSR file as an argument."
    exit 1
fi

# Check if the "certificates" folder already exists
if [ ! -d "certificates" ]; then
    mkdir -p certificates
fi

cp "$1" certificates
cd certificates

# Check if ca.key exists and if it's still valid
if [ ! -f ca.key ] || ! openssl ec -in ca.key -noout &>/dev/null; then
    # Generate the Certificate Authority (CA) key and self-signed certificate
    openssl ecparam -name prime256v1 -genkey -noout -out ca.key
    openssl req -new -x509 -key ca.key -out ca.crt -days 365 -subj "/CN=TII"
fi

# Sign the server CSR with the CA to get the server certificate
openssl x509 -req -in "$1" -CA ca.crt -CAkey ca.key -CAcreateserial -out "$(basename "$1" .csr).crt" -days 365

# Verify that the certificate file has been created
if [ -f "$(basename "$1" .csr).crt" ]; then
    echo "Certificates have been generated successfully in the 'certificates' directory."
else
    echo "Failed to generate the certificate. Please check the input CSR and CA files."
fi

echo "Verifying certificates..."
openssl verify -CAfile ca.crt "$(basename "$1" .csr).crt"
