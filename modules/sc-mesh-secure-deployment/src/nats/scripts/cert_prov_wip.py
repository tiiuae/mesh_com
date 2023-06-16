import base64
import os
import random
import string
import subprocess

import PyKCS11.LowLevel
import requests
import hashlib

from PyKCS11 import *

from cryptography import x509
from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric import rsa

from cryptography.x509.oid import NameOID
from cryptography.x509 import load_pem_x509_certificate
from cryptography.x509 import load_der_x509_certificate

from datetime import datetime
from cryptography.hazmat.primitives import hashes
# To export pin
from Crypto.Protocol.KDF import PBKDF2
from Crypto.Hash import SHA256
from Crypto.Util.Padding import unpad
from Crypto.Cipher import AES

# Initialize PKCS11 interface
os.environ['PKCS11_MODULE'] = LIB = "/usr/local/lib/softhsm/libsofthsm2.so"
pkcs11 = PyKCS11.PyKCS11Lib()
pkcs11.load(os.environ['PKCS11_MODULE'])

TOKEN_LABEL = "secccoms"
PRIVATE_KEY_LABEL = "PrivateProvisioningKey"
PUBLIC_KEY_LABEL = "PublicProvisioningKey"
CERTIFICATE_LABEL = "ProvisioningCertificate"
KEY_LENGTH = 2048

KEY_PAIR_ID = (0x99,)
KEY_ID = 99

# File to store label, encrypted pin etc
# OUTPUT_PATH = "/opt"
OUTPUT_PATH = "/home/saku/workspaces/output"
USER_PIN_FILE = "user_pin"
SO_PIN_FILE = "so_pin"
DEVICE_ID_FILE = "/opt/identity"
CSR_FILE = "/home/saku/workspaces/output/mycsr.csr"


def generate_pin():
    pin_length = 6
    # Generate a random 6-digit PIN
    pin = ''.join(random.choices(string.digits, k=pin_length))
    return pin


def export_pin(pin, file_name, label):
    # Check if the directory does not exist and create it if needed
    if not os.path.exists(OUTPUT_PATH):
        os.makedirs(OUTPUT_PATH)

    # Remove the output file if it already exists
    output_file = os.path.join(OUTPUT_PATH, file_name)
    if os.path.isfile(output_file):
        os.remove(output_file)

    # Execute the openssl command to encrypt and export the PIN
    openssl_command = f'echo "{pin}" | openssl enc -aes-256-cbc -md sha256 -a -pbkdf2 -iter 100000 -salt -pass pass:{label} > {output_file}'
    print(openssl_command)
    ret = subprocess.run(openssl_command, shell=True)
    if ret.returncode != 0:
        print(str(ret.returncode) + str(ret.stdout) + str(ret.stderr))
        return False
    return True


def recover_pin(filename):
    try:
        with open(os.path.join(OUTPUT_PATH, filename)) as file:
            pin_aux = file.readlines()  # need to make it absolute
        # Determine salt and ciphertext
        encryptedDataB64 = pin_aux[0].split('\n')[0]
        encryptedData = base64.b64decode(encryptedDataB64)
        salt = encryptedData[8:16]
        ciphertext = encryptedData[16:]
        # Reconstruct Key/IV-pair
        pbkdf2Hash = PBKDF2(TOKEN_LABEL, salt, 32 + 16, count=100000,
                            hmac_hash_module=SHA256)
        key = pbkdf2Hash[:32]
        iv = pbkdf2Hash[32:32 + 16]
        # Decrypt with AES-256 / CBC / PKCS7 Padding
        cipher = AES.new(key, AES.MODE_CBC, iv)
        return unpad(cipher.decrypt(ciphertext), 16).decode().split('\n')[0]
    except FileNotFoundError:
        print("No pin found")


def get_device_id():
    try:
        with open(DEVICE_ID_FILE) as file:
            device_id = file.readlines()
            return device_id
    except FileNotFoundError:
        print("Device id not found")


print(pkcs11.getSlotList(tokenPresent=False))

token_serial = ""


def get_pkcs11_session():
    slots = pkcs11.getSlotList(tokenPresent=True)
    for slot in slots:
        try:
            session = pkcs11.openSession(slot,
                                         CKF_SERIAL_SESSION | CKF_RW_SESSION)
            print("SlotID:", session.getSessionInfo().slotID)
            # Login as security officer
            session.login(pin=recover_pin(SO_PIN_FILE),
                          user_type=PyKCS11.LowLevel.CKU_SO)

            # Fixme: using same PIN for user and security officer
            # Set user pin
            session.initPin(recover_pin(SO_PIN_FILE))
            # Log out SO and login as normal user
            session.logout()
            session.login(recover_pin(SO_PIN_FILE),
                          user_type=PyKCS11.LowLevel.CKU_USER)

            token_info = pkcs11.getTokenInfo(slot)
            # Debug
            # print("Token info:", token_info)
            return session
        except PyKCS11Error as e:
            print("exception:", e)
            pass
    return False


def create_token(token_name):
    # Generate security officer pin for the token
    so_pin = generate_pin()
    slots = pkcs11.getSlotList(tokenPresent=False)
    for slot in slots:
        try:
            pkcs11.initToken(slot, so_pin, token_name)
            # Encrypt and export pin for later usage
            ret = export_pin(so_pin, SO_PIN_FILE, TOKEN_LABEL)
            break
        except PyKCS11Error as e:
            print("Create token exception:", e)
            pass


# Testing, get session with token
session = get_pkcs11_session()

# Create token if one was not found
if not session:
    create_token(TOKEN_LABEL)
    session = get_pkcs11_session()
    print("Session status after create token:", session)


def get_certificate():
    # Get certificate objects by label and id
    certificate_objects = session.findObjects([(PyKCS11.LowLevel.CKA_CLASS,
                                                PyKCS11.LowLevel.CKO_CERTIFICATE),
                                               (PyKCS11.LowLevel.CKA_LABEL,
                                                CERTIFICATE_LABEL),
                                               (PyKCS11.LowLevel.CKA_ID,
                                                KEY_PAIR_ID)
                                               ])
    if len(certificate_objects) > 0:
        print(certificate_objects[0])
        for certificate_object in certificate_objects:
            certificate_bytes = bytes(session.getAttributeValue(certificate_object,
                                                                [PyKCS11.LowLevel.CKA_VALUE])[0])

            # Verify the validity of the certificate
            certificate = load_der_x509_certificate(certificate_bytes)
            current_time = datetime.now()
            if certificate.not_valid_before <= current_time <= certificate.not_valid_after:
                """
                print("Certificate is valid from",
                      certificate.not_valid_before, "until",
                      certificate.not_valid_after)
                """
                # Return certiticate asPEM format
                return certificate.public_bytes(
                    encoding=serialization.Encoding.PEM).decode()
            else:
                print("Certificate is not valid.")
    return False


def get_private_key_object():
    # Get private key objects by label and id
    priv_key_objects = session.findObjects([(PyKCS11.LowLevel.CKA_CLASS,
                                             PyKCS11.LowLevel.CKO_PRIVATE_KEY),
                                            (PyKCS11.LowLevel.CKA_LABEL,
                                             PRIVATE_KEY_LABEL),
                                            (PyKCS11.LowLevel.CKA_ID,
                                             KEY_PAIR_ID)
                                            ])
    # Return first found private key object
    if len(priv_key_objects) > 0:
        return priv_key_objects[0]
    else:
        return False


def get_public_key_object():
    # Get public key objects by label and id
    pub_key_objects = session.findObjects([(PyKCS11.LowLevel.CKA_CLASS,
                                            PyKCS11.LowLevel.CKO_PUBLIC_KEY),
                                           (PyKCS11.LowLevel.CKA_LABEL,
                                            PUBLIC_KEY_LABEL),
                                           (PyKCS11.LowLevel.CKA_ID,
                                            KEY_PAIR_ID)
                                           ])
    # Return first found public key object
    if len(pub_key_objects) > 0:
        return pub_key_objects[0]
    else:
        return False


def create_rsa_keypair():
    public_key_template = [
        (PyKCS11.CKA_CLASS, PyKCS11.CKO_PUBLIC_KEY),
        (PyKCS11.CKA_PRIVATE, PyKCS11.CK_FALSE),
        (PyKCS11.CKA_TOKEN, PyKCS11.CK_TRUE),
        (PyKCS11.CKA_ENCRYPT, PyKCS11.CK_TRUE),
        (PyKCS11.CKA_VERIFY, PyKCS11.CK_TRUE),
        (PyKCS11.CKA_WRAP, PyKCS11.CK_TRUE),
        (PyKCS11.CKA_KEY_TYPE, PyKCS11.CKK_RSA),
        (PyKCS11.CKA_VERIFY_RECOVER, PyKCS11.CK_TRUE),
        (PyKCS11.CKA_LABEL, PUBLIC_KEY_LABEL),
        (PyKCS11.CKA_MODULUS_BITS, 2048),
        (PyKCS11.CKA_ID, KEY_PAIR_ID),
    ]

    private_key_template = [
        (PyKCS11.CKA_CLASS, PyKCS11.CKO_PRIVATE_KEY),
        (PyKCS11.CKA_TOKEN, PyKCS11.CK_TRUE),
        (PyKCS11.CKA_PRIVATE, PyKCS11.CK_TRUE),
        (PyKCS11.CKA_TOKEN, PyKCS11.CK_TRUE),
        (PyKCS11.CKA_SENSITIVE, PyKCS11.CK_TRUE),
        (PyKCS11.CKA_DECRYPT, PyKCS11.CK_TRUE),
        (PyKCS11.CKA_SIGN, PyKCS11.CK_TRUE),
        (PyKCS11.CKA_UNWRAP, PyKCS11.CK_TRUE),
        (PyKCS11.CKA_LABEL, PRIVATE_KEY_LABEL),
        (PyKCS11.CKA_ID, KEY_PAIR_ID),
    ]
    session.generateKeyPair(public_key_template, private_key_template)

def create_csr():
    # Command to run
    os.environ['OPENSSL_PIN'] = recover_pin(SO_PIN_FILE)
    print(os.environ['OPENSSL_PIN'])
    SUBJ = "/C=AE/ST=Abu Dhabi/L=Abu Dhabi/O=TII/OU=SSRC/CN=*.tii.ae"
    command = [
        'openssl',
        'req',
        '-new',
        '-engine', 'pkcs11',
        '-keyform', 'engine',
        '-key', str(KEY_ID),
        '-passin', 'env:OPENSSL_PIN',
        '-out', CSR_FILE,
        '-subj', SUBJ
    ]

    print("csr command: ", command)
    # Run the command
    result = subprocess.run(command, capture_output=True, text=True)

    # Check the result
    if result.returncode == 0:
        print("Command executed successfully.")
    else:
        print("Command execution failed.")
        print("Error output:")
        print(result.stderr)

def request_certificate():
    with open(CSR_FILE, 'rb') as file:
        csr = file.read()

    # Post the CSR to the signing server
    url = "http://localhost:80/api/devices/provision"
    headers = {"Content-Type": "application/json"}
    payload = {"csr": csr.decode("utf-8")}
    response = requests.post(url, headers=headers, json=payload)
    response.raise_for_status()

    # Extract the signed certificate from the response
    response_json = response.json()
    signed_certificate_data = response_json["certificate"]
    ca_certificate = response_json["ca_certificate"]

    print("### Printing received certificates from server: ###")
    print(signed_certificate_data)
    print("### Finished printing received certificates from server: ###")

    print("### Printing CA certificate from server ###")
    print(ca_certificate)
    print("### Finished printing CA certificate from server ###")

    # Fixme: It appears that server response "certificate" contains two
    #  certificates thus it is likely so called certificate chain.
    #  So perhaps "ca_certificate" is then "ID certificate". These needs to
    #  be double checked and certificate saving and querying needs to be fixed.

    # Save the signed certificate to SoftHSM
    try:
        # Read certificate as PEM format
        certificate = load_pem_x509_certificate(
            signed_certificate_data.encode("utf-8"))
        # Verify the validity of the received certificate
        current_time = datetime.now()
        if current_time > certificate.not_valid_after:
            print("Received certificate has already expired.")
            exit(1)

        # Convert the PEM-encoded certificate to DER format
        certificate_data = certificate.public_bytes(serialization.Encoding.DER)
        print(certificate.subject.get_attributes_for_oid(NameOID.COMMON_NAME)[
                  0].value.encode("utf-8"))
        # Store certificate to HSM
        cert_template = [
            (PyKCS11.LowLevel.CKA_CLASS, PyKCS11.LowLevel.CKO_CERTIFICATE),
            (PyKCS11.LowLevel.CKA_PRIVATE, PyKCS11.CK_FALSE),
            (PyKCS11.LowLevel.CKA_LABEL, CERTIFICATE_LABEL),
            (PyKCS11.LowLevel.CKA_TOKEN, PyKCS11.CK_TRUE),
            (PyKCS11.LowLevel.CKA_CERTIFICATE_TYPE, PyKCS11.CKC_X_509),
            (PyKCS11.LowLevel.CKA_MODIFIABLE, PyKCS11.CK_TRUE),
            (PyKCS11.LowLevel.CKA_VALUE, certificate_data),
            # must be BER-encoded
            # Todo: Subject information should be extracted from received
            #  certificate and filled accordingly
            (
                PyKCS11.CKA_SUBJECT,
                certificate.subject.get_attributes_for_oid(
                    NameOID.COMMON_NAME)[0].value.encode("utf-8"),
            ),
            # must be set and DER, see Table 24, X.509 Certificate Object Attributes
            (
                PyKCS11.CKA_ID,
                KEY_PAIR_ID,
            ),
            # must be set, and DER see Table 24, X.509 Certificate Object Attributes
        ]
        certificate_object = session.createObject(cert_template)
        print("Certificate saved successfully.")


    except pkcs11.PyKCS11Error as e:
        print("Failed to save certificate and private key:", e)

    # Get all objects in the session
    objects = session.findObjects()

    # Iterate over the objects and print their attributes
    for obj in objects:
        print("### Print all objects ###")
        print(obj)
        print("### Finished printing all objects ###")


print("pin:", recover_pin(SO_PIN_FILE))
if not get_private_key_object():
    create_rsa_keypair()
if not get_certificate():
    create_csr()
    request_certificate()

print("### List objects ###")
print("Private key objects:\n", get_private_key_object())
print("Public key objects:\n", get_public_key_object())
print("Certificate from HSM:\n", get_certificate())