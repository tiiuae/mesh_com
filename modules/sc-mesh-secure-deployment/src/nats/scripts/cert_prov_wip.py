import base64
import os
import random
import string
import subprocess

import argparse
import asyncio

import PyKCS11.LowLevel
import requests

from PyKCS11 import *

from cryptography.hazmat.primitives import serialization
from cryptography.x509.oid import NameOID
from cryptography.x509 import load_pem_x509_certificate
from cryptography.x509 import load_der_x509_certificate

from datetime import datetime

# To export pin
from Crypto.Protocol.KDF import PBKDF2
from Crypto.Hash import SHA256
from Crypto.Util.Padding import unpad
from Crypto.Cipher import AES

TOKEN_LABEL = "secccoms"
PRIVATE_KEY_LABEL = "Private Provisioning Key"
PUBLIC_KEY_LABEL = "Public Provisioning Key"
ID_CERTIFICATE_LABEL = "Device ID Certificate"
ROOT_CA_CERTIFICATE_LABEL = "Root CA Certificate"
KEY_LENGTH = 2048
KEY_PAIR_ID = (0x99,)
KEY_ID = 99

# File to store label, encrypted pin etc
#BASE_DIR = "/opt"
#BASE_DIR = "/home/saku/workspaces/output"
#HSM_PIN_DATA = BASE_DIR + "/hsm"
#CSR_PATH = BASE_DIR + "/csr"
#USER_PIN_FILE = "user_pin"
#SO_PIN_FILE = "so_pin"
#DEVICE_ID_FILE = "identity"
#CSR_FILE = "prov_csr.csr"
#PROVISIONING_SERVER_URL = "http://localhost:80/api/devices/provision"

class CommsHSMController:
    """
    Comms HSM Controller class.
    """
    def __init__(self, base_dir: str) -> None:
        print("CommsHSMController: __init__")
        self.pkcs11_session = None
        self.token_user_pin = ""
        self.token_so_pin = ""
        self.base_dir = base_dir
        self.user_pin_file = self.base_dir + "/hsm/user_pin"
        self.so_pin_file = self.base_dir + "/hsm/so_pin"
        
        
        self.use_soft_hsm = False
        self.login_required = False         # CKF_LOGIN_REQUIRED
        self.so_pin_required = False        # CKF_SO_PIN_LOCKED
        self.so_pin_to_be_changed = False   # CKF_SO_PIN_TO_BE_CHANGED
        self.user_pin_initialized = False   # CKF_USER_PIN_INITIALIZED
        self.token_has_rng = False          # CKF_RNG (token has Random Number Generator)
        self.token_label = "secccoms"       # Used with SoftHSM and related PIN encryption (needs imrpovements)
        
        # Instantiate PyKCS11Lib
        self.pkcs11 = PyKCS11.PyKCS11Lib()
        # Load library into use
        self.pkcs11.load(self.__get_hsm_library())
    
    def open_session(self):
        print("CommsHSMController: open_session")
        self.token_user_pin = self.__recover_pin(self.user_pin_file)
        self.token_so_pin = self.__recover_pin(self.so_pin_file)
        self.pkcs11_session = self.__get_pkcs11_session()
        
        # SoftHSM doesn't have valid token after flashing
        # but one needs to be created.
        if self.pkcs11_session is None and self.use_soft_hsm is True:
            self.__create_token(TOKEN_LABEL)
            self.pkcs11_session = self.__get_pkcs11_session()
        
        if self.pkcs11_session is None:
            return False
        else:
            return True
    
    def close_session(self):
        if self.pkcs11_session:
            self.pkcs11_session.closeSession()
            self.pkcs11_session = None
 
    def __get_comms_pcb_version(self, file_path):
        try:
            with open(file_path, 'r') as file:
                content = file.read()
                for line in content.splitlines():
                    if line.startswith("COMMS_PCB_VERSION="):
                        version = line.split("=")[1]
                        return version
        except FileNotFoundError:
            return 0
    
    def __get_hsm_library(self):
        
        # CM2.0 has SE050_C i.e. is using libsss_pkcs11.so
        if self.__get_comms_pcb_version("/opt/hardware/comms_pcb_version") == 1:
            path = "/usr/lib/libsss_pkcs11.so"
            if os.path.exists(path):
                self.use_soft_hsm = False
                print(path)
                return path
        # Use SoftHSM for others
        paths_to_check = [
            "/usr/local/lib/softhsm/libsofthsm2.so",
            "/usr/lib/softhsm/libsofthsm2.so"
            ]   
        for path in paths_to_check:
            if os.path.exists(path):
                self.use_soft_hsm = True
                print(path)
                return path
        return None

    def __recover_pin(self, filename):
        try:
            with open(filename) as file:
                pin_aux = file.readlines()  # need to make it absolute
            # Determine salt and ciphertext
            encryptedDataB64 = pin_aux[0].split('\n')[0]
            encryptedData = base64.b64decode(encryptedDataB64)
            salt = encryptedData[8:16]
            ciphertext = encryptedData[16:]
            # Reconstruct Key/IV-pair
            pbkdf2Hash = PBKDF2(self.token_label, salt, 32 + 16, count=100000,
                                hmac_hash_module=SHA256)
            key = pbkdf2Hash[:32]
            iv = pbkdf2Hash[32:32 + 16]
            # Decrypt with AES-256 / CBC / PKCS7 Padding
            cipher = AES.new(key, AES.MODE_CBC, iv)
            return unpad(cipher.decrypt(ciphertext), 16).decode().split('\n')[0]
        except FileNotFoundError:
            print("No pin found")
            return ""
        
    def __generate_pin(self):
        pin_length = 6
        # Generate a random 6-digit PIN
        pin = ''.join(random.choices(string.digits, k=pin_length))
        return pin

    def __export_pin(self, pin, file_name, label):
         # Create directories if they don't exist
        os.makedirs(os.path.dirname(file_name), exist_ok=True)

        # Remove the output file if it already exists
        if os.path.isfile(file_name):
            os.remove(file_name)

        # Execute the openssl command to encrypt and export the PIN
        openssl_command = f'echo "{pin}" | openssl enc -aes-256-cbc -md sha256 -a -pbkdf2 -iter 100000 -salt -pass pass:{label} > {file_name}'
        print(openssl_command)
        ret = subprocess.run(openssl_command, shell=True)
        if ret.returncode != 0:
            print(str(ret.returncode) + str(ret.stdout) + str(ret.stderr))
            return False
        return True

    def __get_pkcs11_session(self):
        slots = self.pkcs11.getSlotList(tokenPresent=True)
        for slot in slots:
            try:
                session = self.pkcs11.openSession(slot,
                                            PyKCS11.LowLevel.CKF_SERIAL_SESSION | 
                                            PyKCS11.LowLevel.CKF_RW_SESSION)
                print("SlotID:", session.getSessionInfo().slotID)
                token_info = self.pkcs11.getTokenInfo(slot)
                
                # Check login requirements
                self.__get_token_info(slot)
                if  self.login_required:
                    print("login required")
                    if self.user_pin_initialized is False:
                        print("login so user")
                        # Login as security officer
                        session.login(pin=self.token_so_pin,
                                    user_type=PyKCS11.LowLevel.CKU_SO)
                        # Generate user pin for the token
                        self.token_user_pin = self.__generate_pin()
                        # Encrypt and export pin for later usage
                        self.__export_pin(self.token_user_pin, self.user_pin_file, self.token_label)
                        # Set user pin for the token
                        session.initPin(self.token_user_pin)
                        # Log out SO
                        session.logout()
                    # Log in as normal user
                    print("login normal user")
                    session.login(self.token_user_pin,
                                user_type=PyKCS11.LowLevel.CKU_USER)
                    # Refresh login requirements
                    self.__get_token_info(slot)
                print("so_pin", self.token_so_pin)
                print("user_pin", self.token_user_pin)
                return session
            except PyKCS11Error as e:
                print("exception:", e)
                pass
        return None


    def __create_token(self, token_name):
        slots = self.pkcs11.getSlotList(tokenPresent=False)
        for slot in slots:
            try:
                self.__get_token_info(slot)
                if self.so_pin_required:
                    # Generate security officer pin for the token
                    self.token_so_pin = self.__generate_pin()
                    # Encrypt and export pins for later usage
                    self.__export_pin(self.token_so_pin, self.so_pin_file, token_name)
                # Initialize token                  
                self.pkcs11.initToken(slot, self.token_so_pin, token_name)
                break
            except PyKCS11Error as e:
                print("Create token exception:", e)
                pass

    def __get_token_info(self, slot):
        # Get token info
        token_info = self.pkcs11.getTokenInfo(slot)

        # Get token name and strip out extra zeroes (seen at least with SoftHSM label names)
        self.token_label = token_info.label.replace("\x00", "")
        
        # Set few member variables from token flags bitmask
        self.login_required = bool(token_info.flags & PyKCS11.LowLevel.CKF_LOGIN_REQUIRED)
        self.so_pin_required = bool(token_info.flags & PyKCS11.LowLevel.CKF_SO_PIN_LOCKED)
        self.so_pin_to_be_changed = bool(token_info.flags & PyKCS11.LowLevel.CKF_SO_PIN_TO_BE_CHANGED)
        self.user_pin_initialized = bool(token_info.flags & PyKCS11.LowLevel.CKF_USER_PIN_INITIALIZED)
        self.token_has_rng = bool(token_info.flags & PyKCS11.LowLevel.CKF_RNG)

        print("login_required", self.login_required)
        print("so_pin_required", self.so_pin_required)
        print("so_pin_to_be_changed", self.so_pin_to_be_changed)
        print("user_pin_initialized", self.user_pin_initialized)
        print("token_has_rng", self.token_has_rng)
   
    def save_certificate(self, cert_pem, label):
        try:
            print("Certificate Subject:", cert_pem.subject)
            print("Certificate Issuer:", cert_pem.issuer.rfc4514_string())
            print("Certificate Not Before:", cert_pem.not_valid_before)
            print("Certificate Not After:", cert_pem.not_valid_after)
            print("Certificate Serial Number:", cert_pem.serial_number)
            print("Certificate version:",cert_pem.version)
            print("=" * 50)            
            
            # Verify the validity of the received certificate
            current_time = datetime.now()
            if current_time > cert_pem.not_valid_after:
                print("Received certificate has already expired.")
                exit(1)


            # Convert the PEM-encoded certificate to DER format
            certificate_data = cert_pem.public_bytes(serialization.Encoding.DER)
            
            # Convert the serial number integer to a byte string
            serial_number_int = cert_pem.serial_number
            serial_number_bytes = serial_number_int.to_bytes((serial_number_int.bit_length() + 7) // 8, byteorder='big')
            
            # DER-encoded serial number: Tag + Length + Value
            serial_number_der = b"\x02" + bytes([len(serial_number_bytes)]) + serial_number_bytes

            # Fill the certificate template
            cert_template = [
                (PyKCS11.LowLevel.CKA_CLASS, PyKCS11.LowLevel.CKO_CERTIFICATE),
                (PyKCS11.LowLevel.CKA_PRIVATE, PyKCS11.CK_FALSE),
                (PyKCS11.LowLevel.CKA_LABEL, label),
                (PyKCS11.LowLevel.CKA_TOKEN, PyKCS11.CK_TRUE),
                (PyKCS11.LowLevel.CKA_CERTIFICATE_TYPE, PyKCS11.CKC_X_509),
                (PyKCS11.LowLevel.CKA_MODIFIABLE, PyKCS11.CK_TRUE),
                (PyKCS11.LowLevel.CKA_VALUE, certificate_data),
                (PyKCS11.LowLevel.CKA_SUBJECT, cert_pem.subject.rfc4514_string().encode('utf-8')),
                (PyKCS11.LowLevel.CKA_ID, KEY_PAIR_ID),
                (PyKCS11.LowLevel.CKA_ISSUER, cert_pem.issuer.rfc4514_string().encode('utf-8')),
                (PyKCS11.LowLevel.CKA_START_DATE, cert_pem.not_valid_before.strftime("%Y%m%d").encode('utf-8')),
                (PyKCS11.LowLevel.CKA_END_DATE, cert_pem.not_valid_after.strftime("%Y%m%d").encode('utf-8')),
                (PyKCS11.LowLevel.CKA_SERIAL_NUMBER, serial_number_der),

            ]
            # Store certificate to HSM
            certificate_object = self.pkcs11_session.createObject(cert_template)
            print("Certificate saved successfully.")
            return True

        except PyKCS11Error as e:
            print("Failed to save certificate:", e)
            return False

    def get_certificate(self):
        # Get certificate objects by label and id
        certificate_objects = self.pkcs11_session.findObjects([(PyKCS11.LowLevel.CKA_CLASS,
                                                            PyKCS11.LowLevel.CKO_CERTIFICATE),
                                                            (PyKCS11.LowLevel.CKA_LABEL,
                                                             ID_CERTIFICATE_LABEL),
                                                            (PyKCS11.LowLevel.CKA_ID, 
                                                             KEY_PAIR_ID)])
        if len(certificate_objects) > 0:
            print(certificate_objects[0])
            for certificate_object in certificate_objects:
                certificate_bytes = bytes(self.pkcs11_session.getAttributeValue(certificate_object,
                                                                    [PyKCS11.LowLevel.CKA_VALUE])[0])

                # Verify the validity of the certificate
                certificate = load_der_x509_certificate(certificate_bytes)
                current_time = datetime.now()
                if certificate.not_valid_before <= current_time <= certificate.not_valid_after:
                    # Return first valid certiticate as PEM format
                    return certificate.public_bytes(
                        encoding=serialization.Encoding.PEM).decode()
                else:
                    print("Certificate is not valid.")
                    # Delete invalid certificate
                    self.pkcs11_session.destroyObject(certificate_object)
                    print("Invalid certificate has been deleted.")
        return None


    def get_private_key_object(self):
        # Get private key objects by label and id
        priv_key_objects = self.pkcs11_session.findObjects([(PyKCS11.LowLevel.CKA_CLASS,
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
            return None


    def get_public_key_object(self):
        # Get public key objects by label and id
        pub_key_objects = self.pkcs11_session.findObjects([(PyKCS11.LowLevel.CKA_CLASS,
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
            return None


    def create_rsa_keypair(self):
        
        public_key_template = [
            (PyKCS11.CKA_CLASS, PyKCS11.CKO_PUBLIC_KEY),
            (PyKCS11.CKA_PRIVATE, PyKCS11.CK_FALSE),
            (PyKCS11.CKA_TOKEN, PyKCS11.CK_TRUE),
            (PyKCS11.CKA_ENCRYPT, PyKCS11.CK_TRUE),
            (PyKCS11.CKA_VERIFY, PyKCS11.CK_TRUE),
            (PyKCS11.CKA_WRAP, PyKCS11.CK_TRUE),
            (PyKCS11.CKA_KEY_TYPE, PyKCS11.CKK_RSA),
            (PyKCS11.CKA_VERIFY_RECOVER, PyKCS11.CK_TRUE),
            (PyKCS11.CKA_LABEL, PUBLIC_KEY_LABEL),  # Needs to be changed to work with HW HSM
            (PyKCS11.CKA_MODULUS_BITS, 2048),
            (PyKCS11.CKA_ID, KEY_PAIR_ID),          # Needs to be changed to work with HW HSM
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
        self.pkcs11_session.generateKeyPair(public_key_template, private_key_template)

    def create_csr(self, priv_key_id, subject, filename):
        # Create directories if they don't exist
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        
        # Openssl tool requires environment variable usage
        os.environ['PKCS11_PIN'] = self.token_user_pin

        command = [
            'openssl',
            'req',
            '-new',
            '-engine', 'pkcs11',
            '-keyform', 'engine',
            '-key', str(priv_key_id),
            '-passin', 'env:PKCS11_PIN',
            '-out', str(filename),
            '-subj', str(subject)
        ]

        print("csr command: ", command)
        # Run the command
        result = subprocess.run(command, capture_output=True, text=True)

        # Check the result
        if result.returncode == 0:
            print("Command executed successfully.")
            return True
        else:
            print("Command execution failed.")
            print("Error output:")
            print(result.stderr)
            return False


class CommsProvisioning:
    """
    Comms Provisioning class.

    NoCertificate
        RequestCertificate
        store certificate
    else
        Exit
    """
    def __init__(self, server: str = "localhost", port: str = "80", outdir: str = "/opt") -> None:
        self.outdir = outdir
        self.device_id_file = self.outdir + "/identity"
        self.provisioning_csr_file = self.outdir + "/csr/prov_csr.csr"
        self.is_provisioned = False
        self.server_url = "http://" + server + ":" + port + "/api/devices/provision"
        self.device_id = self.__get_device_id()
        self.csr_subject = "/CN=" + self.device_id
        self.hsm_ctrl = CommsHSMController(self.outdir)
        self.hsm_ctrl.open_session()

    def __get_device_id(self):
        try:
            with open(self.device_id_file) as file:
                device_id = file.readline()
                return device_id
        except FileNotFoundError:
            return "None"

    def check_is_provisioned(self):
        if self.hsm_ctrl.get_certificate() is None:
            self.is_provisioned = False
        else:
            self.is_provisioned = True
        return self.is_provisioned 
    
    def do_provisioning(self):
        prov_agent.hsm_ctrl.create_csr(priv_key_id=str(KEY_ID), 
                                       subject=self.csr_subject, 
                                       filename = self.provisioning_csr_file)
        prov_agent.request_certificate()

 
    def request_certificate(self):
        with open(self.provisioning_csr_file, 'rb') as file:
            csr = file.read()

        # Post the CSR to the signing server
        url =  self.server_url
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

        print("### Printing root CA certificate from server ###")
        print(ca_certificate)
        print("### Finished printing CA certificate from server ###")

        # Provisioning server response "certificate" contains two
        # certificates i.e. the device id specific client certificate 
        # and the provisioning CA certificate.  
        # => Split the PEM data into individual certificates
        signed_certificates = signed_certificate_data.split('-----END CERTIFICATE-----\n')
        
        # Remove any empty strings from the split operation
        signed_certificates = [cert.strip() for cert in signed_certificates if cert.strip()]

        # Save certificates
        for index, cert_data in enumerate(signed_certificates):
            cert_data = cert_data + "\n-----END CERTIFICATE-----\n"
            
            # Read certificate as PEM format
            certificate = load_pem_x509_certificate(
                cert_data.encode("utf-8"))
            
            device_id = self.device_id

            if device_id in certificate.subject.rfc4514_string():
                label = ID_CERTIFICATE_LABEL
            else:
                label = ID_CERTIFICATE_LABEL + " Issuer " + str(index)
            self.hsm_ctrl.save_certificate(certificate, label)
        
        # Save root CA certificate
        certificate = load_pem_x509_certificate(ca_certificate.encode("utf-8"))
        label = ROOT_CA_CERTIFICATE_LABEL
        self.hsm_ctrl.save_certificate(certificate, label)

        # Get all objects in the session
        #objects = session.findObjects()

        # Iterate over the objects and print their attributes
        #for obj in objects:
        #    print("### Print all objects ###")
        #    print(obj)
        #print("### Finished printing all objects ###")

#print("### List objects ###")
#print("Private key objects:\n", get_private_key_object())
#print("Public key objects:\n", get_public_key_object())
#print("Certificate from HSM:\n", get_certificate())

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Provisioning agent settings')
    parser.add_argument('-s', '--server', help='Provisioning Server IP', required=False)
    parser.add_argument('-p', '--port', help='Server port', required=False)
    parser.add_argument('-o', '--outdir', help='Output folder for files', required=False)
    args = parser.parse_args()

    prov_agent = CommsProvisioning(outdir="/home/saku/workspaces/output")

    # TODO: Refactor this logic and perform it in loop 
    # for N time and indicate status somehow (blink some leds?)
    if prov_agent.hsm_ctrl.get_private_key_object() is None:
        prov_agent.hsm_ctrl.create_rsa_keypair()
    
    if prov_agent.check_is_provisioned() is False:
       prov_agent.do_provisioning()