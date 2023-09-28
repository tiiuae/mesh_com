"""
This module contains CommsHSMController class which provides
HSM control functions for its client.
"""
import base64
import os
import random
import string
import subprocess
from datetime import datetime

import PyKCS11.LowLevel
from PyKCS11 import PyKCS11Error
from PyKCS11 import Mechanism

from cryptography.hazmat.primitives import serialization
from cryptography.x509 import Certificate
from cryptography.x509 import load_der_x509_certificate

# To create CSR manually
from asn1crypto import x509
from asn1crypto import csr
from asn1crypto import keys
from asn1crypto import pem
from asn1crypto import core

# To export pin
from Crypto.Protocol.KDF import PBKDF2
from Crypto.Hash import SHA256
from Crypto.Util.Padding import unpad
from Crypto.Cipher import AES

# To get IP addresses for CSR SAN extension
import netifaces as ni


class Cert:
    """
    A class to hold certificate PEM data and validity star/end dates.
    """

    def __init__(self):
        self.pem_data = ""
        self.validity_start = datetime.fromtimestamp(0)
        self.validity_end = datetime.fromtimestamp(0)


class KeyParams:
    """
    A class to hold keypair generation and storing related params.

    Attributes:
        id (str) -- HSM object identifier for the keypair
        label (str) -- HSM object label
        private_key (str) -- Private key file name (full path).
        public_key (str) -- Public key file name (full path).
        algorithm (str) -- Supported values "rsa" or "ec"
        modulus_bits (int) -- Number of modulus bits used for RSA type of key.
        ec_param (str) -- Elliptic curve param used with EC type of key generation.
        use_openssl (bool) -- Defines whether keypair is create via openssl or not.

    """
    def __init__(self, id: str, label: str, private_key: str, public_key: str,
                 algorithm: str = "rsa", modulus_bits: int = 4096,
                 ec_param: str = "prime256v1", use_openssl: bool = True):
        self.id = id
        self.label = label
        self.private_key = private_key
        self.public_key = public_key
        self.algorithm = algorithm
        self.modulus_bits = modulus_bits
        self.ec_param = ec_param
        self.use_openssl = use_openssl


class CsrParams:
    """
    A class to hold CSR params.

    Attributes:
        key_id (str) -- HSM id of the private key to sign the CSR
        device_id (str) -- Device ID to be used as common name.
        filename (str) -- Output filename for the CSR (full path).
        is_server (bool) -- Boolean value to define whether to request
                            certificate for server or client authentication
                            purposes.
        use_openssl (bool) -- Defines whether CSR is create via openssl or not.
    """
    def __init__(self, key_id: str, device_id: str, filename: str,
                 is_server: bool = False, use_openssl: bool = False):
        self.key_id = key_id
        self.device_id = device_id
        self.filename = filename
        self.is_server = is_server
        self.use_openssl = use_openssl


class CommsHSMController:
    """
    Comms HSM Controller class.
    """
    def __init__(self, base_dir: str, board_version: float, logger) -> None:
        self.__pkcs11_session = None
        self.__token_user_pin = ""
        self.__token_so_pin = ""
        self.__base_dir = base_dir
        self.__user_pin_file = self.__base_dir + "/hsm/user_pin"
        self.__so_pin_file = self.__base_dir + "/hsm/so_pin"
        self.__comms_board_version = board_version
        self.__ip_addresses = ["10.10.10.2", "10.10.20.2"]

        self.log = logger
        self.__use_soft_hsm = False
        self.__login_required = False        # CKF_LOGIN_REQUIRED
        self.__so_pin_required = False       # CKF_SO_PIN_LOCKED
        self.__so_pin_to_be_changed = False  # CKF_SO_PIN_TO_BE_CHANGED
        self.__user_pin_initialized = False  # CKF_USER_PIN_INITIALIZED
        self.__token_has_rng = False         # CKF_RNG (Random Number Generator)

        # Used with SoftHSM and related PIN encryption
        self.__token_label = "nats_token"
        self.__current_token_label = ""

        # Define configuration files for openssl
        self.__csr_cnf_file = self.__base_dir + "/csr.cnf"
        os.environ['OPENSSL_CONF'] = self.__base_dir + "/comms_openssl.cnf"

        self.__pkcs11_engine = ""
        self.__pkcs11lib = ""
        # Set engine and lib config
        self.__set_openssl_engine_properties()

        # Instantiate PyKCS11Lib
        self.__pkcs11 = PyKCS11.PyKCS11Lib()
        # Load library into use
        self.__pkcs11.load(self.__pkcs11lib)

    def open_session(self):
        """
        Opens PKCS11 control session.

        Returns:
            bool: True if session was opened, False otherwise.
        """
        self.__token_user_pin = self.__recover_pin(self.__user_pin_file)
        self.__token_so_pin = self.__recover_pin(self.__so_pin_file)
        self.__pkcs11_session = self.__get_pkcs11_session()

        # SoftHSM doesn't have valid token after flashing
        # but one needs to be created.
        if self.__pkcs11_session is None and self.__use_soft_hsm is True:
            self.__create_token(self.__token_label)
            self.__pkcs11_session = self.__get_pkcs11_session()

        if self.__pkcs11_session is None:
            return False
        # Success
        return True

    def close_session(self):
        """
        Closes PKCS11 control session
        """
        if self.__pkcs11_session:
            self.__pkcs11_session.closeSession()
            self.__pkcs11_session = None

    def __del__(self):
        self.close_session()

    def __set_openssl_engine_properties(self):

        # CM2.0 has SE050_C i.e. is using libsss_pkcs11.so
        if self.__comms_board_version == 1:
            path = "/usr/lib/libsss_pkcs11.so"
            if os.path.exists(path):
                self.__use_soft_hsm = False
                self.log.debug("pkcs11 library path %s", path)
                self.__pkcs11lib = path
                self.__pkcs11_engine = "e4sss"
                # Force SoftHSM in use for now
                # return None

        # Use SoftHSM for others
        paths_to_check = [
            "/usr/local/lib/softhsm/libsofthsm2.so",
            "/usr/lib/softhsm/libsofthsm2.so"
        ]

        for path in paths_to_check:
            if os.path.exists(path):
                self.__use_soft_hsm = True
                self.__pkcs11lib = path
                self.__pkcs11_engine = "pkcs11"
                self.log.debug("pkcs11 library path %s", path)
                return None
        return None

    def __recover_pin(self, filename):
        try:
            with open(filename, 'r', encoding='utf-8') as file:
                pin_aux = file.readlines()  # need to make it absolute
            # Determine salt and ciphertext
            encryptedDataB64 = pin_aux[0].split('\n')[0]
            encryptedData = base64.b64decode(encryptedDataB64)
            salt = encryptedData[8:16]
            ciphertext = encryptedData[16:]
            # Reconstruct Key/IV-pair
            pbkdf2Hash = PBKDF2(self.__token_label, salt, 32 + 16, count=100000,
                                hmac_hash_module=SHA256)
            key = pbkdf2Hash[:32]
            iv = pbkdf2Hash[32:32 + 16]
            # Decrypt with AES-256 / CBC / PKCS7 Padding
            cipher = AES.new(key, AES.MODE_CBC, iv)
            return unpad(cipher.decrypt(ciphertext), 16).decode().split('\n')[0]
        except Exception as e:
            self.log.debug("No pin found, error: %s", e)
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
        openssl_command = (
            f'echo "{pin}" | openssl enc -aes-256-cbc -md sha256 -a '
            f'-pbkdf2 -iter 100000 -salt -pass pass:{label}'
            f' > {file_name}'
        )

        ret = subprocess.run(openssl_command, shell=True)
        if ret.returncode != 0:
            self.log.debug("Return Code: %s\nSTDOUT: %s\nSTDERR: %s",
                           ret.returncode, ret.stdout, ret.stderr)
            return False
        return True

    def __get_pkcs11_session(self):
        slots = self.__pkcs11.getSlotList(tokenPresent=True)
        for slot in slots:
            try:
                session = self.__pkcs11.openSession(
                    slot, PyKCS11.LowLevel.CKF_SERIAL_SESSION |
                    PyKCS11.LowLevel.CKF_RW_SESSION)

                # Check login requirements
                self.__get_token_info(slot)
                if self.__login_required:
                    if self.__user_pin_initialized is False:
                        # Login as security officer
                        session.login(pin=self.__token_so_pin,
                                      user_type=PyKCS11.LowLevel.CKU_SO)
                        # Generate user pin for the token
                        self.__token_user_pin = self.__generate_pin()
                        # Encrypt and export pin for later usage
                        self.__export_pin(self.__token_user_pin,
                                          self.__user_pin_file,
                                          self.__token_label)
                        # Set user pin for the token
                        session.initPin(self.__token_user_pin)
                        # Log out Security Officer
                        session.logout()
                    # Log in as normal user
                    session.login(self.__token_user_pin, user_type=PyKCS11.LowLevel.CKU_USER)
                    # Refresh login requirements
                    self.__get_token_info(slot)
                self.log.debug("Token: %s", self.__current_token_label)
                self.log.debug("so_pin %s", self.__token_so_pin)
                self.log.debug("user_pin %s", self.__token_user_pin)
                return session
            except PyKCS11Error as e:
                self.log.debug("exception: %s", e)
        return None

    def __create_token(self, token_name):
        slots = self.__pkcs11.getSlotList(tokenPresent=False)
        for slot in slots:
            try:
                self.__get_token_info(slot)
                if self.__so_pin_required:
                    # Generate security officer pin for the token
                    self.__token_so_pin = self.__generate_pin()
                    # Encrypt and export pins for later usage
                    self.__export_pin(self.__token_so_pin, self.__so_pin_file, token_name)
                # Initialize token
                self.__pkcs11.initToken(slot, self.__token_so_pin, token_name)
                break
            except PyKCS11Error as e:
                self.log.debug("Create token exception: %s", e)

    def __get_token_info(self, slot):
        # Get token info
        token_info = self.__pkcs11.getTokenInfo(slot)

        # Get token name and strip out extra zeroes (seen at least with SoftHSM label names)
        self.__current_token_label = token_info.label.replace("\x00", "").strip()

        # Set few member variables from token flags bitmask
        self.__login_required = bool(token_info.flags & PyKCS11.LowLevel.CKF_LOGIN_REQUIRED)
        self.__so_pin_required = bool(token_info.flags & PyKCS11.LowLevel.CKF_SO_PIN_LOCKED)
        self.__so_pin_to_be_changed = bool(
            token_info.flags & PyKCS11.LowLevel.CKF_SO_PIN_TO_BE_CHANGED)
        self.__user_pin_initialized = bool(
            token_info.flags & PyKCS11.LowLevel.CKF_USER_PIN_INITIALIZED)
        self.__token_has_rng = bool(token_info.flags & PyKCS11.LowLevel.CKF_RNG)

        """
        self.log.debug("login_required %s", self.__login_required)
        self.log.debug("so_pin_required %s", self.__so_pin_required)
        self.log.debug("so_pin_to_be_changed %s", self.__so_pin_to_be_changed)
        self.log.debug("user_pin_initialized %s", self.__user_pin_initialized)
        self.log.debug("token_has_rng %s", self.__token_has_rng)
        mechanisms = self.__pkcs11.getMechanismList(slot)
        for mech in mechanisms:
            self.log.debug(mech)
        """

    def save_certificate(self, cert_pem: Certificate, keypair_id, label: str):
        """
        Saves provided certificate into HSM using provided arguments.

        Arguments:
            cert_pem (Certificate) -- Certificate to save into HSM.
            keypair_id (str) -- ID number for the certificate object.
            label (str) -- Certificate label (used with SoftHSM only).

        Returns:
            bool: True if certificate was saved, False otherwise.
        """
        try:
            self.log.debug("=" * 70)
            self.log.debug("Certificate label: %s", label)
            self.log.debug("Certificate subject: %s", cert_pem.subject)
            self.log.debug("Certificate issuer: %s", cert_pem.issuer.rfc4514_string())
            self.log.debug("Certificate not Before: %s", cert_pem.not_valid_before)
            self.log.debug("Certificate not After: %s", cert_pem.not_valid_after)
            self.log.debug("Certificate serial Number %s:", cert_pem.serial_number)
            self.log.debug("Certificate version: %s", cert_pem.version)

            cka_id = bytes.fromhex(keypair_id)

            # Verify the validity of the received certificate
            current_time = datetime.now()
            if current_time > cert_pem.not_valid_after:
                self.log.debug("Received certificate has already expired.")
                return False

            # Convert the PEM-encoded certificate to DER format
            certificate_data = cert_pem.public_bytes(serialization.Encoding.DER)

            # Convert the serial number integer to a byte string
            serial_number_int = cert_pem.serial_number
            serial_number_bytes = serial_number_int.to_bytes(
                (serial_number_int.bit_length() + 7) // 8, byteorder='big')

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
                (PyKCS11.LowLevel.CKA_ID, cka_id),
                (PyKCS11.LowLevel.CKA_ISSUER, cert_pem.issuer.rfc4514_string().encode('utf-8')),
                (PyKCS11.LowLevel.CKA_START_DATE,
                 cert_pem.not_valid_before.strftime("%Y%m%d").encode('utf-8')),
                (PyKCS11.LowLevel.CKA_END_DATE,
                 cert_pem.not_valid_after.strftime("%Y%m%d").encode('utf-8')),
                (PyKCS11.LowLevel.CKA_SERIAL_NUMBER, serial_number_der),
            ]

            # Store certificate to HSM
            self.__pkcs11_session.createObject(cert_template)
            self.log.debug("Certificate saved successfully.")
            return True

        except PyKCS11Error as e:
            self.log.debug("Failed to save certificate: %s", e)
            return False

    def get_certificate(self, keypair_id, label):
        """
        Returns the first valid certificate in PEM format found with
        the given arguments.

        Arguments:
            keypair_id (str) -- Identifier number for the certicate.
            label (str) -- Label name for the certificate (used with SoftHSM only).

        Returns:
            str or None: Certificate in PEM format or None if not found.
        """
        cka_id = bytes.fromhex(keypair_id)
        cert = Cert()

        filter_template = [
            (PyKCS11.LowLevel.CKA_CLASS, PyKCS11.LowLevel.CKO_CERTIFICATE),
            (PyKCS11.LowLevel.CKA_ID, cka_id)
        ]

        if self.__use_soft_hsm:
            filter_template.append((PyKCS11.LowLevel.CKA_LABEL, label))

        # Get certificate objects by label and id
        certificate_objects = self.__pkcs11_session.findObjects(filter_template)

        if len(certificate_objects) > 0:
            for certificate_object in certificate_objects:
                cka_value = self.__pkcs11_session.getAttributeValue(certificate_object,
                                                                    [PyKCS11.LowLevel.CKA_VALUE])[0]
                if cka_value:
                    certificate_bytes = bytes(cka_value)
                    certificate = load_der_x509_certificate(certificate_bytes)
                    cert.pem_data = certificate.public_bytes(encoding=serialization.Encoding.PEM).decode()
                    cert.validity_start = certificate.not_valid_before
                    cert.validity_end = certificate.not_valid_after
                    return cert
        return None

    def has_private_key(self, keypair_id, label):
        """
        Finds a private key object from the HSM using the provided arguments.

        Arguments:
            keypair_id (str) -- Identifier number for the private key.
            label (str) -- Label name for the private key (used with SoftHSM only).

        Returns:
            bool: True if the private key exists, False otherwise.
        """
        cka_id = bytes.fromhex(keypair_id)

        filter_template = [
            (PyKCS11.LowLevel.CKA_CLASS, PyKCS11.LowLevel.CKO_PRIVATE_KEY),
            (PyKCS11.LowLevel.CKA_ID, cka_id)
        ]

        if self.__use_soft_hsm:
            filter_template.append((PyKCS11.LowLevel.CKA_LABEL, label))

        # Get private key objects by label and id
        priv_key_objects = self.__pkcs11_session.findObjects(filter_template)

        if len(priv_key_objects) > 0:
            # There is at least one matching object
            return True
        # Not found
        return False

    def has_public_key(self, keypair_id, label):
        """
        Finds a publick key object from the HSM using the provided arguments.

        Arguments:
            keypair_id (str) -- Identifier number for the publick key.
            label (str) -- Label name for the public key (used with SoftHSM only).

        Returns:
            bool: True if the public key exists, False otherwise.
        """
        cka_id = bytes.fromhex(keypair_id)

        filter_template = [
            (PyKCS11.LowLevel.CKA_CLASS, PyKCS11.LowLevel.CKO_PUBLIC_KEY),
            (PyKCS11.LowLevel.CKA_ID, cka_id)
        ]

        if self.__use_soft_hsm:
            filter_template.append((PyKCS11.LowLevel.CKA_LABEL, label))

        # Get public key objects by label and id
        pub_key_objects = self.__pkcs11_session.findObjects(filter_template)
        if len(pub_key_objects) > 0:
            # There is at least one matching object
            return True
        # Not found
        return False

    def generate_keypair(self, key_params: KeyParams) -> bool:
        """
        Generates private/public keypair using given paramters.

        Arguments:
            key_params (KeyParams) -- KeyParams object.

        Returns:
            bool: True if the keypair was generated, False otherwise.
        """
        if key_params.algorithm != "rsa" and key_params.algorithm != "ec":
            print(key_params.algorithm)
            self.log.debug("Not supported algorithm!")
            return False

        if key_params.use_openssl:
            retval = self.__generate_keypair_via_openssl(key_params)
        else:
            if key_params.algorithm == "rsa":
                retval = self.__generate_rsa_keypair(key_params.id, key_params.label)
            else:
                retval = self.__generate_ec_keypair(key_params.id, key_params.label)
        return retval

    def __generate_keypair_via_openssl(self, key_params: KeyParams) -> bool:
        """
        Generates authentication keypair via openssl.

        Arguments:
           key_params (KeyParams) -- KeyParams object.

        Returns:
            bool: True if the keypair was generated, False otherwise.
        """
        # Openssl requires environment variable usage for pin
        os.environ['PKCS11_PIN'] = self.__token_user_pin
        print("pin:", self.__token_user_pin)

        # Create directories if they don't exist
        os.makedirs(os.path.dirname(key_params.private_key), exist_ok=True)
        os.makedirs(os.path.dirname(key_params.public_key), exist_ok=True)

        label = key_params.label
        if self.__pkcs11_engine == "e4sss":
            label = "sss:" + key_params.id

        # Generate private key
        if key_params.algorithm == "rsa":
            command = ['openssl', 'genpkey', '-algorithm', 'RSA', '-out',
                       key_params.private_key, '-engine', self.__pkcs11_engine]
        elif key_params.algorithm == "ec":
            command = ['openssl', 'ecparam', '-name', key_params.ec_param, '-genkey',
                       '-out', key_params.private_key, '-engine', self.__pkcs11_engine]
        else:
            self.log.debug("Not supported algorithm!")
            return False

        # Run the command
        result = subprocess.run(command, capture_output=True, text=True)
        # Check the result
        if result.returncode != 0:
            self.log.debug("Private key generation failed:")
            self.log.debug(result.stderr)
            return False

        # Generate public key from private
        command = ['openssl', key_params.algorithm, '-in', key_params.private_key,
                   '-pubout', '-out', key_params.public_key, '-engine', self.__pkcs11_engine]

        # Run the command
        result = subprocess.run(command, capture_output=True, text=True)
        # Check the result
        if result.returncode != 0:
            self.log.debug("Public key generation failed:")
            self.log.debug(result.stderr)
            return False

        # Import private key to HSM
        command = [
            'pkcs11-tool',
            '--module', self.__pkcs11lib,
            '--token', self.__current_token_label,
            '--login',
            '--pin', 'env:PKCS11_PIN',
            '--label', label,
            '--id', key_params.id,
            '--write-object', key_params.private_key,
            '--type', 'privkey'
            ]

        # Run the command
        result = subprocess.run(command, capture_output=True, text=True)
        # Check the result
        if result.returncode != 0:
            self.log.debug("Private key import to HSM failed:")
            self.log.debug(result.stderr)
            return False

        # Import public key to HSM
        command = [
            'pkcs11-tool',
            '--module', self.__pkcs11lib,
            '--token', self.__current_token_label,
            '--login',
            '--pin',  'env:PKCS11_PIN',
            '--label', label,
            '--id', key_params.id,
            '--write-object', key_params.public_key,
            '--type', 'pubkey'
            ]

        # Run the command
        result = subprocess.run(command, capture_output=True, text=True)
        # Check the result
        if result.returncode != 0:
            self.log.debug("Public key import to HSM failed:")
            self.log.debug(result.stderr)
            return False
        # All good
        return True

    def __generate_rsa_keypair(self, keypair_id: str, label: str) -> bool:
        """
        Generates an RSA keypair via HSM.

        Arguments:
            keypair_id (str) -- Identifier number for key objects.
            label (str) -- Label name for key objects (used with SoftHSM only).

        Returns:
            bool: True if the keypair was generated, False otherwise.
        """
        cka_id = bytes.fromhex(keypair_id)

        public_key_template = [
            (PyKCS11.LowLevel.CKA_CLASS, PyKCS11.LowLevel.CKO_PUBLIC_KEY),
            (PyKCS11.LowLevel.CKA_PRIVATE, PyKCS11.LowLevel.CK_FALSE),
            (PyKCS11.LowLevel.CKA_TOKEN, PyKCS11.CK_TRUE),
            (PyKCS11.LowLevel.CKA_ENCRYPT, PyKCS11.LowLevel.CK_TRUE),
            (PyKCS11.LowLevel.CKA_VERIFY, PyKCS11.LowLevel.CK_TRUE),
            (PyKCS11.LowLevel.CKA_WRAP, PyKCS11.LowLevel.CK_TRUE),
            (PyKCS11.LowLevel.CKA_KEY_TYPE, PyKCS11.LowLevel.CKK_RSA),
            (PyKCS11.LowLevel.CKA_VERIFY_RECOVER, PyKCS11.LowLevel.CK_TRUE),
            (PyKCS11.LowLevel.CKA_MODULUS_BITS, 4096),
            (PyKCS11.LowLevel.CKA_ID, cka_id),
        ]

        private_key_template = [
            (PyKCS11.LowLevel.CKA_CLASS, PyKCS11.LowLevel.CKO_PRIVATE_KEY),
            (PyKCS11.LowLevel.CKA_PRIVATE, PyKCS11.LowLevel.CK_TRUE),
            (PyKCS11.LowLevel.CKA_TOKEN, PyKCS11.LowLevel.CK_TRUE),
            (PyKCS11.LowLevel.CKA_SENSITIVE, PyKCS11.LowLevel.CK_TRUE),
            (PyKCS11.LowLevel.CKA_DECRYPT, PyKCS11.LowLevel.CK_TRUE),
            (PyKCS11.LowLevel.CKA_SIGN, PyKCS11.LowLevel.CK_TRUE),
            (PyKCS11.LowLevel.CKA_UNWRAP, PyKCS11.LowLevel.CK_TRUE),
            (PyKCS11.LowLevel.CKA_ID, cka_id),
        ]

        # Label can be defined freely when using SoftHSM.
        if self.__use_soft_hsm:
            public_key_template.append((PyKCS11.LowLevel.CKA_LABEL, label))
            private_key_template.append((PyKCS11.LowLevel.CKA_LABEL, label))

        try:
            self.__pkcs11_session.generateKeyPair(public_key_template,
                                                  private_key_template,
                                                  PyKCS11.MechanismRSAGENERATEKEYPAIR)
        except PyKCS11Error as e:
            self.log.debug("GenerateKeyPair exception: %s", e)
            return False
        else:
            self.log.debug("Generated RSA keypair")
            return True

    def __generate_ec_keypair(self, keypair_id: str, label: str) -> bool:
        """
        Generates an EC keypair via HSM.

        Arguments:
            keypair_id (str) -- Identifier number for key objects.
            label (str) -- Label name for key objects (used with SoftHSM only).

        Returns:
            bool: True if the keypair was generated, False otherwise.
        """
        cka_id = bytes.fromhex(keypair_id)

        # DER-encoded value for OID 1.2.840.10045.3.1.7 (prime256v1)
        ec_params = bytes.fromhex("06082A8648CE3D030107")

        public_key_template = [
            (PyKCS11.LowLevel.CKA_CLASS, PyKCS11.LowLevel.CKO_PUBLIC_KEY),
            (PyKCS11.LowLevel.CKA_PRIVATE, PyKCS11.LowLevel.CK_FALSE),
            (PyKCS11.LowLevel.CKA_TOKEN, PyKCS11.LowLevel.CK_TRUE),
            (PyKCS11.LowLevel.CKA_ENCRYPT, PyKCS11.LowLevel.CK_TRUE),
            (PyKCS11.LowLevel.CKA_VERIFY, PyKCS11.LowLevel.CK_TRUE),
            (PyKCS11.LowLevel.CKA_WRAP, PyKCS11.LowLevel.CK_TRUE),
            (PyKCS11.LowLevel.CKA_KEY_TYPE, PyKCS11.LowLevel.CKK_EC),
            (PyKCS11.LowLevel.CKA_EC_PARAMS, ec_params),
            (PyKCS11.LowLevel.CKA_ID, cka_id),
        ]

        private_key_template = [
            (PyKCS11.LowLevel.CKA_CLASS, PyKCS11.LowLevel.CKO_PRIVATE_KEY),
            (PyKCS11.LowLevel.CKA_KEY_TYPE, PyKCS11.LowLevel.CKK_EC),
            (PyKCS11.LowLevel.CKA_PRIVATE, PyKCS11.LowLevel.CK_TRUE),
            (PyKCS11.LowLevel.CKA_TOKEN, PyKCS11.LowLevel.CK_TRUE),
            (PyKCS11.LowLevel.CKA_SENSITIVE, PyKCS11.LowLevel.CK_TRUE),
            (PyKCS11.LowLevel.CKA_DECRYPT, PyKCS11.LowLevel.CK_TRUE),
            (PyKCS11.LowLevel.CKA_SIGN, PyKCS11.LowLevel.CK_TRUE),
            (PyKCS11.LowLevel.CKA_UNWRAP, PyKCS11.LowLevel.CK_TRUE),
            (PyKCS11.LowLevel.CKA_ID, cka_id),
        ]

        # Label can be defined freely when using SoftHSM.
        if self.__use_soft_hsm:
            public_key_template.append((PyKCS11.LowLevel.CKA_LABEL, label))
            private_key_template.append((PyKCS11.LowLevel.CKA_LABEL, label))

        try:
            self.__pkcs11_session.generateKeyPair(
                public_key_template,
                private_key_template,
                PyKCS11.MechanismECGENERATEKEYPAIR)

        except PyKCS11Error as e:
            self.log.debug("GenerateKeyPair exception: %s", e)
            return False
        else:
            self.log.debug("Generated EC keypair")
            return True

    def __create_csr_via_openssl(self, csr_params: CsrParams):
        """
        Create a Certificate Signing Request (CSR) using OpenSSL with
        the provided arguments.

        Arguments:
            csr_params (CsrParams) -- CsrParams object holding CSR parameters.

        Returns:
            bool: True if CSR generation is successful, False otherwise.
        """
        # Create directories if they don't exist
        os.makedirs(os.path.dirname(csr_params.filename), exist_ok=True)

        # Openssl requires environment variable usage for pin
        os.environ['PKCS11_PIN'] = self.__token_user_pin
        subject = "/CN=" + csr_params.device_id

        # Generate CSR config file
        self.__generate_csr_config_file(subject, csr_params.is_server)

        command = [
            'openssl',
            'req',
            '-new',
            '-engine', self.__pkcs11_engine,
            '-keyform', 'engine',
            '-key', csr_params.key_id,
            '-passin', 'env:PKCS11_PIN',
            '-out', csr_params.filename,
            '-subj', subject,
            '-config', self.__csr_cnf_file
            ]
        # Run the command
        result = subprocess.run(command, capture_output=True, text=True)

        # Check the result
        if result.returncode == 0:
            return True
        else:
            self.log.debug("Command execution failed.")
            self.log.debug("Error output:")
            self.log.debug(result.stderr)
            return False

    def __get_private_key_handle(self, keypair_id):
        """
        Finds a private key object from the HSM using the provided arguments
        and returns a handle to it.

        Arguments:
            keypair_id (str) -- Identifier number for the private key.

        Returns:
            Handle to private key object or None
        """
        cka_id = bytes.fromhex(keypair_id)

        filter_template = [
            (PyKCS11.LowLevel.CKA_CLASS, PyKCS11.LowLevel.CKO_PRIVATE_KEY),
            (PyKCS11.LowLevel.CKA_ID, cka_id)
        ]

        # Get private key objects
        priv_key_objects = self.__pkcs11_session.findObjects(filter_template)
        if len(priv_key_objects) > 0:
            # There is at least one matching object
            return priv_key_objects[0]
        # Not found
        return None

    def __get_public_key(self, keypair_id):
        """
        Finds a public key object from the HSM using the provided arguments
        and returns a handle to it.

        Arguments:
            keypair_id (str) -- Identifier number for the public key.

        Returns:
            Handle to publick key object or None
        """
        cka_id = bytes.fromhex(keypair_id)

        filter_template = [
            (PyKCS11.LowLevel.CKA_CLASS, PyKCS11.LowLevel.CKO_PUBLIC_KEY),
            (PyKCS11.LowLevel.CKA_ID, cka_id)
        ]

        # Get public key objects
        pub_key_objects = self.__pkcs11_session.findObjects(filter_template)
        # self.log.debug(pub_key_objects)
        if len(pub_key_objects) > 0:
            # There is at least one matching object
            return pub_key_objects[0]
        # Not found
        return None

    def __get_public_key_data(self, object_handle):

        attributes = [
            PyKCS11.LowLevel.CKA_KEY_TYPE,
            # For RSA keys
            PyKCS11.LowLevel.CKA_PUBLIC_EXPONENT,
            PyKCS11.LowLevel.CKA_MODULUS,
            # For EC keys
            PyKCS11.LowLevel.CKA_EC_POINT,
            PyKCS11.LowLevel.CKA_EC_PARAMS,
        ]

        # Get the attributes of the private key
        attrs = self.__pkcs11_session.getAttributeValue(object_handle, attributes)

        # Extract the attribute values
        key_type = attrs[0]
        public_exponent = attrs[1]
        modulus = attrs[2]
        ec_point = attrs[3]
        ec_params = attrs[4]

        if key_type == PyKCS11.LowLevel.CKK_RSA:
            e = int.from_bytes(public_exponent, byteorder='big')
            n = int.from_bytes(modulus, byteorder='big')
            public_key = keys.RSAPublicKey({'modulus': n, 'public_exponent': e}).dump()
            parameters = core.Null
        elif key_type == PyKCS11.LowLevel.CKK_EC:
            public_key = bytes(core.OctetString.load(bytes(ec_point)))
            parameters = keys.ECDomainParameters.load(bytes(ec_params))
        else:
            return None

        return public_key, parameters, key_type

    def __create_csr_info(self, subject, public_key, algorithm_name, pub_key_params, is_server):
        """
        Creates the csr info to be used in certificate request.

        Arguments:
            subject (str) -- Identifies to be used as common name identifier.
            public_key (any) -- Public key object instance
            algorithm_name  -- Algorithm name to be used in CSR
            pub_key_params -- Public key paramters
            is_server (bool) -- Boolean value to define is the CSR for server or
                                client usage.
        Returns:
            Handle to publick key object or None

        """
        x509_subject = x509.Name.build({"common_name": subject})

        if is_server:
            key_usage = "server_auth"
        else:
            key_usage = "client_auth"

        extensions = [("basic_constraints",
                       x509.BasicConstraints({"ca": False}),
                       False),
                      ("key_usage",
                       x509.KeyUsage({"digital_signature",
                                      "key_encipherment"}),
                       True),
                      ("extended_key_usage",
                       x509.ExtKeyUsageSyntax([key_usage]),
                       False)]

        if is_server:
            ip_list = self.__get_ip_addresses()
            names = x509.GeneralNames()
            for ip in ip_list:
                names.append(x509.GeneralName("ip_address", ip))
            extensions.append(("subject_alt_name", names, False))

        csr_info = csr.CertificationRequestInfo({
            "version": "v1",
            "subject": x509_subject,
            "subject_pk_info": {
                "algorithm": {
                    "algorithm": algorithm_name,
                    "parameters": pub_key_params,
                },
                "public_key": public_key
            },
            "attributes": [{
                "type": "extension_request",
                "values": [[self.__create_extension(x) for x in extensions]]
            }]
        })
        return csr_info

    def __create_extension(self, extension):
        """
        Creates an ASN.1 certificate request extension structure

        Arguments:
            extension (tuple): -- Tuple with three values: name of the
                                  extension (str), value for the extension(str)
                                  and boolean to express if extension should be
                                  considered critical.
        Returns:
            Extension dictionary.
        """
        name, value, critical = extension
        return {"extn_id": name,
                "extn_value": value,
                "critical": critical}

    def __get_ip_addresses(self):
        """
        Creates a list of IP addresses from device network interfaces.

        Arguments:
            server (bool) -- Boolean value to determine is CSR meant
                             for server or client usage.
        Returns:
            List of IP addresses
        """
        ip_address_set = set(self.__ip_addresses)

        interfaces = ni.interfaces()
        for interface in interfaces:
            addrs = ni.ifaddresses(interface)

            if ni.AF_INET in addrs:
                ipv4_addresses = [addr['addr'] for addr in addrs[ni.AF_INET]]
                for ipv4_addr in ipv4_addresses:
                    if ipv4_addr not in ip_address_set:
                        self.__ip_addresses.append(ipv4_addr)

            if ni.AF_INET6 in addrs:
                ipv6_addresses = [addr['addr'].split('%')[0] for addr in addrs[ni.AF_INET6]]
                for ipv6_addr in ipv6_addresses:
                    if ipv6_addr not in ip_address_set:
                        self.__ip_addresses.append(ipv6_addr)

        # Remove any Docker interfaces
        self.__ip_addresses = [ip for ip in self.__ip_addresses if not ip.startswith("docker")]

        return self.__ip_addresses

    def __generate_csr_config_file(self, subject: str, server=False):
        """
        Creates a configurarion file for CSR creation via openssl

        Arguments:
            server (bool) -- Boolean value to determine is CSR meant
                             for server or client usage.
        Returns:
            bool: True if the config file was generated, False otherwise.
        """
        try:
            with open(self.__csr_cnf_file, 'w') as file:
                file.write("[ req ]\n")
                file.write("distinguished_name=req_distinguished_name\n")
                file.write("req_extensions = v3_req\n")
                file.write("\n[ req_distinguished_name ]\n")
                file.write(subject+"\n")
                file.write("\n[ v3_req ]\n")
                file.write("basicConstraints = CA:FALSE\n")
                file.write("keyUsage=digitalSignature, keyEncipherment\n")
                if not server:
                    file.write("extendedKeyUsage = clientAuth\n")
                else:
                    ip_addresses = self.__get_ip_addresses()
                    file.write("extendedKeyUsage = serverAuth\n")
                    file.write("subjectAltName = @alt_names\n")
                    file.write("\n[alt_names]\n")
                    for i, ip_address in enumerate(ip_addresses, start=1):
                        file.write(f"IP.{i} = {ip_address}\n")
            return True
        except Exception as e:
            self.log.debug("Error creating CSR config file: %s", e)
            return False

    def create_csr(self, csr_params: CsrParams):
        """
        Create a Certificate Signing Request (CSR) either via openssl or
        asn1crypto library depending on given parameters.

        Arguments:
            csr_params (CsrParams) -- CsrParams object holding CSR parameters.

        Returns:
            bool: True if CSR generation is successful, False otherwise.
        """
        if csr_params.use_openssl:
            retval = self.__create_csr_via_openssl(csr_params)
        else:
            retval = self.__create_csr(csr_params)
        return retval

    def __create_csr(self, csr_params: CsrParams):
        """
        Create a Certificate Signing Request (CSR) using asn1crypto library.
        CSR is signed via HSM.

        Arguments:
            csr_params (CsrParams) -- CsrParams object holding CSR parameters.
        Returns:
            bool: True if CSR generation is successful, False otherwise.
        """
        # Create directories if they don't exist
        os.makedirs(os.path.dirname(csr_params.filename), exist_ok=True)

        # Get the private key handle for signing CSR
        private_key_handle = self.__get_private_key_handle(csr_params.key_id)
        public_key_handle = self.__get_public_key(csr_params.key_id)

        if private_key_handle is None:
            self.log.debug("Private key not found")
            return False

        if public_key_handle is None:
            return False

        public_key, pub_key_params, key_type = self.__get_public_key_data(public_key_handle)

        if key_type == PyKCS11.LowLevel.CKK_RSA:
            public_key = keys.RSAPublicKey.load(public_key)
            mechanism = Mechanism(PyKCS11.LowLevel.CKM_SHA256_RSA_PKCS, None)
            algorithm_name = "rsa"
            sign_algorithm_name = "sha256_rsa"
        else:  # key_type == PyKCS11.LowLevel.CKK_EC
            mechanism = Mechanism(PyKCS11.LowLevel.CKM_ECDSA, None)
            algorithm_name = "ec"
            sign_algorithm_name = "ecdsa"

        csr_info = self.__create_csr_info(csr_params.device_id, public_key, algorithm_name,
                                          pub_key_params, csr_params.is_server)

        # Sign the CSR Info
        signature = self.__pkcs11_session.sign(private_key_handle,
                                               csr_info.dump(),
                                               mechanism)

        # Combine CSR info with the signature
        signed_csr = csr.CertificationRequest({
            'certification_request_info': csr_info,  # Unsigned CSR data
            'signature_algorithm': {
                'algorithm': sign_algorithm_name,
                'parameters': None,
            },
            'signature': bytes(signature),  # The actual signature
        })
        print(csr_params.filename)
        # Write CSR in PEM format to a file
        try:
            pem_csr = pem.armor("CERTIFICATE REQUEST", signed_csr.dump())
            with open(csr_params.filename, 'wb') as csr_file:
                csr_file.write(pem_csr)
        except Exception as e:
            self.log.debug("Error %s when writing CSR file", e)
            return False

        return True
