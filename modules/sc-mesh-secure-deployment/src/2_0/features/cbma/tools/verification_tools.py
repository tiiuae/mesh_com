from datetime import datetime
from OpenSSL import crypto
from cryptography import x509
from cryptography.hazmat.backends import default_backend
from .utils import get_mac_from_ipv6
import re
import ssl
import os


# Custom exceptions for certificate verification
class CertificateExpiredError(Exception):
    pass


class CertificateActivationError(Exception):
    pass


class CertificateHostnameError(Exception):
    pass


class CertificateIssuerError(Exception):
    pass


class CertificateVerificationError(Exception):
    pass


class CertificateNoPresentError(Exception):
    pass


class CertificateDifferentCN(Exception):
    pass


class ServerConnectionRefusedError(Exception):
    pass


def verify_cert(cert, ca_cert, IPaddress, interface, logging):
    try:
        return validation(cert, ca_cert, IPaddress, interface, logging)
    except (CertificateExpiredError, CertificateHostnameError, CertificateIssuerError, ValueError) as e:
        logging.error(f"Certificate verification failed with {IPaddress}.", exc_info=True)
        return False
    except Exception as e:
        logging.error(f"An unexpected error occurred during certificate verification with {IPaddress}.", exc_info=True)
        return False


def validation(cert, ca_cert, IPaddress, interface, logging):
    # Load the DER certificate into an OpenSSL certificate object
    _x509 = crypto.load_certificate(crypto.FILETYPE_ASN1, cert)

    # Get the certificate expiration date and activation date using OpenSSL methods
    expiration_date_str = _x509.get_notAfter().decode('utf-8')
    activation_date_str = _x509.get_notBefore().decode('utf-8')

    expiration_date = datetime.strptime(expiration_date_str, '%Y%m%d%H%M%SZ')
    activation_date = datetime.strptime(activation_date_str, '%Y%m%d%H%M%SZ')
    current_date = datetime.now()

    # Check if the certificate has expired
    if expiration_date < current_date:
        logging.error(f"Certificate of {IPaddress} has expired.", exc_info=True)
        raise CertificateExpiredError("Certificate has expired.")

    if activation_date > current_date:
        logging.error(f"Client certificate not yet active for {IPaddress}.", exc_info=True)
        raise CertificateExpiredError("Client certificate not yet active")

    _cert = x509.load_der_x509_certificate(cert, default_backend())

    # Extract the MAC from SAN otherName field
    san_extension = _cert.extensions.get_extension_for_class(
        x509.SubjectAlternativeName
    )
    oid = x509.ObjectIdentifier("1.3.6.1.1.1.1.22")
    othername_values = [
        general_name.value
        for general_name in san_extension.value
        if isinstance(general_name, x509.OtherName) and general_name.type_id == oid
    ]
    mac_from_san = None
    for value in othername_values:
        try:
            if isinstance(value, bytes):
                if mac_from_san:
                    logging.error(f"Multiple otherName values found in the SAN, continuing with the first one")
                else:
                    mac_from_san = value[2:].decode('ascii')  # Skip the first 2 bytes
        except Exception as e:
            logging.error(f"Error processing OtherName value: {e}")

    if mac_from_san != get_mac_from_ipv6(IPaddress, interface):
        logging.error(f"MAC in SAN otherName doesn't match MAC Address for {IPaddress}", exc_info=True)
        raise CertificateDifferentCN("MAC from SAN otherName does not match the interface MAC Address.")

    if _verify_certificate_chain(cert, ca_cert, logging):
        # If the client certificate has passed all verifications, you can print or log a success message
        logging.info(f"Certificate verification successful for {IPaddress}.")
        return True

    raise CertificateVerificationError("Verification of certificate chain failed.")


def _verify_certificate_chain(cert, trusted_certs, logging):
    try:
        x509 = crypto.load_certificate(crypto.FILETYPE_ASN1, cert)
        store = crypto.X509Store()

        # Check if trusted_certs is a list, if not make it a list
        if not isinstance(trusted_certs, list):
            trusted_certs = [trusted_certs]

        _PEM_RE = re.compile(
            b'-----BEGIN CERTIFICATE-----\r?.+?\r?-----END CERTIFICATE-----\r?\n?', re.DOTALL
        )

        for _cert in trusted_certs:
            with open(_cert, 'rb') as f:
                for c in _PEM_RE.finditer(f.read()):
                    cert_data = crypto.load_certificate(crypto.FILETYPE_PEM, c.group())
                    store.add_cert(cert_data)

        store_ctx = crypto.X509StoreContext(store, x509)
        store_ctx.verify_certificate()

        return True
    except Exception as e:
        logging.error(f"Certificate chain verification failed: {e}", exc_info=True)
        return False

def store_peer_certificate(peer_cert, peer_mac, logger):
    """
    Store peer certificate to a file
    """
    try:
        # Convert DER to PEM
        peer_cert_pem = ssl.DER_cert_to_PEM_cert(peer_cert)

        # Write the PEM certificate to a file
        directory = '/tmp/peer_certificates/'
        filename = f"{peer_mac}.pem"
        file_path = os.path.join(directory, filename)

        # Create the directory if it doesn't exist
        if not os.path.exists(directory):
            os.makedirs(directory)

        # Write the PEM certificate to the file
        with open(file_path, 'w') as cert_file:
            cert_file.write(peer_cert_pem)

        logger.info(f"Stored peer certificate for {peer_mac}")
    except Exception as e:
        logger.info(f"Error storing peer certificate for {peer_mac}: {e}")
