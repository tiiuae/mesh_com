from datetime import datetime
import OpenSSL
from OpenSSL import crypto
from .utils import extract_mac_from_ipv6


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

def verify_cert(cert, ca_cert, IPaddress,  logging):
    try:
        return validation(cert, ca_cert, IPaddress, logging)
    except (CertificateExpiredError, CertificateHostnameError, CertificateIssuerError, ValueError) as e:
        logging.error(f"Certificate verification failed with {IPaddress}.", exc_info=True)
        return False
    except Exception as e:
        logging.error(f"An unexpected error occurred during certificate verification with {IPaddress}.", exc_info=True)
        return False


def validation(cert, ca_cert, IPaddress, logging):
    # Load the DER certificate into an OpenSSL certificate object
    x509 = OpenSSL.crypto.load_certificate(OpenSSL.crypto.FILETYPE_ASN1, cert)

    # Get the certificate expiration date and activation date using OpenSSL methods
    expiration_date_str = x509.get_notAfter().decode('utf-8')
    activation_date_str = x509.get_notBefore().decode('utf-8')

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

    # Extract the actual ID from CN
    common_name = x509.get_subject().CN
    if common_name != extract_mac_from_ipv6(IPaddress):
        logging.error(f"CN does not match the MAC Address for {IPaddress}", exc_info=True)
        raise CertificateDifferentCN("CN does not match the MAC Address.")

    if _verify_certificate_chain(cert, ca_cert, logging):
        # Extract the public key from the certificate
        pub_key_der = OpenSSL.crypto.dump_publickey(OpenSSL.crypto.FILETYPE_ASN1, x509.get_pubkey())
        # If the client certificate has passed all verifications, you can print or log a success message
        logging.info(f"Certificate verification successful for {IPaddress}.")
        return True

    else:
        raise CertificateVerificationError("Verification of certificate chain failed.")


def _verify_certificate_chain(cert, trusted_certs, logging):
    """
    this function is not being used right now, because we only have one certificate (ca.crt), not a full chain (CA, Interm CA, etc)
    but I left the code for further adaptation
    """
    try:
        x509 = crypto.load_certificate(crypto.FILETYPE_ASN1, cert)
        cert_pem = crypto.dump_certificate(crypto.FILETYPE_PEM, x509)

        store = crypto.X509Store()

        # Check if trusted_certs is a list, if not make it a list
        if not isinstance(trusted_certs, list):
            trusted_certs = [trusted_certs]

        for _cert in trusted_certs:
            cert_data = crypto.load_certificate(crypto.FILETYPE_PEM, open(_cert).read())
            store.add_cert(cert_data)

        store_ctx = crypto.X509StoreContext(store, x509)
        store_ctx.verify_certificate()

        return True
    except Exception as e:
        logging.error(f"Certificate chain verification failed: {e}", exc_info=True)
        return False
