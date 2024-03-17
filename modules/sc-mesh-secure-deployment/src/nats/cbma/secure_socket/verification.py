import re

from datetime import datetime
from OpenSSL import SSL, crypto

from models.secure_socket.verification import ( TLSVerificationCallbackInterface,
                                                X509VerificationCodes,
                                                CertificateVerificationError )
from utils.networking import get_mac_from_ipv6
from utils.certificates import get_mac_from_san
from utils.logging import get_logger


logger = get_logger()


class TLSVerification(TLSVerificationCallbackInterface):
    _PEM_RE = re.compile(
        b'-----BEGIN CERTIFICATE-----\r?.+?\r?-----END CERTIFICATE-----\r?\n?', re.DOTALL
    )

    def __init__(self, chain: list[str]) -> None:
        self.x509_store = crypto.X509Store()

        for cert in chain:
            cert_bytes = open(cert, 'rb').read()
            for c in TLSVerification._PEM_RE.finditer(cert_bytes):
                cert_data = crypto.load_certificate(crypto.FILETYPE_PEM, c.group())
                self.x509_store.add_cert(cert_data)


    def verify(self, conn: SSL.Connection, cert: crypto.X509, errnum: int, depth: int, ok: int) -> bool:
        """
        In practise this callback is called three times:
        the first two times it is called with the @errnum being not zero.
        In the third time it is 0.
        Thus, when @errnum is != 0, we return True, because otherwise the verification
        will be cancelled and there would not be any subsequent calls.

        The error codes for @errnum are documented here:
        https://github.com/openssl/openssl/blob/master/include/openssl/x509_vfy.h.in (16.2.2024)
        """
        if errnum != 0 and \
           errnum != X509VerificationCodes.X509_V_ERR_SELF_SIGNED_CERT_IN_CHAIN and \
           depth > 0:
            # self-signed certificate is acceptable if it is root CA
            # TODO: Ideally depth > 0 could be replaced with depth == <chain-length>, but do not know if we can get it
            return False

        if errnum != 0 or ok != 1:
            return True

        peer_ipv6 = conn._socket.getpeername()[0]
        return self.verify_peer(peer_ipv6, cert)


    def verify_peer(self, peer_ipv6: str, peer_cert: crypto.X509) -> bool:
        try:
            peer_conn_mac = get_mac_from_ipv6(peer_ipv6)
            peer_cert_mac = get_mac_from_san(peer_cert)
        except Exception as e:
            logger.error(e)
            return False

        if peer_conn_mac.lower() != peer_cert_mac.lower():
            logger.error(f"Peer MAC address doesn't match the certificate's one: {peer_conn_mac} vs {peer_cert_mac}")
            return False
        return self.verify_certificate(peer_cert)


    def verify_certificate(self, cert: crypto.X509) -> bool:
        try:
            return self._validation(cert)
        except Exception as e:
            logger.error(f"An unexpected error occurred during certificate verification: {e}", exc_info=True)
            return False


    def _validation(self, cert_x509: crypto.X509) -> bool:
        expiration_date = cert_x509.get_notAfter()
        activation_date = cert_x509.get_notBefore()

        if expiration_date == None or activation_date == None:
            logger.error("Certificate doesn't contain Not Before or After fields")
            return False

        expiration_date_str = expiration_date.decode('utf-8')
        activation_date_str = activation_date.decode('utf-8')

        expiration_date = datetime.strptime(expiration_date_str, '%Y%m%d%H%M%SZ')
        activation_date = datetime.strptime(activation_date_str, '%Y%m%d%H%M%SZ')
        current_date = datetime.now()

        if expiration_date < current_date:
            logger.error('Certificate has expired')
            return False

        # TODO - Uncomment when device date doesn't revert to Jan 1 1970
        # if activation_date > current_date:
        #     logger.error('Client certificate not yet active')
        #     return False

        return self._verify_certificate_chain(cert_x509)


    def _verify_certificate_chain(self, cert_x509: crypto.X509) -> bool:
        try:
            store_ctx = crypto.X509StoreContext(self.x509_store, cert_x509)
            store_ctx.verify_certificate()
            logger.debug('Certificate verification successful')
            return True
        except Exception as e:
            logger.error(f"Certificate chain verification failed: {e}", exc_info=True)
            return False


    @staticmethod
    def catch_certificate_verification_failure(err: Exception) -> Exception:
        """
        When verify callback fails, it raises generic SSL.Error with data:
        [('SSL routines', '', 'certificate verify failed')]

        This function tries to find that error. If found, it returns CertificateVerificationError,
        which is easier to read.

        @Raises the identified Exception, if found, or the original Exception generic SSL.Error otherwise
        """
        try:
            # using try-except clause to check to make access to lists and tuples, not even trying to
            # do fine-grained type checking
            # err.args is like: ([('SSL routines', '', 'tlsv1 alert unknown ca')],)
            # so we have to access tuple, then array and then tuple
            error_msg = err.args[0][0][2]
            if error_msg.lower() == 'certificate verify failed':
                raise CertificateVerificationError('Certificate verification failed')
            elif error_msg.lower() == 'tlsv1 alert unknown ca':
                raise CertificateVerificationError('Certificate verification failed')
        except CertificateVerificationError as e:
            return e
        except Exception as e:
            # if access to e.args fails, the access itself can give variety of errors,
            # like "string index out of range". In this case we return the original Exception
            return err

        return err
