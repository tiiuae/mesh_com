import os

from typing import Optional
from OpenSSL import SSL, crypto

from models.secure_socket.secure_context import SecureContextInterface, OpenSSLCallback
from models.certificates import CBMACertificates
from utils.logging import get_logger


logger = get_logger()


class FileBasedSecureContext(SecureContextInterface):
    CTX_OPTIONS: int = SSL.OP_NO_SSLv2 | SSL.OP_NO_SSLv3 | SSL.OP_NO_TLSv1 | \
                       SSL.OP_NO_TLSv1_1
    SSL_SESSION_TIMEOUT: int = 60  # seconds

    def __init__(self,
                 certificates: CBMACertificates,
                 tls_method: int,
                 verify_callback: Optional[OpenSSLCallback] = None
                 ) -> None:
        '''
        Parameters:
            certificates (Certificates): Paths to certificate files
            tls_method (int): SSL.TLS_CLIENT_METHOD or SSL.TLS_SERVER_METHOD
            verify_callback (Union[OpenSSLCallback, None]): Custom callback to call on connection verification
        '''
        self.ssl_ctx = self.__create_ssl_context(certificates, tls_method, verify_callback)


    def create_ssl_context(self) -> SSL.Context:
        return self.ssl_ctx


    def __create_ssl_context(self,
                             certificates: CBMACertificates,
                             tls_method: int,
                             verify_callback: Optional[OpenSSLCallback] = None
                             ) -> SSL.Context:
        '''
        Returns:
            SSL.Context
        '''
        ctx = SSL.Context(tls_method)

        # Force usage of TLSv3 by disabling all other protocols
        ctx.set_options(self.CTX_OPTIONS)
        ctx.set_verify(SSL.VERIFY_PEER|SSL.VERIFY_FAIL_IF_NO_PEER_CERT, verify_callback)
        ctx.set_session_cache_mode(SSL.SESS_CACHE_OFF)
        ctx.set_timeout(self.SSL_SESSION_TIMEOUT)

        # NOTE: The order of loading the chain, cert and key matter to avoid errors
        ctx.use_certificate_chain_file(certificates.chain[0])

        if len(certificates.chain) > 1:
            for cert in certificates.chain[1:]:
                cert_bytes = open(cert, 'rb').read()
                x509_cert = crypto.load_certificate(crypto.FILETYPE_PEM, cert_bytes)
                ctx.add_extra_chain_cert(x509_cert)

        if certificates.ca:
            # TODO - What do add_client_ca() and load_client_ca() are for?
            if os.path.isdir(certificates.ca):
                ctx.load_verify_locations(cafile=None, capath=certificates.ca)
            else:
                ctx.load_verify_locations(cafile=certificates.ca)

        ctx.use_certificate_file(certificates.cert)
        ctx.use_privatekey_file(certificates.key)
        ctx.check_privatekey()

        return ctx

