import os
import sys

from OpenSSL import SSL

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(__file__), os.pardir)))

from secure_socket.secure_socket import FileBasedSecureSocket
from models.certificates import CBMACertificates


TEST_DATA_FOLDER_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_data/certificates")


class TestCreateSSLContext(object):
    def test_certificate_loading(self):
        certfile = os.path.join(TEST_DATA_FOLDER_PATH, "csl1.local.crt")
        keyfile = os.path.join(TEST_DATA_FOLDER_PATH, "csl1.local.key")
        chain_file = os.path.join(TEST_DATA_FOLDER_PATH, "chain.crt")
        cert_paths = CBMACertificates(certfile, keyfile, [chain_file])

        class TestSecureSocket(FileBasedSecureSocket):
            def __init__(self, cert_paths: CBMACertificates, tls_method: int) -> None:
                super().__init__(cert_paths, tls_method)

            def verify(self, conn, cert, errnum, depth, ok) -> bool:
                return True

        secure_socket = TestSecureSocket(cert_paths, SSL.SSLv23_METHOD)
        try:
            secure_socket.create_ssl_context()
            assert True
        except SSL.Error:
            assert False
