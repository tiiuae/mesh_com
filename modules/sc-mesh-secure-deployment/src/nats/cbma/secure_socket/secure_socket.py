from secure_socket.secure_context import FileBasedSecureContext
from secure_socket.verification import TLSVerification
from secure_socket.secure_connection import FileBasedSecureConnection
from models.certificates import CBMACertificates


class FileBasedSecureSocket(FileBasedSecureContext, TLSVerification):
    def __init__(self, certificates: CBMACertificates, tls_method: int) -> None:
        FileBasedSecureContext.__init__(self, certificates, tls_method, self.verify)
        TLSVerification.__init__(self, certificates.chain)


    def _close(self) -> None:
        if (sock_conn_obj := self.__dict__.get("sock_conn_obj")) and \
           isinstance(sock_conn_obj, FileBasedSecureConnection):
            sock_conn_obj.close()
