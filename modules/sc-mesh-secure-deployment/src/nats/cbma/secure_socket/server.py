import socket
import errno

from OpenSSL import SSL

from models.certificates import CBMACertificates
from secure_socket.secure_socket import FileBasedSecureSocket
from secure_socket.secure_connection import FileBasedSecureConnection, MACsecCallbackType
from secure_socket.verification import CertificateVerificationError
from utils.logging import get_logger


logger = get_logger()


class FileBasedSecureSocketServer(FileBasedSecureSocket):
    # this is meant to run as single threaded, so max connections is 1
    MAX_CONNECTIONS_TO_LISTEN = 1
    TIMEOUT_SECONDS = 15

    def __init__(self,
                 interface: str,
                 port: int,
                 certificates: CBMACertificates,
                 macsec_callback: MACsecCallbackType
                 ) -> None:
        self.interface = interface
        self.port = port
        self.macsec_callback = macsec_callback

        super().__init__(certificates, SSL.TLSv1_2_METHOD)


    def __create_socket_connection_object(self) -> FileBasedSecureConnection:
        ctx = self.create_ssl_context()

        sock_conn_obj = FileBasedSecureConnection(ctx,
                                                  socket.socket(socket.AF_INET6, socket.SOCK_STREAM))
        sock_conn_obj._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock_conn_obj._socket.settimeout(self.TIMEOUT_SECONDS)
        sock_conn_obj._socket.setsockopt(socket.SOL_SOCKET,
                                         socket.SO_BINDTODEVICE,
                                         str(self.interface + '\0').encode('utf-8'))

        index = socket.if_nametoindex(self.interface)

        sock_conn_obj.bind(("::", self.port, 0, index))
        sock_conn_obj.set_accept_state()

        return sock_conn_obj


    def handle_client(self, client_connection: FileBasedSecureConnection, client_address: str) -> bool:
        try:
            logger.debug(f"Starting handshake with {client_address}")
            client_connection.do_handshake()
        except SSL.Error as err:
            err = self.catch_certificate_verification_failure(err)
            logger.error(f"Handshake with {client_address} failed: {err}")
            return False

        logger.debug(f"Handshake with {client_address} successful")
        return True


    def listen(self) -> bool:
        client_handled: bool = False
        try:
            self.sock_conn_obj = self.__create_socket_connection_object()
            self.sock_conn_obj.listen(self.MAX_CONNECTIONS_TO_LISTEN)
            logger.debug("Server listening")

            client_connection, client_address = self.sock_conn_obj.accept()
            if client_handled := self.handle_client(client_connection, client_address):
                return self.macsec_callback(client_connection)
        except socket.timeout:
            logger.error("Socket timed out")
        except CertificateVerificationError as e:
            logger.error(e)
        except socket.error as e:
            if e.errno == errno.EBADF:
                logger.debug("Client closed the connection")
            else:
                logger.error(f"Socket error: {e}")
        except Exception as e:
            logger.error(f"Unexpected exception: {e}", exc_info=True)
        finally:
            self._close()

        logger.info("Exiting")
        return client_handled
