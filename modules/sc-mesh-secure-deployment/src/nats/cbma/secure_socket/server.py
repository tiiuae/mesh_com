import socket
import errno

from typing import Any

from OpenSSL import SSL

from models.certificates import CBMACertificates
from secure_socket.secure_socket import FileBasedSecureSocket
from secure_socket.secure_connection import FileBasedSecureConnection, MACsecCallbackType
from secure_socket.verification import CertificateVerificationError
from utils.networking import get_interface_link_local_ipv6_address
from utils.logging import get_logger


logger = get_logger()


class FileBasedSecureSocketServer(FileBasedSecureSocket):
    MAX_CONNECTIONS_TO_LISTEN: int = 10
    TIMEOUT_SECONDS: int = 4

    def __init__(self,
                 interface: str,
                 client_ipv6: str,
                 port: int,
                 certificates: CBMACertificates,
                 macsec_callback: MACsecCallbackType
                 ) -> None:
        self.interface = interface
        self.port = port
        self.client_ipv6 = client_ipv6
        self.macsec_callback = macsec_callback

        super().__init__(certificates, SSL.SSLv23_METHOD)


    def __create_socket_connection_object(self) -> FileBasedSecureConnection:
        ctx = self.create_ssl_context()

        sock_conn_obj = FileBasedSecureConnection(ctx,
                                                  socket.socket(socket.AF_INET6, socket.SOCK_STREAM))
        sock_conn_obj._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock_conn_obj._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        sock_conn_obj._socket.settimeout(self.TIMEOUT_SECONDS)

        ipv6 = get_interface_link_local_ipv6_address(self.interface)
        index = socket.if_nametoindex(self.interface)

        sock_conn_obj.bind((ipv6, self.port, 0, index))

        return sock_conn_obj


    def handle_client(self, client_connection: FileBasedSecureConnection, client_address: Any) -> bool:
        client_ipv6 = client_address[0]
        try:
            logger.debug(f"Starting handshake with {client_ipv6}")
            client_connection.do_handshake()
        except SSL.Error as err:
            err = self.catch_certificate_verification_failure(err)
            logger.error(f"Handshake with {client_ipv6} failed: {err}")
            return False

        logger.debug(f"Handshake with {client_ipv6} successful")
        return True


    def listen(self) -> bool:
        client_handled: bool = False
        client_connection = None
        try:
            self.sock_conn_obj = self.__create_socket_connection_object()
            self.sock_conn_obj.listen(self.MAX_CONNECTIONS_TO_LISTEN)
            logger.debug('Server listening')

            while True:
                client_connection, client_address = self.sock_conn_obj.accept()
                if self.client_ipv6 == client_address[0]:
                    break
                client_connection.close()
            self._close()

            if self.handle_client(client_connection, client_address):
                if client_handled := self.macsec_callback(client_connection):
                    client_connection._socket.close()
                    client_connection = None
        except socket.timeout:
            logger.error('Socket timed out')
        except CertificateVerificationError as e:
            logger.error(e)
        except socket.error as e:
            if e.errno == errno.EBADF:
                logger.debug('Client closed the connection')
            else:
                logger.error(f"Socket error: {e}")
        except Exception as e:
            logger.error(f"Unexpected exception: {e}", exc_info=True)
        finally:
            if client_connection:
                client_connection.close()
            self._close()

        logger.debug('Exiting')
        return client_handled
