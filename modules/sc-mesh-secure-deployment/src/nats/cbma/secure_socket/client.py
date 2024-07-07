import time
import random
import socket

from typing import Union
from operator import itemgetter

from OpenSSL import SSL

from models.certificates import CBMACertificates
from secure_socket.secure_socket import FileBasedSecureSocket
from secure_socket.secure_connection import FileBasedSecureConnection, MACsecCallbackType
from secure_socket.verification import CertificateVerificationError
from utils.logging import get_logger


logger = get_logger()


class FileBasedSecureSocketClient(FileBasedSecureSocket):
    TIMEOUT_SECONDS: int = 4
    DEFAULT_CONNECT_PARAMS: dict[str, Union[int, float]] = {
        'MAX_RETRIES': 5,
        'MIN_WAIT_TIME_SECONDS': 0.2,
        'MAX_WAIT_TIME_SECONDS': 1.0
    }
    HANDSHAKE_PARAMS: dict[str, Union[int, float]] = {
        'MAX_RETRIES': 5,
        'WAIT_TIME_SECONDS': 0.5,
    }

    def __init__(self,
                 interface: str,
                 server_ipv6: str,
                 server_port: int,
                 certificates: CBMACertificates,
                 macsec_callback: MACsecCallbackType
                 ) -> None:
        self.interface = interface
        self.server_ipv6 = server_ipv6
        self.server_port = server_port
        self.macsec_callback = macsec_callback

        super().__init__(certificates, SSL.SSLv23_METHOD)


    def __create_socket_connection_object(self) -> FileBasedSecureConnection:
        ctx = self.create_ssl_context()

        sock_conn_obj = FileBasedSecureConnection(ctx,
                                                  socket.socket(socket.AF_INET6, socket.SOCK_STREAM))
        sock_conn_obj._socket.settimeout(self.TIMEOUT_SECONDS)
        sock_conn_obj.set_connect_state()

        return sock_conn_obj


    def __do_handshake(self) -> bool:
        """
        Performs handshake with server.
        We need to try the handshake a few times before it is done with server.

        Returns True if handshake was successful, False otherwise
        """
        max_retries_handshake, wait_time_handshake = itemgetter('MAX_RETRIES', 'WAIT_TIME_SECONDS')(FileBasedSecureSocketClient.HANDSHAKE_PARAMS)

        retries_handshake = 0
        while retries_handshake < max_retries_handshake:
            retries_handshake += 1
            try:
                logger.debug(f"Attempt {retries_handshake}/{max_retries_handshake} - Performing handshake with {self.server_ipv6}")
                self.sock_conn_obj.do_handshake()
                logger.debug(f"Handshake with {self.server_ipv6} successful after {retries_handshake} attempts")
                return True
            except SSL.WantReadError:
                # this is normal error which happens when handshake initiation with server did not succeed,
                # this is why we can retry handshake a few times
                time.sleep(wait_time_handshake)
                continue
            except socket.timeout:
                raise socket.timeout()
            except Exception as err:
                err = self.catch_certificate_verification_failure(err)
                logger.error(f"Exception when performing handshake with {self.server_ipv6}: {err}")
            break

        logger.error(f"Handshake with {self.server_ipv6} failed after {retries_handshake} attempts")
        return False


    def connect(self, retry_params: dict[str, Union[int, float]] = DEFAULT_CONNECT_PARAMS) -> bool:
        """
        Returns True if connection was established, otherwise False.
        """
        max_retries, min_wait_time, max_wait_time = itemgetter('MAX_RETRIES', 'MIN_WAIT_TIME_SECONDS', 'MAX_WAIT_TIME_SECONDS')(retry_params)
        conn_params = (self.server_ipv6, self.server_port, 0, socket.if_nametoindex(self.interface))

        retries = 0
        while retries < max_retries:
            retries += 1
            self._close()
            try:
                self.sock_conn_obj = self.__create_socket_connection_object()
                logger.debug(f"Attempt {retries}/{max_retries} - Connecting with {self.server_ipv6}")
                self.sock_conn_obj.connect(conn_params)
                if not self.__do_handshake():
                    break
                return self.macsec_callback(self.sock_conn_obj)
            except socket.timeout:
                logger.error('Socket timed out')
                break
            except (socket.error, CertificateVerificationError) as e:
                if isinstance(e, CertificateVerificationError):
                    logger.error(e)
                else:
                    logger.error(f"Socket error: {e}")
                wait_time = random.uniform(min_wait_time, max_wait_time)
                logger.warning(f"Attempt {retries}/{max_retries} - Retrying connection with {self.server_ipv6} in {wait_time:.2f} seconds")
                time.sleep(wait_time)
            except Exception as e:
                logger.error(f"Unexpected exception: {e}", exc_info=True)
                break
        else:
            logger.error(f"Exceeded maximum retry attempts - Unable to connect to {self.server_ipv6}")
        self._close()

        return False
