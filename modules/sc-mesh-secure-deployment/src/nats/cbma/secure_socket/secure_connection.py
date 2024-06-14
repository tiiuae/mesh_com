from __future__ import annotations

import socket
import errno

from typing import Any, Callable, Tuple
from OpenSSL import SSL

from models.secure_socket.secure_connection import ISecureConnection
from utils.logging import get_logger


logger = get_logger()


class SecureConnection(ISecureConnection):
    pass


# TODO - Move to models
MACsecCallbackType = Callable[[SecureConnection], bool]
SecureConnectionCallbackType = Callable[[MACsecCallbackType], bool]


class FileBasedSecureConnection(SSL.Connection, SecureConnection):
    def __init__(self, context: SSL.Context, sock: socket.socket) -> None:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
        SSL.Connection.__init__(self, context, sock)
        SecureConnection.__init__(self)


    def get_sock_name(self) -> Any:
        return self._socket.getsockname()


    def get_peer_name(self) -> Any:
        return self._socket.getpeername()


    def accept(self) -> Tuple[FileBasedSecureConnection, Any]:
        client_connection, client_address = self._socket.accept()
        conn = FileBasedSecureConnection(self._context, client_connection)
        conn.set_accept_state()

        return (conn, client_address)


    def shutdown(self, _how: int) -> bool:
        try:
            self.sock_shutdown(_how)
        except socket.error as e:
            if e.errno != errno.ENOTCONN and e.errno != errno.EBADF:
                logger.error(f"Closing the connection error: {e}")
                return False
        return True


    def close(self) -> bool:
        success = self.shutdown(socket.SHUT_RDWR)
        try:
            self._socket.close()
        except IOError as e:
            if e.errno != errno.EBADF:
                logger.error(f"Closing the socket error: {e}")
                success = False
        except Exception as e:
            logger.error(f"Exception when closing the socket: {e}")
            success = False
        return success
