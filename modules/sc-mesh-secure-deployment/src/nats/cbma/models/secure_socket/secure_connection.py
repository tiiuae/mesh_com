from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional, Union, Tuple, Any


class ISecureConnection(ABC):
    @abstractmethod
    def send(self, buf: Union[bytes, str], flags: int = 0) -> int:
        raise NotImplementedError('send not implemented')


    @abstractmethod
    def sendall(self, buf: Union[bytes, str], flags: int = 0) -> int:
        raise NotImplementedError('sendall not implemented')


    @abstractmethod
    def recv(self, bufsize: int, flags: Optional[int] = None) -> bytes:
        raise NotImplementedError('recv not implemented')


    @abstractmethod
    def get_sock_name(self) -> Any:
        raise NotImplementedError('get_sock_name not implemented')


    @abstractmethod
    def get_peer_name(self) -> Any:
        raise NotImplementedError('get_peer_name not implemented')


    @abstractmethod
    def accept(self) -> Tuple[ISecureConnection, Any]:
        raise NotImplementedError('accept not implemented')


    @abstractmethod
    def shutdown(self, _how: int) -> bool:
        raise NotImplementedError('shutdown not implemented')


    @abstractmethod
    def close(self) -> bool:
        raise NotImplementedError('close not implemented')
