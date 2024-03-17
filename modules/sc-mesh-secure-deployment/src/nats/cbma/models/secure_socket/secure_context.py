from abc import ABC, abstractmethod
from typing import Callable

from OpenSSL import SSL, crypto


OpenSSLCallback = Callable[[SSL.Connection, crypto.X509, int, int, int], bool]


class SecureContextInterface(ABC):
    @abstractmethod
    def create_ssl_context(self) -> SSL.Context:
        raise NotImplementedError('create_ssl_context not implemented')

