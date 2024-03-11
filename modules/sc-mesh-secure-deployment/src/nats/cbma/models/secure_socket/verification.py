from abc import ABC, abstractmethod
from enum import IntEnum

from OpenSSL import SSL, crypto


# https://github.com/openssl/openssl/blob/master/include/openssl/x509_vfy.h.in
class X509VerificationCodes(IntEnum):
    X509_V_ERR_SELF_SIGNED_CERT_IN_CHAIN = 19


class CertificateVerificationError(Exception):
  pass


class TLSVerificationCallbackInterface(ABC):
    @abstractmethod
    def verify(self, conn: SSL.Connection, cert: crypto.X509, errnum: int, depth: int, ok: int) -> bool:
        raise NotImplementedError("Verify not implemented")


