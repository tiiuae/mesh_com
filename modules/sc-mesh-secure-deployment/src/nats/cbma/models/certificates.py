from abc import ABC, abstractmethod
from dataclasses import dataclass
from datetime import datetime


@dataclass
class CBMACertificates(object):
    cert: str
    key: str
    chain: list[str]
    ca: str = ''


class ICertificate(ABC):
    @abstractmethod
    def get_subject_key_identifier(self) -> bool:
        raise NotImplementedError("get_subject_key_identifier not implemented")

    @abstractmethod
    def get_authority_key_identifier(self) -> bytes:
        raise NotImplementedError("get_authority_key_identifier not implemented")

    @abstractmethod
    def get_end_date(self) -> datetime:
        raise NotImplementedError("get_end_date not implemented")
