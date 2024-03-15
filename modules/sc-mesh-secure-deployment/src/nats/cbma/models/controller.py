from abc import ABC, abstractmethod
from typing import Optional


class ICBMAController(ABC):
    @abstractmethod
    def add_interface(self, interface: str, is_upper: bool = False) -> bool:
        raise NotImplementedError("add_interface not implemented")


    @abstractmethod
    def remove_interface(self, interface: str) -> bool:
        raise NotImplementedError("remove_interface not implemented")


    @abstractmethod
    def start(self) -> bool:
        raise NotImplementedError("start not implemented")


    @abstractmethod
    def stop(self) -> bool:
        raise NotImplementedError("stop not implemented")


    @abstractmethod
    def join(self, timeout: Optional[float] = None) -> bool:
        raise NotImplementedError("join not implemented")
