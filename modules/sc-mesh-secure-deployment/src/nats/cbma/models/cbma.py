from abc import ABC, abstractmethod


class ICBMA(ABC):
    @abstractmethod
    def start(self) -> bool:
        raise NotImplementedError("start not implemented")


    @abstractmethod
    def stop(self) -> bool:
        raise NotImplementedError("stop not implemented")

