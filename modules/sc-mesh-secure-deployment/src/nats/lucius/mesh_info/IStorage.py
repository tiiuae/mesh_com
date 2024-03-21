from abc import ABC, abstractmethod

class IStorage(ABC):
  @abstractmethod
  def store_local_data(self, node_data: dict) -> None:
    raise NotImplementedError("store_local_data not implemented")

  @abstractmethod
  def get_local_data(self) -> dict:
    raise NotImplementedError("get_local_data not implemented")

  @abstractmethod
  def store_other_node_data(self, node_data: list[dict]) -> None:
    raise NotImplementedError("store_other_node_data not implemented")

  @abstractmethod
  def get_other_node_data(self) -> list[dict]:
    raise NotImplementedError("get_other_node_data not implemented")