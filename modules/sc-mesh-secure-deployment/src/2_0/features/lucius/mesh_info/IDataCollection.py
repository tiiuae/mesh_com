from abc import ABC, abstractmethod

from mesh_info.types import GetLocalDataCallbackType, GetOtherDataCallbackType


class IDataCollection(ABC):
  @abstractmethod
  def get_local_data(self, data_cb: GetLocalDataCallbackType) -> None:
    raise NotImplementedError("get_local_data not implemented")

  @abstractmethod
  def get_other_node_data(self, data_cb: GetOtherDataCallbackType) -> None:
    raise NotImplementedError("get_other_node_data not implemented")