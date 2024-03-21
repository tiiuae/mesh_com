from abc import ABC, abstractmethod
from typing import Callable

from communication.alfred import AlfredComms
from mesh_info.local import LocalNode

from mesh_info.types import GetLocalDataCallbackType, GetOtherDataCallbackType


class IDataCollectionScheduler(ABC):
  @abstractmethod
  def start(self):
    raise NotImplementedError("start not implemented")

  @abstractmethod
  def stop(self):
    raise NotImplementedError("stop not implemented")
  
  @abstractmethod
  def collect_local_data(local_node: LocalNode, comm: AlfredComms, data_cb: GetLocalDataCallbackType) -> None:
    raise NotImplementedError("get_local_data not implemented")

  @abstractmethod
  def collect_other_node_data(local_node: LocalNode, comm: AlfredComms, data_cb: Callable) -> None:
    raise NotImplementedError("get_other_node_data not implemented")