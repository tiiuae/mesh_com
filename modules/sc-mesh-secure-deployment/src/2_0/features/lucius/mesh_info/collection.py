from communication.alfred import AlfredComms
from mesh_info.local import LocalNode
from mesh_info.types import GetLocalDataCallbackType, GetOtherDataCallbackType
from mesh_info.IDataCollection import IDataCollection


class DataCollector(IDataCollection):
  def __init__(self, batman_interface: str) -> None:
    self.local_node = LocalNode(batman_interface)
    self.mac = self.local_node.data.mac
    self.comm = AlfredComms(batman_interface)

  def get_local_data(self, data_cb: GetLocalDataCallbackType):
    self.local_node.update_data()
    local_data = self.local_node.to_dict()

    # publish the local data to the network
    self.comm.send(local_data)

    data_cb(local_data)

  def get_other_node_data(self, data_cb: GetOtherDataCallbackType) -> None:
    nodes_data: list[dict] = self.comm.receive()
    other_nodes_data = list(filter(lambda node_data: (node_data["mac"] != self.mac), nodes_data))
    if len(other_nodes_data) > 0:
      data_cb(other_nodes_data)

