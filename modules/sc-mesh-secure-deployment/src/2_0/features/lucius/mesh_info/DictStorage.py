from mesh_info.IStorage import IStorage

class DictStorage(IStorage):
  def __init__(self) -> None:
    super().__init__()
    self._local_storage = {}
    self._other_nodes_storage = []

  def store_local_data(self, node_data: dict) -> None:
    print(f"store_local_data: {node_data}")
    self._local_storage = node_data

  def get_local_data(self) -> dict:
    return self._local_storage

  def store_other_node_data(self, node_data: list[dict]) -> None:
    print(f"store_other_node_data: {node_data}")
    self._other_nodes_storage = node_data

  def get_other_node_data(self) -> list[dict]:
    return self._other_nodes_storage