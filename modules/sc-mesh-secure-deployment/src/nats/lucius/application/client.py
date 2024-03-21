from application.types import NODE_DATA_REQUEST
from communication.socket import Socket


class Client:
  def __init__(self) -> None:
    self.socket = Socket()

  def get_node_data(self):
    self.socket.send(NODE_DATA_REQUEST)