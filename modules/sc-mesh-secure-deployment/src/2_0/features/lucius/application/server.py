import signal
import sys

import json
import threading

from application.types import BATMAN_INTERFACES, NODE_DATA_REQUEST, SocketSendType
from communication.socket import Socket
from communication.httpserver import LuciusHTTPServer

from mesh_info.collection import DataCollector
from mesh_info.scheduler import DataCollectionScheduler
from mesh_info.DictStorage import DictStorage


class Server:
  def __init__(self) -> None:
    self.data_collectors = []
    self.storages = []
    self.schedulers = []

    for batman_interface in BATMAN_INTERFACES:
      data_collector = DataCollector(batman_interface)
      storage = DictStorage()
      scheduler = DataCollectionScheduler(data_collector, storage)

      self.data_collectors.append(data_collector)
      self.storages.append(storage)
      self.schedulers.append(scheduler)

    self.socket = Socket()

    def http_client_response_callback(data: str, send: SocketSendType):
      send(json.dumps(self._get_latest_data()))

    self.http_server = LuciusHTTPServer(http_client_response_callback)

    signal.signal(signal.SIGINT, self.__signal_handler)

  def __signal_handler(self, sig, frame):
    print('You pressed Ctrl+C!')
    self._stop()
    sys.exit(0)

  def _get_latest_data(self):
    return {
      "local": {
        BATMAN_INTERFACES[0]: self.storages[0].get_local_data(),
        BATMAN_INTERFACES[1]: self.storages[1].get_local_data(),
      },
      "other": {
        BATMAN_INTERFACES[0]: self.storages[0].get_other_node_data(),
        BATMAN_INTERFACES[1]: self.storages[1].get_other_node_data(),
      },
    }

  def start(self):
    for scheduler in self.schedulers:
      scheduler.start()

    self.http_server.start()

    def client_response_callback(data: str, send: SocketSendType):
      if data == NODE_DATA_REQUEST:
        send(json.dumps(self._get_latest_data()))

    self.socket.serve(client_response_callback)

  def _stop(self):
    for scheduler in self.schedulers:
      scheduler.stop()

    self.http_server.stop()