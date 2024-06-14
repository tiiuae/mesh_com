import threading

from mesh_info.IDataCollection import IDataCollection
from mesh_info.IStorage import IStorage

# schedules the data collection
class DataCollectionScheduler:
  LOCAL_DATA_COLLECTION_TIMEOUT_SECONDS = 5.0
  OTHER_DATA_COLLECTION_TIMEOUT_SECONDS = 2.0

  def __init__(self, data_collector: IDataCollection, storage: IStorage,
               local_data_collection_timeout = LOCAL_DATA_COLLECTION_TIMEOUT_SECONDS, other_data_collection_timeout = OTHER_DATA_COLLECTION_TIMEOUT_SECONDS) -> None:
    self.data_collector = data_collector
    self.storage = storage
    self.local_data_collection_timeout = local_data_collection_timeout
    self.other_data_collection_timeout = other_data_collection_timeout

    self.event = threading.Event()

  def start(self):
    # get data from local node
    local_data_collection_thread = threading.Thread(target=self.collect_local_data, args=(self.data_collector, self.storage, self.event, self.local_data_collection_timeout))
    local_data_collection_thread.start()

    # get data from other nodes
    other_nodes_data_collection_lower_macsec_thread = threading.Thread(target=self.collect_other_node_data, args=(self.data_collector, self.storage, self.event, self.other_data_collection_timeout))
    other_nodes_data_collection_lower_macsec_thread.start()

    self.threads = [local_data_collection_thread, other_nodes_data_collection_lower_macsec_thread]

  def stop(self):
    self.event.set()
    for thread in self.threads:
      thread.join()

  @staticmethod
  def collect_local_data(data_collector: IDataCollection, storage: IStorage, event: threading.Event, timeout: int):
    while not event.is_set():
      data_collector.get_local_data(storage.store_local_data)
      event.wait(timeout)

  @staticmethod
  def collect_other_node_data(data_collector: IDataCollection, storage: IStorage, event: threading.Event, timeout: int):
    while not event.is_set():
      data_collector.get_other_node_data(storage.store_other_node_data)
      event.wait(timeout)