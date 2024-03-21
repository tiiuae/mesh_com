import argparse
import json
import sys

from mesh_info.collection import DataCollector
from mesh_info.scheduler import DataCollectionScheduler
from mesh_info.DictStorage import DictStorage
from mesh_info.local import LocalNode

def main_local_node(batman_interface: str):
  local_node = LocalNode(batman_interface)

  local_node.update_data()

  data = local_node.to_json()

  print(data)

def main_data_collection(batman_interface: str):
  data_collector = DataCollector(batman_interface)
  storage = DictStorage()
  scheduler = DataCollectionScheduler(data_collector, storage)

  scheduler.start()

  input("Press Enter to exit...")

  scheduler.stop()


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument("-b", "--batman_interface", default="bat0")
  args = parser.parse_args()

  # main_local_node(args.batman_interface)
  main_data_collection(args.batman_interface)
