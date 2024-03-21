import argparse

from application.client import Client
from application.server import Server


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument("data", nargs="?", type=str)
  parser.add_argument("-s", "--server", action="store_true")

  args = parser.parse_args()

  #main_alfred(args.batman_interface, args.send, args.receive, args.data)
  if args.server:
    print("Running as server")
    server = Server()
    server.start()
  else:
    print("Running as client")
    client = Client()
    print(client.get_node_data())
