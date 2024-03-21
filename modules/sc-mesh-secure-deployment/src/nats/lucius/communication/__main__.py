import argparse
import json
import sys

from communication.alfred import AlfredComms
from communication.socket import Socket


def main_alfred(batman_interface: str, send: bool, receive: bool, data: str):
  if send:
    comm = AlfredComms(batman_interface)
    comm.send(json.loads(data))
  elif receive:
    comm = AlfredComms(batman_interface)
    response = comm.receive()
    print(response)
  else:
    print("no option defined")
    sys.exit(1)

def main_socket(server: bool):
  socket = Socket()

  def client_response_callback(data: str, send):
    if data == "GET":
      send("callback response")

  if server:
    socket.serve(client_response_callback)
  else:
    return socket.send("GET")

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument("data", nargs="?", type=str)
  parser.add_argument("-s", "--send", action="store_true")
  parser.add_argument("-r", "--receive", action="store_true")
  parser.add_argument("-b", "--batman_interface", default="bat0")
  args = parser.parse_args()

  batman_interface = args.batman_interface

  #main_alfred(args.batman_interface, args.send, args.receive, args.data)
  main_socket(args.receive)
