from enum import IntEnum
import json
import os
import subprocess

class RequestType(IntEnum):
  LOWER_MACSEC = 64
  UPPER_MACSEC = 65

ALFRED_BINARY = "/usr/bin/alfred"
ALFRED_UPPER_MACSEC_SOCKET = "/var/run/alfred_upper.sock"

DEVICE_ID = "/opt/identity"
NODE_ID = "/var/tmp/node_id"

def get_device_id():
    with open(DEVICE_ID, mode='r') as file:
        return file.read().strip()

def calculate_node_id():
    if os.path.isfile(NODE_ID):
        with open(NODE_ID, mode='r') as file:
            return file.read()

    UNIQUE_NODE_ID_COMMAND = "sha256sum /proc/cpuinfo | awk '{print $1}'"
    node_id = subprocess.Popen(UNIQUE_NODE_ID_COMMAND, shell=True, stdout=subprocess.PIPE).stdout.read().decode('UTF-8').strip()

    with open(file=NODE_ID, mode='w') as file:
        file.write(node_id)
    return node_id


class AlfredComms:
  def __init__(self, batman_interface: str) -> None:
    if batman_interface == "bat0":
      self.request_type = RequestType.LOWER_MACSEC
      self.socket_args = "" # default
    elif batman_interface == "bat1":
      self.request_type = RequestType.UPPER_MACSEC
      self.socket_args = f" -u {ALFRED_UPPER_MACSEC_SOCKET}"
    else:
      raise Exception(f"Unknown batman_interface '{batman_interface}', do not know how to handle")

    self.device_id = get_device_id()
    self.node_id = calculate_node_id()

  @staticmethod
  def parse_message(message: str) -> dict:
    # remove the trailing ','. If it is not found abort.
    message = message.rstrip()
    if message.rfind(',') != len(message) - 1:
      raise Exception(f"Invalid message from Alfred, no trailing ',' found in {message}")
    message = message[:len(message) - 1]
    
    message = message.lstrip('{ ').rstrip('} ')

    # maxsplit == 1, we want to split only mac and data, not touch what is in data
    mac, data = message.split(", ", maxsplit=1)
    mac = mac.strip('"')
    data = data.strip('"')
    data = data.rstrip()
    data = data.encode('utf-8').decode('unicode_escape')

    try:
      return { "mac": mac, "data": json.loads(data) }
    except ValueError as err:
      raise Exception(f"Parsing string '{data}' to JSON failed: {str(err)}")

  @staticmethod
  def parse_response(response: str) -> list[dict]:
    # successful response is of format:
    # '{ <mac1>, <data1> },
    #  { <mac2>, <data2> },
    #  ...
    # '

    # remove '\x0a"' left by Alfred
    # when data is sent to Alfred using command line, for example:
    # echo '<data>' | alfred <args>,
    # Alfred adds \x0a to the <data>
    response = response.replace("\x0a\"", "\"")

    # split messages
    messages = response.split("\n")
    parsed_messages = []
    for m in messages:
      if len(m) > 0:
        parsed_messages.append(AlfredComms.parse_message(m))

    return parsed_messages


  def send(self, data: dict) -> int:
    data['id'] = self.device_id
    data['sha256sum'] = self.node_id
    send_command = f"echo '{json.dumps(data)}' | {ALFRED_BINARY} -s {self.request_type}{self.socket_args}"
    return subprocess.Popen(send_command, shell=True, stdout=subprocess.PIPE).returncode

  def receive(self) -> dict:
    receive_command = f"{ALFRED_BINARY} -r {self.request_type}"
    response_str = subprocess.Popen(receive_command, shell=True, stdout=subprocess.PIPE).stdout.read().decode('UTF-8')

    if not response_str or len(response_str) == 0:
      return {}
    
    return AlfredComms.parse_response(response_str)
