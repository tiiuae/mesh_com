import dataclasses
from dataclasses import dataclass
import json
import socket
import subprocess
import uuid


@dataclass
class NeighborInformation:
  interface: str
  mac: str
  last_seen: str


@dataclass
class LocalNodeInformation:
  hostname: str
  mac: str
  neighbors: list[NeighborInformation]
  next_hops: list[str]


class LocalNode:
  """LocalNode collects all the wanted information from the device Lucius
     is running on and returns them as JSON.
  """
  def __init__(self, batman_interface: str) -> None:
    self.batman_interface = batman_interface

    self.data: LocalNodeInformation = LocalNodeInformation(
      socket.gethostname(),
      LocalNode.get_mac_address(self.batman_interface),
      [],
      [],
    )

  @staticmethod
  def run_command(command: str):
    return subprocess.Popen(command, shell=True, stdout=subprocess.PIPE).stdout.read().decode('UTF-8')

  # TODO get mac address of interface that corresponds to batman_interface
  @staticmethod
  def get_mac_address(interface: str):
    # hardcoded for now
    interface = "wlp1s0"
    return LocalNode.run_command(f"cat /sys/class/net/{interface}/address").strip()

  @staticmethod
  def parse_neighbors_response(response: str) -> list[NeighborInformation]:
    neighbors: list[NeighborInformation] = []
    data_rows = response.strip().split("\n")[2:]
    for row in data_rows:
      # this works for BATMAN_IV
      # interface, mac, last_seen = row.split()
      # this works for BATMAN_V
      mac, lastseen_interface = row.split(maxsplit=1)
      lastseen_interface = lastseen_interface.strip()
      last_seen = lastseen_interface.split()[0]
      interface = lastseen_interface.split("[")[1].split("]")[0]

      neighbors.append(NeighborInformation(interface, mac, last_seen))
    
    return neighbors

  # for BATMAN_IV
  # @staticmethod
  # def parse_originators_response(response: str) -> list[str]:
  #   """
  #     Parses starred lines from 'batctl meshif <bat-if> originators' response

  #     @response (str): originators response to parse, for example:
  #      "[B.A.T.M.A.N. adv 2021.3, MainIF/MAC: wlp1s0/04:f0:21:bf:88:f4 (bat0/12:c3:c6:37:ee:56 BATMAN_IV)]
  #         Originator        last-seen (#/255) Nexthop           [outgoingIF]
  #         04:f0:21:bf:88:bc    0.748s   (194) 04:f0:21:b8:c3:c2 [    wlp1s0]
  #         04:f0:21:bf:88:bc    0.748s   (187) 04:f0:21:bf:89:29 [    wlp1s0]
  #       * 04:f0:21:bf:88:bc    0.748s   (255) 04:f0:21:bf:88:bc [    wlp1s0]
  #         04:f0:21:bf:89:29    0.564s   (194) 04:f0:21:b8:c3:c2 [    wlp1s0]
  #         04:f0:21:bf:89:29    0.564s   (198) 04:f0:21:bf:88:bc [    wlp1s0]
  #       * 04:f0:21:bf:89:29    0.564s   (246) 04:f0:21:bf:89:29 [    wlp1s0]
  #         04:f0:21:b8:c3:c2    0.232s   (187) 04:f0:21:bf:89:29 [    wlp1s0]
  #         04:f0:21:b8:c3:c2    0.232s   (191) 04:f0:21:bf:88:bc [    wlp1s0]
  #       * 04:f0:21:b8:c3:c2    0.232s   (251) 04:f0:21:b8:c3:c2 [    wlp1s0]"

  #     Returns: str array containing MAC addresses of next hops
  #   """
  #   data_rows = response.strip().split("\n")[2:]
  #   data_rows = list(filter(lambda row: row.lstrip().startswith('*'), data_rows))

  #   def get_next_hop_mac(row):
  #     row = row.lstrip()
  #     cols = row.split()
  #     return cols[4]

  #   next_hop_mac_addresses = list(map(get_next_hop_mac, data_rows))
    
  #   return next_hop_mac_addresses

  # for BATMAN_V
  @staticmethod
  def parse_originators_response(response: str) -> list[str]:
    """
      Parses starred lines from 'batctl meshif <bat-if> originators' response

      @response (str): originators response to parse, for example:
       "[B.A.T.M.A.N. adv 2021.3, MainIF/MAC: lmb00301a4fc821/00:30:1a:4f:c8:21 (bat0/02:30:1a:4f:c8:21 BATMAN_V)]
          Originator        last-seen ( throughput)  Nexthop           [outgoingIF]
        * 00:30:1a:50:2f:9d   15.540s (        1.0)  00:30:1a:50:2f:9d [lmb00301a4fc821]"

      Returns: str array containing MAC addresses of next hops
    """
    data_rows = response.strip().split("\n")[2:]
    data_rows = list(filter(lambda row: row.lstrip().startswith('*'), data_rows))

    def get_next_hop_mac(row):
      row = row.lstrip()
      cols = row.split()
      return cols[-2]

    next_hop_mac_addresses = list(map(get_next_hop_mac, data_rows))
    
    return next_hop_mac_addresses

  def _get_neighbors(self) -> list[NeighborInformation]:
    response = LocalNode.run_command(f"batctl meshif {self.batman_interface} neighbors")
    return LocalNode.parse_neighbors_response(response)

  def _get_next_hops(self) -> list[str]:
    response = LocalNode.run_command(f"batctl meshif {self.batman_interface} originators")
    originators = LocalNode.parse_originators_response(response)
    # delete duplicates from list
    unique_originators = list(set(originators))

    return unique_originators

  def update_data(self) -> None:
    self.data.neighbors = self._get_neighbors()
    self.data.next_hops = self._get_next_hops()

  def to_dict(self) -> str:
    return dataclasses.asdict(self.data)

  def to_json(self) -> str:
    try:
      data_dict = dataclasses.asdict(self.data)
      return json.dumps(data_dict)
    except ValueError as err:
      raise Exception(f"Parsing string '{data_dict}' to JSON failed: {str(err)}")