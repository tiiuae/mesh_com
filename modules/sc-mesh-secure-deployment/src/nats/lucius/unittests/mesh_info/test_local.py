from lucius.mesh_info.local import LocalNode

class TestLocalNode:
  # for BATMAN_IV
  # def test_parse_neighbors_response(self):
  #   response = """[B.A.T.M.A.N. adv 2021.3, MainIF/MAC: wlp1s0/00:30:1a:4f:c8:21 (bat0/00:30:1a:4f:c8:21 BATMAN_IV)]
  #     IF             Neighbor              last-seen
  #           wlp1s0	  00:30:1a:4f:cf:3b    0.292s"""
    
  #   neighbors = LocalNode.parse_neighbors_response(response)
  #   assert len(neighbors) == 1
  #   assert neighbors[0].interface == "wlp1s0"
  #   assert neighbors[0].mac == "00:30:1a:4f:cf:3b"
  #   assert neighbors[0].last_seen == "0.292s"

  # for BATMAN_V
  def test_parse_neighbors_response(self):
    response = """[B.A.T.M.A.N. adv 2021.3, MainIF/MAC: lmb00301a4fc821/00:30:1a:4f:c8:21 (bat0/02:30:1a:4f:c8:21 BATMAN_V)]
IF             Neighbor              last-seen
00:30:1a:50:2f:9d    0.356s (        1.0) [lmb00301a4fc821]"""
    
    neighbors = LocalNode.parse_neighbors_response(response)
    assert len(neighbors) == 1
    assert neighbors[0].interface == "lmb00301a4fc821"
    assert neighbors[0].mac == "00:30:1a:50:2f:9d"
    assert neighbors[0].last_seen == "0.356s"


  # for BATMAN_IV
  # def test_parse_originators_response(self):
  #   response = """[B.A.T.M.A.N. adv 2021.3, MainIF/MAC: wlp1s0/04:f0:21:bf:88:f4 (bat0/12:c3:c6:37:ee:56 BATMAN_IV)]
  #       Originator        last-seen (#/255) Nexthop           [outgoingIF]
  #       04:f0:21:bf:88:bc    0.748s   (194) 04:f0:21:b8:c3:c2 [    wlp1s0]
  #       04:f0:21:bf:88:bc    0.748s   (187) 04:f0:21:bf:89:29 [    wlp1s0]
  #     * 04:f0:21:bf:88:bc    0.748s   (255) 04:f0:21:bf:88:bc [    wlp1s0]
  #       04:f0:21:bf:89:29    0.564s   (194) 04:f0:21:b8:c3:c2 [    wlp1s0]
  #       04:f0:21:bf:89:29    0.564s   (198) 04:f0:21:bf:88:bc [    wlp1s0]
  #     * 04:f0:21:bf:89:29    0.564s   (246) 04:f0:21:bf:89:29 [    wlp1s0]
  #       04:f0:21:b8:c3:c2    0.232s   (187) 04:f0:21:bf:89:29 [    wlp1s0]
  #       04:f0:21:b8:c3:c2    0.232s   (191) 04:f0:21:bf:88:bc [    wlp1s0]
  #     * 04:f0:21:b8:c3:c2    0.232s   (251) 04:f0:21:b8:c3:c2 [    wlp1s0]"""

  #   next_hops = LocalNode.parse_originators_response(response)

  #   assert next_hops == ['04:f0:21:bf:88:bc', '04:f0:21:bf:89:29', '04:f0:21:b8:c3:c2']

  # for BATMAN_V
  def test_parse_originators_response(self):
    response = """[B.A.T.M.A.N. adv 2021.3, MainIF/MAC: lmb00301a4fc821/00:30:1a:4f:c8:21 (bat0/02:30:1a:4f:c8:21 BATMAN_V)]
   Originator        last-seen ( throughput)  Nexthop           [outgoingIF]
 * 00:30:1a:50:2f:9d   15.540s (        1.0)  00:30:1a:50:2f:9d [lmb00301a4fc821]"""

    next_hops = LocalNode.parse_originators_response(response)

    assert next_hops == ['00:30:1a:50:2f:9d']

        