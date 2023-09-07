# Decision-engine  


This is the decision function of the Network Expert Security System (NESS) of the Mesh 1.5 Security architecture.

It is based on [Pyke 1.1.1](https://github.com/evertrol/pyke3), for rules construction and inference.
The decision engine works by proving Goal_A by using a Rule_A, itself having some other Goals to be proven with a Rule for each of them, etc.
The overall approach deployed by NESS is to receive inputs as a dashboard, analyze its content with above proof method and generate actions from what was proved. The dashboard consists of:
 - The total number of nodes in the network
 - The Base table (Security part, as per hash verification) and all Security-related updates from all nodes as received by the current node (as per hashes verification)
 - A security-related entry is a list of the form:
`[nodeID, [Server_sub_list], [Flags_sub_list], Security_result],`

The Security part of the Table is a list of above entries ordered by increasing nodeIDs.

As a result of the proofs, NESS Decision outputs an 8-bit code as an action A to execute.

`Bits 7-6 of A: [01] -> Note, [10] -> Warning, [11] -> Alert`

`Bits 5-0 of A: Action details, (1) -> No Signaling, (3) -> Uncertain Status, (4) -> Consistency Issues, (2) -> Suspected Malicious`

E.g.:

      A = 0b01000001 -> This node is trusted with available information above,

      A = 0b11000010 -> This node is Suspected Malicious with available information above,

      A = 0b10000011 -> This node has Uncertain status.


In NESS decision process, the proofs loop through all the nodes in the network having an entry in the table and an action A is issued for each.

## Data simulator
A [simulator](https://github.com/martin-tii/decision-engine/tree/master/ness_decision/input-simulator) was created to obtain data to test the decision engine
