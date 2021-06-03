## Introduction

This diagram shows the basic behavior of the distributed authentication.

![alt text](../../../images/distributed-auth.png?style=centerme)

Initially, we have one node serving as an authenticator. After some nodes are connected, the server considers the possibility to give the responsibility of creating another server with an authenticated node, called server 2.
When a new node arrives, as node 2, it will request who are the servers, both servers (S1 and S2), will announce themself as available. The new incomer will evaluate which is the closest server in terms of latency. Once the node determines the closest, it will send the request to join the network.
The new server creates a random subnet of the mesh for providing new IP addresses.