# Provisioning Server for Mutual Authentication

## Introduction

The provisioning server is needed to create the ID, certificates, sign the public key :lock_with_ink_pen: and provide the certificates to the nodes. The server uses GnuPG software for key creation and signing. 

The provisioning Server is only executed **once**, before the mission, only for key generation,  node ID, and mesh parameters provisioning.The node ID is generated from the user input of the MAC address of the node (interface of the mesh). The MAC address is hashed using the blake2s algorithm. The output is a hexadecimal digest of 2 bytes. The ID is obtained with the conversion of the digest as an integer.

The public :key: and private keys :old_key: are stored in separate files nodeIDpb.asc and nodeIDpr.asc. Then both files are transferred (via scp) to the node. This means that **both node and server must be connected to the same Access Point**. 

The mesh parameters are obtained from an already created YAML file, the [mesh_com.conf](https://github.com/tiiuae/mesh_com/blob/feature/develop_pgp/modules/sc-mesh-secure-deployment/src/mesh_com.conf). The mesh IP is randomly generated and verified if it has not been provisioned. 

Also, the server creates a comma-separated value (csv) of all seen nodes. This file is stored in ‘auth/dev.csv' and contains the Node ID, MAC Address, Mesh IP, and Fingerprint of the public key. 

![Conceptual Diagram](../images/mutual-authentication.png)

## Run Program
### Normal Execution 
```
./provServer.py
```
It will be asked the MAC address (mesh interface) of the destination node

`` Enter a MAC Address (in the format AA:BB:CC:DD:EE) : ``

and then the IP address for transferring the files:

`Would you like to send the certificates to a node? (yes/no): yes`

`Enter node IP address: 192.168.1.100`

### Cleaning Execution
This will clean all the keys generated as well as the files.

```
./provServer.py -c
```

## Code Flow
![Conceptual Diagram](../images/prov-server.png)