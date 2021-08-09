# sc-mesh-secure-deployment

## About
This code provides provisioning and authentication services for a Wi-Fi mesh network employing [BATMAN-adv](https://www.open-mesh.org/projects/batman-adv/wiki) at layer-2 on Ubuntu, and is based on the configuration provided [here](https://github.com/tiiuae/mesh_com), from: [Technology Innovation Institute (TII)](https://tii.ae).

The Server side authenticates nodes validating certificates based on the Elliptic Curve Integrated Encryption Scheme (ECIES). The code can be found [here](https://github.com/tiiuae/cryptolib).

## Introduction

Initially, the client sends a request to join the mesh network. This request is attached with node certificates. Once the server validates the certificate, it encrypts the mesh parameters and sends them back to the requested node. The client detects the OS that is running (Ubuntu or OpenWRT) and sets the specific configuration of the mesh network.

![alt text](../../images/Diagram.png?style=centerme)


## Installation
Firstly we need to install linux dependencies.

```bash
$ sudo apt update
$ sudo apt install \
    git make python3-pip batctl ssh clang libssl-dev net-tools \
    iperf3 avahi-daemon avahi-dnsconfd avahi-utils libnss-mdns \
    bmon isc-dhcp-server alfred batctl resolvconf
```

Clone this repository if you haven't already.

```bash
$ git clone https://github.com/tiiuae/mesh_com
```

From the **top level mesh_com directory**, init the *cryptolib* submodule.

```bash
$ git submodule update --init common/security/cryptolib/
```

Using the *configure.sh* script, you can now set up two different configurations, **server** and **client**, as well as either connect to an access point or set your machine up as an access point. Two things to note:

1. The server is currently only used for (i) authenticating clients and can be **any machine on the network** and (ii) distributing the mesh network configuration (e.g., channel, tx_power, etc.)
2. The **FIRST** client node to be authenticated by the server will be configured as a gateway (i.e., for internet access).

```bash
$ cd modules/sc-mesh-secure-deployment/
$ ./configure.sh --help
```

### On the Server-Side

To set up a machine up as an authentication server, **WITHOUT SUDO** please run... 

```bash
$ ./configure.sh -s
```

On the server open a web browser and go to `http://0.0.0.0:5000`. A web page with the authenticated and no-authenticated nodes should be displayed.

![alt text](../../images/server-screenshot.png?style=centerme)

### On the Client Side
To set your node up as a client, please run...

```bash
$ sudo ./configure.sh -c
```

Please remember to note:

1. It will automatically try to discover the server through avahi in order to fetch the certificates **(make sure you are connected to the same network as the server during this process!)**.
2. The **FIRST** node to connect to the server will automatically be set up as the mesh gateway.

When the configuration is complete **the node will reboot** and automatically connect to the BATMAN-adv L2 network. You can test the client node by pinging an address.

```bash
$ ping 8.8.8.8
```

You now have two options:

1. Leave the client as a L2 routing node with BATMAN-adv.
2. Set a secondary WLAN interface as a Wi-Fi Access Point to allow you to connect STA devices to the network (see below).

### Setup a Client as an Access Point / Gateway

To set your client up as an access point, the configuration script has an *-ap* option that allows you to either **connect to** or **create** an Access Point. Run the configuration script as follows...

```bash
sudo ./configure.sh -ap
```
