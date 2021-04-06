# sc-mesh-secure-deployment

## About
This code provides provisioning and authentication services for a Wi-Fi mesh network employing [BATMAN-adv](https://www.open-mesh.org/projects/batman-adv/wiki) at layer-2 on Ubuntu, and is based on the configuration provided [here](https://github.com/tiiuae/mesh_com), from: [Technology Innovation Institute (TII)](https://tii.ae).

The Server side authenticates nodes validating certificates based on the Elliptic Curve Integrated Encryption Scheme (ECIES). The code can be found [here](https://github.com/tiiuae/cryptolib).

## Introduction

Initially, the client sends a request to join the mesh network. This request is attached with node certificates. Once the server validates the certificate, it encrypts the mesh parameters and sends them back to the requested node. The client detects the OS that is running (Ubuntu or OpenWRT) and sets the specific configuration of the mesh network.

![alt text](images/Diagram.png?style=centerme)


## Installation
To get started, either

1. Copy the *install.sh* script to the home folder of your mounted Ubuntu drive.
2. On your host machine, copy download the *install.sh* script from this repo into your home folder.

Once you have done this, startup your host machine and run install.sh.

```bash
cd ~
sudo ./install.sh
```

This script will do the following...

1. Give you the option to connect to an access point (*N.B. This should be the same network as your server*).
2. Download all required packages.
3. Clone **this** repository (https://github.com/tiiuae/sc-mesh-secure-deployment.git)

Once this process is complete, you should now have a git repo called **sc-mesh-secure-deployment** in your home directory alongside the install.sh script.

```bash
cd sc-mesh-secure-deployment
```

Using the *configure.sh* script, you can now set up two different configurations, **server** and **client**, as well as either connect to an access point or set your machine up as an access point.

```bash
./configure.sh --help
```

### On the Server-Side
Your server will provide the necessary authentication certificates for you mesh network, as well as avahi services to allow clients to autodiscover the authentication server and automatically fetch these certificates. The *configure.sh* script should guide you through the process. The server must be executed on the */home/<username>/sc-mesh-secure-deployment* path. To set your machine up as a server, please run...

```bash
./configure.sh -s
```

### On the Client Side
Likewise, the *configure.sh* script should guide you through the process of setting your host up as a mesh client.

1. It will automatically try to discover the server through avahi in order to fetch the certificates **(make sure you are connected to the same network as the server during this process!)**.
2. The **FIRST** node to connect to the server will automatically be set up as the mesh gateway, thus providing the mesh with internet access.

To set your machine up as a client, please run...

```bash
./configure.sh -c
```

Please note that when the configuration is complete **the node will reboot** and automatically connect to the BATMAN-adv L2 network. You now have three options:

1. Leave the client as a L2 router with BATMAN-adv.
2. Set a secondary wlan interface as a Wi-Fi Access Point to allow you to connect STA devices to the network.
3. Set a secondary wlan interface to connect to a Wi-Fi AP and act as gateway.

### Setup a Client as an Access Point / Gateway

To set your client up as an access point, the configuration script has an *-ap* option that allows you to either **connect to** or **create** an Access Point. Run the configuration script as follows...


```bash
./configure.sh -ap
```

### Usage
On the server open a web browser and type...

```bash
http://127.0.0.1:5000
```
A web page with the authenticated and no-authenticated nodes should be displayed

![alt text](images/server-screenshot.png?style=centerme)
