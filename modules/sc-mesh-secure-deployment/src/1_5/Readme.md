# Mesh Shield 1.5

The Mesh Shield 1.5 is a security solution for mesh network devices that provides advanced protection against malicious attacks. It includes several security features that help ensure the security of the mesh network, including:

1. Mutual Authentication: This feature verifies the identity participant devices  before allowing access to the network. It helps prevent unauthorized access and protects against identity theft.
    - For the PoC (MS 1.5) we use prime field Weierstrass 256-bit Elliptic Curves (Also known as secp256r1P-256). In addition, we use SOFT-HSM with the pkcs11-tool for key handling.

2. Continuous Authentication: This feature monitors network activity in real-time, detecting and preventing any unusual or suspicious behavior. It helps prevent attacks that can bypass traditional security measures such as firewalls and antivirus software.

3. Decision Engine: The decision engine uses is a rule-based solution that creates a degree of trust for each node. 

4. Malicious Announcement: This feature automatically blocks any devices that are identified as malicious or compromised. It prevents these devices from accessing the network and spreading malware.

5. Quarantine: The quarantine feature isolates any devices that are suspected of being compromised or infected. It prevents them from accessing the network until they are thoroughly scanned and deemed safe.

The following image shows a flow diagram of the Mesh Shield 1.5:

<img src="./common/img/15Arc.png"/>


## Execution

The Mesh Shield 1.5 (MS1.5) runs on a docker automatically loaded during booting. 
The MS1.5 path is on ```/opt/container-data/mesh/mesh_com/modules/sc-mesh-secure-deployment/src/1_5#```
* `SC_MESH_FOLDER=/opt/container-data/mesh/mesh_com/modules/sc-mesh-secure-deployment/src/1_5`

The system has a modularity execution that make it easy to run one or all the security features. 
By default, the system will run Mutual Authentication (step 1) and the security beat (step 2 to 5).
Nevertheless, it is possible to disable one or all the security features on the file: 
``cat features.yaml``
```
only_mesh: false
NESS: false
continuous: false
mutual: true
secbeat: true
quarantine: false
mesh: true
provisioning: false
``` 

## Configuration
1. To configure the features that need to run on the system modify the `features.yaml` file. 
If the `provisioning` feature is true, then the `root_cert.der` file should be placed on the `/etc/ssl/certs` folder. Otherwise, the system uses a "fake" `root_cert.der` for testing. 
2. If any specific parameter of the mesh need to modify (`SSID`, `frequency`, etc)  modify the `$SC_MESH_FOLDER/common/mesh_com_11s.conf`.  
   * This file should be modified by the provisioning server. 
3. For setting the mesh shield version and installing the machine learning packages, modify the file `mesh.conf` on the docker file
   * Currently only the last to variables are used in MS1.5 (MSVERSION and ML) --> this file needs to be integrated `with mesh_com_11s.conf` file