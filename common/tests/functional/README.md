# Functional Test Case Scripts
* Script execution needs root rights and depending on your test environment sudo rights also

## Test Mesh Connection (no VLAN)
* Server providing interface to test against (multiple servers can be started with uniq IP):
```
./test_mesh_connection.sh -i 192.168.1.4 -e SAE -f 2412 -m server -n mesh
```
* (executor) Start tests against 192.168.1.5: 
```
./test_mesh_connection.sh -i 192.168.1.3 -e SAE -f 2412 -m client -s 192.168.1.4 -n mesh
```
* Script help for more parameters:
```
./test_mesh_connection.sh -h
```

## Test Mesh Connection (with VLAN)
* Will chain nodes in IP address order
* Server providing interface to test against (multiple servers can be started with uniq **consecutive** IPs):
```
./test_mesh_connection.sh -i 192.168.1.4 -e SAE -f 2412 -m server -n mesh_vlan
```
* (executor) Start tests against 192.168.1.5: 
```
./test_mesh_connection.sh -i 192.168.1.3 -e SAE -f 2412 -m client -s 192.168.1.4 -n mesh_vlan
```
* Script help for more parameters:
```
./test_mesh_connection.sh -h
```

## Test Layer2 Connection
* Server providing interface to test against (multiple servers can be started with uniq IP):
```
./test_layer2_connection.sh -i 192.168.1.4 -e SAE -f 2412 -m server -n mesh
```
* (executor) Start tests against 192.168.1.5: 
```
./test_layer2_connection.sh -i 192.168.1.3 -e SAE -f 2412 -m client -s 192.168.1.4 -n mesh
```
* Script help for more parameters:
```
./test_layer2_connection.sh -h
```

## Test Mesh Connection + AP
* Server providing interface to test against(creates AP):
```
./test_mesh_connection.sh -i 192.168.1.5 -m server -n mesh_ap
```
* (executor) Start tests against 192.168.1.5 with "-n skip" 
  * use e.g. your Ubuntu desktop to connect AP (passwd:1234567899, static IP: 192.168.1.100, gateway:192.168.1.1 and wpa2)
  * and execute:
```
 ./test_mesh_connection.sh -i 192.168.1.3 -m client -s 192.168.1.5 -n skip
```
* Script help:
```
./test_mesh_connection.sh -h
```
## Without Script Init&Setup
* Server providing interface to test against:
```
./test_mesh_connection.sh -i 192.168.1.5 -m server -n skip
```
* (executor) Start tests against 192.168.1.5 with "-n skip" 
```
./test_mesh_connection.sh -i 192.168.1.3 -m client -s 192.168.1.5 -n skip 
```
* Script help:
```
./test_mesh_connection.sh -h
```

## Check BSS Listing
* Scans all supported channels and reports all unique BSS channels found:
```
./check_BSS_list.sh -m test_case_run -d 1000
```

## Check Channel Lists
* Verifies channel list with FI, AE and US regulatory domains
```
$ ./check_channel_list.sh
```

## Clean All Setups
* Will kill wpa/hostpad/iperf3 processes 
* Tries to remove changed network configurations
* Will remove set Mesh Point Wi-Fi card setup (deletes Wi-Fi interface)
* !! **check common/deinit.sh script content before executing ./test_setup_clean.sh**
```
$ ./test_setup_clean.sh
```
