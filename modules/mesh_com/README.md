# mesh_com

mesh_com ROS node subsriber listening topic "mesh_parameters".
Supports following configuration JSON as String format:

1st level
```json
{
 "edge": {2nd level},                 "mesh network class. edge=edge mesh network, gs=groundstation mesh network"
}
```

2nd level
```json
{
 "api_version": 1,                 "interface version for future purposes"
 "ssid": "gold",                   "0-32 octets, UTF-8, shlex.quote chars limiting"
 "key": "foobar",                  "key for the network"
 "ap_mac": "00:11:22:33:44:55",    "bssid for mesh network"
 "country": "fi",                  "Country code, sets tx power limits and supported
                                   channels"
 "frequency": "5220",              "wifi channel frequency, depends on the country
                                   code and HW"
 "ip": "192.168.1.1",              "select unique IP address"
 "subnet": "255.255.255.0",        "subnet mask"
 "tx_power": "30",                 "select 30dBm, HW and regulations limiting it
                                   correct level.
                                   Can be used to set lower dBm levels for testing
                                   purposes (e.g. 5dBm)"
 "mode": "mesh"                    "mesh=mesh network, ap=debug hotspot"
}
```
## Google IoT config block example
```python
initial-wifi:
    api_version: 2
    ssid: gold
    key: "1234567890"
    ap_mac: "00:11:22:33:44:55"
    country: cn
    frequency: "5805"
    ip: 192.168.1.3
    subnet: 255.255.255.0
    tx_power: "30"
    mode: mesh
    edge:
        api_version: 2
        ssid: default_edge_mesh
        key: "1234567890"
        ap_mac: "00:11:22:33:44:66"
        country: cn
        frequency: "5220"
        ip: 192.168.246.203
        subnet: 255.255.255.0
        tx_power: "30"
        mode: mesh
    gs:
        api_version: 2
        ssid: default_gs_mesh
        key: "1234567890"
        ap_mac: "00:11:22:33:44:55"
        country: cn
        frequency: "5805"
        ip: 192.168.248.203
        subnet: 255.255.255.0
        tx_power: "30"
        mode: mesh

```