# mesh_com

mesh_com ROS node subsriber listening topic "mesh_parameters". 
Supports following configuration JSON as String format:

```json
{
 "api_version_gs": 1,                 "interface version for future purposes"
 "ssid_gs": "gold",                   "0-32 octets, UTF-8, shlex.quote chars limiting"
 "key_gs": "foobar",                  "key for the network"
 "enc_gs": "wep",                     "encryption (wep, wpa2, wpa3, sae)"
 "ap_mac_gs": "00:11:22:33:44:55",    "bssid for mesh network"
 "country_gs": "fi",                  "Country code, sets tx power limits and supported
                                   channels"
 "frequency_gs": "5220",              "wifi channel frequency, depends on the country
                                   code and HW"
 "ip_gs": "192.168.1.1",              "select unique IP address"
 "subnet_gs": "255.255.255.0",        "subnet mask"
 "tx_power_gs": "30",                 "select 30dBm, HW and regulations limiting it
                                   correct level.
                                   Can be used to set lower dBm levels for testing
                                   purposes (e.g. 5dBm)"
 "mode_gs": "mesh"                    "mesh=mesh network, ap=debug hotspot"
}
```

