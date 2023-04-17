#! /bin/bash

killall wpa_supplicant
ifconfig wlp1s0 down
ifconfig wlp1s0 0
ifconfig wlp1s0 up
cp /demo/root_cert.der /etc/ssl/certs/
rm -r  ../auth/; rm -r ../pubKeys/
iptables -F
brctl delif br-lan eth1
ifconfig br-lan down
brctl delbr br-lan
cp
