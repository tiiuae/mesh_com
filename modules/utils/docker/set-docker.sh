#! /bin/bash

#set default gw (working on the lab)

echo '> Type the gateway IP address...'
read -p "- IP: " ip

ip route add default via $ip dev eth0

#set dns
echo 'nameserver 8.8.8.8' > /etc/resolv.conf


#set correct date
echo '> Type the current date (format: YYYY-MM-DD)'
read -p "- Date: " date
echo '> Type the current time (format: HH:MM:SS)'
read -p "- Time: " ctime

date -s "$date $ctime"

#restart docker
dockervar=$(ls /etc/init.d/ |grep docker)

/etc/init.d/$dockervar stop
/etc/init.d/$dockervar start

#verify if wlan0 (onboard is working)

adev=$(iw dev |wc -l)

if [ "$adev" == 7 ]; then
echo mmc1:0001:2 > /sys/bus/sdio/drivers/brcmfmac/bind
fi
