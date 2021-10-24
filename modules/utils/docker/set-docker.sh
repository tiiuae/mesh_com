#! /bin/bash

#set default gw (working on the lab)

distro=$(lsb_release -a 2>/dev/null | grep "Distributor ID" | awk '{print $3}')
echo $distro
if [ "$distro" == "Ubuntu" ]; then
sudo apt-get install \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg \
    lsb-release

curl -fsSL https://download.docker.com/linux/debian/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update

sudo apt-get install docker-ce docker-ce-cli containerd.io

dockervar=$(ls /etc/init.d/ |grep docker)

/etc/init.d/$dockervar stop
/etc/init.d/$dockervar start
else
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
fi
