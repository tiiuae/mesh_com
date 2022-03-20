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

python3 ${PWD}/python_scripts/flood_pr.py $interface

}

## Init , test and deinit are called.

main(){

	_init

	_test

	_deinit

	_result

}

main "$@"
