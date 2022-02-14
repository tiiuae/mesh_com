#!/bin/bash

source ./common.sh

test_case="test case name from excel"

description="test case description from excel"

# define globals
# myvar=0

#######################################
# Init
# Globals:
#  result
#  device_list
# Arguments:
#######################################
_init() {

  echo "$0, init called" | print_log
	# detect_wifi
	# multiple wifi options --> can be detected as follows:
	# manufacturer 0x168c = Qualcomm
	# devices = 0x0034 0x003c 9462/988x  11s
	#           0x003e        6174       adhoc
	#           0x0033        Doodle
	find_wifi_device "pci" 0x168c "0x0034 0x003c 0x003e"
	phyname=${device_list[0]}  # only first pci device is used here
	wifidev="mesh0"
  if ! iw phy $phyname interface add $wifidev type mp; then
    result=$FAIL
    return
  fi

	# Create wpa_supplicant.conf here if needed
}

#######################################
# Test
# Globals:
#  result
#  wifidev
#  channel_list
# Arguments:
#######################################
_test() {
  echo "$0, test called" | print_log

  #1 make test and return with fail if command fails

  # my_command executed and pass logs to logs/-folder
  ret=$?
	if [ "$ret" != 0 ]; then
	  result=$FAIL
	  return
	else
	  result=$PASS
	fi

}

#######################################
# Result
# Globals:
#  result
# Arguments:
#######################################
_result() {
  echo "$0, result called" | print_log

  #1 analyse logs from logs/ - folder

  # add here

	#2 make decision ($PASS or $FAIL)
	if [ 1 -gt 0 ]; then
		result=$PASS
	else
	  result=$FAIL
	fi
}

#######################################
# main
# Globals:
#  result
# Arguments:
#######################################
main() {
  echo "### Test Case: $test_case" | print_log result
  echo "### Test Description: $description" | print_log result

  _init

  if [ "$result" -eq "$FAIL" ]; then
  	echo "FAILED  _init: $test_case" | print_log result
  	exit 0
  fi

  _test

  if [ "$result" -eq "$FAIL" ]; then
  	echo "FAILED  _test: $test_case" | print_log result
  	exit 0
  fi

  _result

  if [ "$result" -eq "$FAIL" ]; then
   	echo "FAILED  : $test_case" | print_log result
   	exit 0
  else
  	echo "PASSED  : $test_case" | print_log result
   	exit 0
  fi
}

main