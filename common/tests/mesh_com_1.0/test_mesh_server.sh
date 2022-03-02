#!/bin/bash

source ./common/common.sh

test_case="mesh_com v1.0 server functionality"

description="Test mesh_com server"

#######################################
# Init
# Globals:
#  result
#  device_list
# Arguments:
#######################################
_init() {
  _deinit
  test_mesh_com_1.0_dependencies
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

  if ! [ "$result" -eq "$FAIL" ]; then
    start_mesh_server
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
 
  retries=3

  for ((i=0; i<retries; i++)); do
    server_details=$(timeout 7 avahi-browse -rptf _http._tcp | awk -F';' '$1 == "=" && $3 == "IPv4" && $4 == "mesh_server" {print $4 " " $8 " " $7}')
	if [ "$server_details" ] ; then
	    break
	fi
  done 
  if [ "$server_details" ] ; then
    echo -e "server found: $server_details" | print_log
    result=$PASS
  else
    result=$FAIL
  fi

  
}

#######################################
# DeInit
# Globals:
#  device_list
# Arguments:
#######################################
_deinit() {
  echo "$0, deinit called" | print_log
  #kill flask server
  pid=$(netstat -tlnp | awk '/:5000 */ {split($NF,a,"/"); print a[1]}')
  echo $pid
  kill -9 $pid
  killall wpa_supplicant
  /etc/init.d/S50avahi-daemon stop
  /etc/init.d/S30dbus stop
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

   sleep 10
  _result

  if [ "$result" -eq "$FAIL" ]; then
	echo "FAILED  : $test_case" | print_log result
	exit 0
  else
	echo "PASSED  : $test_case" | print_log result
  fi

  _deinit
}

main
