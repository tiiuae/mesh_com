#!/bin/bash

source ./common/common.sh

test_case="mesh_com v1.0 client functionality"

description="mesh_com client Test Case"
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
#  wifi dev
#  channel_list
# Arguments:
#######################################
_test() {
  echo "$0, test called" | print_log log_client

  if ! [ "$result" -eq "$FAIL" ]; then
    start_mesh_client
  fi
}

#######################################
# Result
# Globals:
#  result
# Arguments:
#######################################
_result() {
  echo "$0, result called" | print_log log_client

	SERVER_VALID=$(cat "$MESH_COM_ROOTDIR/modules/sc-mesh-secure-deployment/src/testclient.txt")
	CLIENT_VALID=$(cat "$MESH_COM_ROOTDIR/modules/sc-mesh-secure-deployment/src/testclient1.txt")
	CLIENT_MAC=$(cat "$MESH_COM_ROOTDIR/modules/sc-mesh-secure-deployment/src/testclientmac.txt")

  if [[ "$SERVER_VALID" ]] && [[ "$CLIENT_VALID" ]] ; then
    echo -e "Client is valid: $CLIENT_MAC" | print_log log_client
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
  echo "$0, deinit called" | print_log log_client
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
  echo "### Test Case: $test_case" | print_log result_client
  echo "### Test Description: $description" | print_log result_client

  _init

  if [ "$result" -eq "$FAIL" ]; then
	echo "FAILED  _init: $test_case" | print_log result_client
	exit 0
  fi

  _test

  if [ "$result" -eq "$FAIL" ]; then
	echo "FAILED  _test: $test_case" | print_log result_client
	exit 0
  fi

   sleep 10
  _result

  if [ "$result" -eq "$FAIL" ]; then
	echo "FAILED  : $test_case" | print_log result_client
	exit 0
  else
	echo "PASSED  : $test_case" | print_log result_client
  fi

  _deinit
}

main
