#!/bin/bash

source ../common/common.sh   # common tools
source ./osf_common.sh       # common osf tools

test_case="OSF"
description="Test read statistics"

# define globals
result=$PASS
NODEIPV6ADDR=0
NODEPORT=7777

#######################################
# Init
# Globals:
#  result
# Arguments:
#######################################
_init() {
  echo "$0, init called" | print_log

  # Check if osf interface exists
  if ! ip link show "$OSF_INTERFACE" &>/dev/null; then
    echo "osf interface does not exist." | print_log
    result=$FAIL
    return
  fi
}

#######################################
# Tests of udp slip commands
# Globals:
# result
# Arguments:
#######################################

tc_1() {

  # Extract the raw output from slipcmd to get the IPv6 address of the node. We need to stop and start OSF as slipcmd 
  # uses the same serial ports as tunslip6, and doesn't release them until after the script ends, so jams tunslip6.
  /etc/init.d/S98osf52setup stop
  sleep 3
  
  echo -n "TC 1: Get IPV6 address of the node "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'?Y' "$SERIALPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:2}" != "!Y" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n  [ "${slip_output:2:60}" ]
  echo " result= PASS"
  NODEIPV6ADDR="${slip_output:2:60}"
  fi
 
  # recover osf interface
  /etc/init.d/S98osf52setup start
  sleep 3
  return
}

tc_2() {
  slip_output=$(echo -n "?SFVER" | nc -6u -w1 -W1 "$NODEIPV6ADDR" "$NODEPORT")
  echo -n "TC 2: Get FW version "
  #slip_output=$(echo -n "?SFVER" | nc -6u -w1 -W1 "$NODEIPV6ADDR" "$NODEPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:6}" != "!SFVER" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n  [ "${slip_output:7:20}" ]
  echo " result= PASS"
  fi
  return
}

tc_3() {
  echo -n "TC 3: Get timesync node_id "
  slip_output=$(echo -n "?SFTS" | nc -6u -w1 -W1 "$NODEIPV6ADDR" "$NODEPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:5}" != "!SFTS" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n  [ "${slip_output:6:3}" ]
  echo " result= PASS"
  fi
  return
}

tc_4() {
  echo -n "TC 4: Get RF parameters "
  slip_output=$(echo -n "?SFRF" | nc -6u -w1 -W1 "$NODEIPV6ADDR" "$NODEPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:5}" != "!SFRF" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n  [ "${slip_output:8:20}" ]
  echo " result= PASS"
  fi
  return
}

tc_5() {
  echo -n "TC 5: Get RSSI "
  slip_output=$(echo -n "?RS" | nc -6u -w1 -W1 "$NODEIPV6ADDR" "$NODEPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:5}" != "!RSSI" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n  [ "${slip_output:6:30}" ]
  echo " result= PASS"
  fi
  return
}

tc_6() {
  echo -n "TC 6: Clear statistical counters "
  slip_output=$(echo -n "?SZ" | nc -6u -w1 -W1 "$NODEIPV6ADDR" "$NODEPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:3}" != "!SZ" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n  [ "${slip_output:6:30}" ]
  echo " result= PASS"
  fi
  return
}

tc_7() {
  echo -n "TC 7: Get RX statistics for SYNC type of round "
  slip_output=$(echo -n "?SS" | nc -6u -w1 -W1 "$NODEIPV6ADDR" "$NODEPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:3}" != "!SS" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n  [ "${slip_output:3:30}" ]
  echo " result= PASS"
  fi
  return
}

tc_8() {
  echo -n "TC 8: Get RX statistics for TX type of round "
  slip_output=$(echo -n "?ST" | nc -6u -w1 -W1 "$NODEIPV6ADDR" "$NODEPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:3}" != "!ST" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n  [ "${slip_output:3:30}" ]
  echo " result= PASS"
  fi
  return
}

tc_9() {
  echo -n "TC 9: Get RX statistics for ACK type of round "
  slip_output=$(echo -n "?SA" | nc -6u -w1 -W1 "$NODEIPV6ADDR" "$NODEPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:3}" != "!SA" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n  [ "${slip_output:3:30}" ]
  echo " result= PASS"
  fi
  return
}

#######################################
# Test
# Globals:
#  result
# Arguments:
#######################################
_tests() {
  echo "$0, test called" | print_log

 # call tests
 # Request address of the node
 tc_1
 if [ "$result" -eq "$FAIL" ]; then
    echo "FAILED  _test: $test_case" | print_log result
    return 0
 fi 
 # Request version of FW
 tc_2
 # Request Timesync node id
 ##tc_3
 # Request RF parameters
 tc_4
 # Request RSSI
 tc_5
 # Clear statistical counters
 ##tc_6
 # Request RX statistics for SYNC type of round
 tc_7
 # Request RX statistics for TX type of round
 tc_8
 # Request RX statistics for ACK type of round
 tc_9
}

#######################################
# Result
# Globals:
#  result
# Arguments:
#######################################
_result() {
  echo "$0, result called" | print_log

  # Analyse logs from logs/ - folder
  # ... 

  # Make decision ($PASS or $FAIL)
  # ... (this is a placeholder, adjust as needed)
  #if [ 1 -gt 0 ]; then
    #result=$PASS
  #else
    #result=$FAIL
  #fi
}

#######################################
# main
# Globals:
#  result
# Arguments:
#######################################
main() {
  local help_text=0

  while getopts ":h" flag; do
    case "${flag}" in
      h) help_text=1;;
      *) help_text=1;;
    esac
  done

  if [ "$help_text" -eq 1 ]; then
    echo "
        Usage: sudo $0 [-h]
        -h              Help"
    return
  fi

  echo "### Test Case: $test_case" | print_log result
  echo "### Test Description: $description" | print_log result

  _init
  if [ "$result" -eq "$FAIL" ]; then
    echo "FAILED  _init: $test_case" | print_log result
    exit 0
  fi

  # Extract IPV6 address of the node and execute tests.
  _tests
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

main "$@"
