#!/bin/bash

source ../common/common.sh   # common tools
source ./osf_common.sh       # common osf tools

test_case="OSF"
description="Test osf join multicast group"

# define globals
result=$PASS

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
# Tests of osf performance
# Globals:
# result
# Arguments:
#######################################
tc_1() {
  echo -n "TC 1: Join multicast group ${OSF_MCAST_IPV6_ADDR}, port ${OSF_MCAST_IPV6_PORT}, wait message ..."
  output=$(timeout --preserve-status 60s python3 joinmcastgroup6.py ${OSF_MCAST_IPV6_ADDR} ${OSF_MCAST_IPV6_PORT} ${OSF_INTERFACE} 2>&1)
  if [ -z "$output" ]; then
    result=$FAIL
    echo " result= FAIL"
  else
    echo " "
    echo -n "$output"
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
 
 # Call tests
 # Join multicast group and wait UDP message
 tc_1
 if [ "$result" -eq "$FAIL" ]; then
   return
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

  echo "### Test Case: $test_case" | print_log result
  echo "### Test Description: $description" | print_log result
  
  _init
  if [ "$result" -eq "$FAIL" ]; then
    echo "FAILED  _init: $test_case" | print_log result
    exit 0
  fi

  # Join and receive test
  _tests
  if [ "$result" -eq "$FAIL" ]; then
    echo "FAILED  _test: $test_case" | print_log result
    #exit 0
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
