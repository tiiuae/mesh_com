#!/bin/bash

source ../common/common.sh   # common tools
source ./osf_common.sh       # common osf tools

test_case="OSF"
description="Start iperf3 server"

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
# Test
# Globals:
#  result
# Arguments:
#######################################
_tests() {
  echo "$0, test called" | print_log

 # kill old instances of iperf3 servers
 killall iperf3 &>/dev/null
 sleep 0.5
 killall iperf3 &>/dev/null
 sleep 0.5
 
 # check if iperf3 is installed
 if ! [ -x "$(type -P iperf3)" ]; then
  echo "ERROR: script requires iperf3"
  result=$FAIL
  return
  fi
  
 # start iperf3 server underground
 iperf3 -s&
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

  # starts iperf3 server
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
