#!/bin/bash

source ../common/common.sh   # common tools
source ./osf_common.sh       # common osf tools

test_case="OSF"
description="Test OSF interface"

# define globals
PASS=0
FAIL=1
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
_test() {
  echo "$0, test called" | print_log

  # Extract the raw output from slipcmd to get the IPv6 address of the node. We need to stop and start OSF as slipcmd 
  # uses the same serial ports as tunslip6, and doesn't release them until after the script ends, so jams tunslip6.
  /etc/init.d/S98osf52setup stop
  sleep 3
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'?Y' "$SERIALPORT")
  /etc/init.d/S98osf52setup start
  sleep 4

  # Check if slipcmd returned any output
  if [ -z "$slip_output" ]; then
    echo "slipcmd did not return an IPv6 address." | print_log
    result=$FAIL
    return
  fi

  # Extract the IPv6 address from slip_output
  ip_address=$(echo "$slip_output" | grep -oE "$OSF_IPV6_PREFIX"[0-9a-fA-F:]+)

  # Check if ip_address is empty after extraction
  if [ -z "$ip_address" ]; then
    echo "Failed to extract IPv6 address from slipcmd output." | print_log
    result=$FAIL
    return
  fi

  # Use ping6 to ping the IP address over the osf interface
  ping6 -c 3 "$ip_address" &>/dev/null
  if [ "$?" -ne 0 ]; then
    echo "ping6 test to $ip_address over osf failed." | print_log
    result=$FAIL
    return
  fi

  result=$PASS
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

main "$@"
