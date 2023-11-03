#!/bin/bash

source ./common/common.sh   # common tools

test_case="OSF"
description="Test OSF connection"

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

  # Check if tun0 interface exists
  if ! ip link show tun0 &>/dev/null; then
    echo "tun0 interface does not exist." | print_log
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

  # Check if tun0 has an IPv6 address
  if ! ip -6 addr show tun0 | grep -q "inet6"; then
    echo "tun0 interface does not have an IPv6 address." | print_log
    result=$FAIL
    return
  fi

  # Use ping6 to ping the IP address fd02::1 over the tun0 interface
  if ! ping6 -c 3 -I tun0 fd02::1 &>/dev/null; then
    echo "ping6 test to fd02::1 over tun0 failed." | print_log
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
