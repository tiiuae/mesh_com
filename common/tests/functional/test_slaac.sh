#!/bin/bash

source ./common/common.sh   # common tools

test_case="slaac"
description="Test SLAAC addressing"

# Define globals
PASS=0
FAIL=1
result=$PASS
interface=$1

#######################################
# Init
# Globals:
#  result
# Arguments:
#######################################
_init() {
  echo "$0, init called" | print_log

  # Check if interface is provided
  if [ -z "$interface" ]; then
    echo "Interface not specified" | print_log
    result=$FAIL
    return
  fi

  # Check if interface exists
  if ! ip link show "$interface" > /dev/null 2>&1; then
    echo "Interface $interface does not exist" | print_log
    result=$FAIL
    return
  fi

  # Check if IPv6 is enabled on the interface
  if ! sysctl -a 2>/dev/null | grep -q "net.ipv6.conf.$interface.disable_ipv6 = 0"; then
    echo "IPv6 is not enabled on interface $interface" | print_log
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

  # Get IPv6 addresses assigned to the interface
  ipv6_addresses=$(ip -6 addr show "$interface" scope global | grep "inet6" | awk '{print $2}')

  # No IPv6 address found
  if [ -z "$ipv6_addresses" ]; then
    echo "No IPv6 address assigned to interface $interface" | print_log
    result=$FAIL
    return
  fi

  # Check if any of the addresses matches the expected SLAAC prefix
  # (Assuming the SLAAC prefix is fdff as in the setup script)
  expected_prefix="fdff"
  for addr in $ipv6_addresses; do
    if [[ "$addr" == "$expected_prefix"* ]]; then
      echo "IPv6 address $addr on $interface matches expected SLAAC prefix $expected_prefix" | print_log
      result=$PASS
      return
    fi
  done

  # If we reach this point, no address matched the expected prefix
  echo "No IPv6 address on interface $interface matches the expected SLAAC prefix $expected_prefix" | print_log
  result=$FAIL
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
  if [ "$result" -eq "$PASS" ]; then
    echo "SLAAC addressing is correctly configured on interface $interface" | print_log
  else
    echo "SLAAC addressing configuration failed on interface $interface" | print_log
  fi
}

#######################################
# main
# Globals:
#  result
# Arguments:
#######################################
main() {
  if [ "$#" -ne 1 ]; then
    echo "Usage: sudo $0 <interface>"
    exit 0
  fi

  echo "### Test Case: $test_case" | print_log result
  echo "### Test Description: $description" | print_log result

  _init
  if [ "$result" -eq "$FAIL" ]; then
    echo "FAILED  _init: $test_case" | print_log result
    return
  fi

  _test
  if [ "$result" -eq "$FAIL" ]; then
    echo "FAILED  _test: $test_case" | print_log result
    return
  fi

  _result
  if [ "$result" -eq "$FAIL" ]; then
    echo "FAILED  : $test_case" | print_log result
  else
    echo "PASSED  : $test_case" | print_log result
  fi
}

main "$@"
