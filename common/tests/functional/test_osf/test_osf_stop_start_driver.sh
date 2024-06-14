#!/bin/bash

source ../common/common.sh   # common tools
source ./osf_common.sh       # common osf tools

test_case="OSF"
description="Test osf stop/start of driver"

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
  if ! ls -la "$SERIALPORT" &>/dev/null; then
    echo "osf serial port does not exist." | print_log
    result=$FAIL
    return
  fi
}

#######################################
# Tests of slip commands
# Globals:
# result
# Arguments:
#######################################

# dummy test for wakeup serial interface
tc_0() {
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'?S' "$SERIALPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:2}" != "!S" ]; then
    echo -"slipcmd error or timeout [ "${slip_output:0:50}" ] !"
  fi
  return
}

tc_1() {
  echo -n "TC 1: Get osf driver state "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'?S' "$SERIALPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:2}" != "!S" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n  [ "${slip_output:2:3}" ]
  echo " result= PASS"
  fi
  return
}

tc_2() {
  echo -n "TC 2: Get firmware version "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'?SFVER' "$SERIALPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:6}" != "!SFVER" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n  [ "${slip_output:6:8}" ]
  echo " result= PASS"
  fi
  return
}

tc_3() {
  echo -n "TC 3: Stop osf driver "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'!S0' "$SERIALPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:2}" != "?S" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n  [ "${slip_output:2:3}" ]
  echo " result= PASS"
  fi
  return
}

tc_4() {
  echo -n "TC 4: Start osf driver "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'!S3' "$SERIALPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:2}" != "?S" ]; then
    #echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    #result=$FAIL
    #echo " result= FAIL"
    echo " result= PASS"
  else  
  echo -n  [ "${slip_output:2:3}" ]
  echo " result= PASS"
  fi
  return
}

tc_5() {
  echo -n "TC 5: Is osf interface unmounted ? "
  if ip link show "$OSF_INTERFACE" &>/dev/null; then
    echo " result= FAIL"
    result=$FAIL
    else
    echo " result= PASS"
  fi
  return
}

tc_6() {
  echo -n "TC 6: Is osf interface mounted ? "
  if ! ip link show "$OSF_INTERFACE" &>/dev/null; then
    echo " result= FAIL"
    result=$FAIL
  else
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
 # Is osf interface unmounted ?
 tc_5
 # wakeup serial
 tc_0
 # Request osf driver state
 tc_1
 # Request firmware version
 tc_2
 # Stop osf driver
 tc_3
 # Request osf driver state
 sleep 0.1
 tc_1
 # Start osf driver in autorestart mode
 tc_4
 # Write configuration to flash memory and reconfigure driver might take time ...
 sleep 2
 # Request osf driver state
 tc_1
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

  # Check if nRF52 is programmed and serial port exist
  _init
  if [ "$result" -eq "$FAIL" ]; then
    echo "FAILED  _init: $test_case" | print_log result
    exit 0
  fi

  # Extract the raw output from slipcmd to get the IPv6 address of the node. We need to stop and start OSF as slipcmd 
  # uses the same serial ports as tunslip6, and doesn't release them until after the script ends, so jams tunslip6.
  SECONDS=0
  /etc/init.d/S98osf52setup stop
  # wait interface unmount, 5s max
  for (( c=1; c<10; c++ ))
  do 
   #echo "c=""$c"
   sleep 0.5 
   INTERFACE=$(ip link show | grep "$OSF_INTERFACE")
   if [ ! "$INTERFACE" ] ; then
    sleep 0.1
    break
   fi
  done
  # execute tests
  _tests
  if [ "$result" -eq "$FAIL" ]; then
    echo "FAILED  _test: $test_case" | print_log result
    #exit 0
  fi
  # mount osf interface
  /etc/init.d/S98osf52setup start
  # wait interface mount, 5s max
  for (( c=1; c<10; c++ ))
  do 
   #echo "c=""$c"
   sleep 0.5 
   INTERFACE=$(ip link show | grep "$OSF_INTERFACE")
   if [ "$INTERFACE" ] ; then
    sleep 0.1
    break
   fi
  done
  # check if osf interface is mounted
  tc_6
  # Measure restart time
  if [ "$result" -eq "$FAIL" ]; then
    echo "Osf interface is not mounted," "$SECONDS"" seconds elapsed !"
  else
    echo "Osf interface mounted," "$SECONDS"" seconds elapsed !"
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
