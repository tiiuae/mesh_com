#!/bin/bash

source ../common/common.sh   # common tools
source ./osf_common.sh       # common osf tools

test_case="OSF"
description="Test slip commands"

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
  echo -n "TC 3: Get IPV6 address of the node "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'?Y' "$SERIALPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:2}" != "!Y" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n  [ "${slip_output:2:60}" ]
  echo " result= PASS"
  fi
  return
}

tc_4() {
  echo -n "TC 4: Get IPV6 address of the host "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'?H' "$SERIALPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:2}" != "!H" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n  [ "${slip_output:2:60}" ]
  echo " result= PASS"
  fi
  return
}

tc_5() {
  echo -n "TC 5: Get timesync node_id "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'?SFTS' "$SERIALPORT")
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

tc_6() {
  echo -n "TC 6: Get radio physical layer "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'?SFPHY' "$SERIALPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:6}" != "!SFPHY" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n  [ "${slip_output:7:3}" ]
  echo " result= PASS"
  fi
  return
}

tc_7() {
  echo -n "TC 7: Get number of transmission per flood, NTX "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'?SFNTX' "$SERIALPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:6}" != "!SFNTX" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n  [ "${slip_output:7:3}" ]
  echo " result= PASS"
  fi
  return
}

tc_8() {
  echo -n "TC 8: Get Number of TA pairs in the STA, NTA "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'?SFNTA' "$SERIALPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:6}" != "!SFNTA" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n  [ "${slip_output:7:3}" ]
  echo " result= PASS"
  fi
  return
}

tc_9() {
  echo -n "TC 9: Get channels hopping list "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'?SFCHH' "$SERIALPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:6}" != "!SFCHH" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n  [ "${slip_output:7:30}" ]
  echo " result= PASS"
  fi
  return
}

tc_10() {
  echo -n "TC 10: Get KEY1 "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'?SFKEY1' "$SERIALPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:7}" != "!SFKEY1" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n  [ "${slip_output:8:60}" ]
  echo " result= PASS"
  fi
  return
}

tc_11() {
  echo -n "TC 11: Get KEY2 "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'?SFKEY2' "$SERIALPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:7}" != "!SFKEY2" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n  [ "${slip_output:8:60}" ]
  echo " result= PASS"
  fi
  return
}

tc_12() {
  echo -n "TC 12: Get RF parameters "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'?SFRF' "$SERIALPORT")
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

tc_13() {
  echo -n "TC 13: Get RSSI "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'?RS' "$SERIALPORT")
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

#######################################
# Test
# Globals:
#  result
# Arguments:
#######################################
_tests() {
  echo "$0, test called" | print_log

 # call tests
 tc_0
 # Request osf driver state
 tc_1
 # Request firmware version
 tc_2
 # Request address of the node
 tc_3
 # Request address of the host
 tc_4
 # Request timesync node id
 tc_5
 # Request radio physical layer
 tc_6
 # Request NTX
 tc_7
 # Request NTA
 tc_8
 # Request channels hopping list
 tc_9
 # Request KEY1
 tc_10
 # Request KEY2
 tc_11
 # Request RF parameters
 tc_12
 # Request RSSI
 tc_13
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

  # Extract the raw output from slipcmd to get the IPv6 address of the node. We need to stop and start OSF as slipcmd 
  # uses the same serial ports as tunslip6, and doesn't release them until after the script ends, so jams tunslip6.
  /etc/init.d/S98osf52setup stop
  sleep 3
  _tests
  if [ "$result" -eq "$FAIL" ]; then
    echo "FAILED  _test: $test_case" | print_log result
    #exit 0
  fi
  # recover osf interface
  /etc/init.d/S98osf52setup start
  sleep 4

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
