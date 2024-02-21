#!/bin/bash

source ../common/common.sh   # common tools
source ./osf_common.sh       # common osf tools

test_case="OSF"
description="Test osf configuration BLE_1M 1"

# define globals
PASS=0
FAIL=1
result=$PASS
rfparams=0,0,0,0,0

# IPV6 prefix
PREFIX1="fde3,49c,9f74,4742" 

# PHY for S, T and A types of rounds
PHY_BLE_1M=3                
PHY_BLE_2M=4
PHY_BLE_500K=6
PHY_BLE_125K=5
PHY_IEEE=15
# Channels hopping list. 5 channels always.
OSF_CHH="5,10,25,40,80"
# Number of TA pairs 1..12
OSF_NTA="6"
# Set number of transmissions per flood 2..12
OSF_NTX="3"
# KEY1
KEY1="2B,7E,15,16,28,AE,D2,A6,AB,F7,15,88,09,CF,4F,3C"
# KEY2
KEY2="2C,7F,16,17,29,AF,D3,A7,AC,F8,16,89,0A,DF,5F,4C"
# Firmware version
FWVER="301"

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
  echo -n "TC 4: Get RF parameters "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'?SFRF' "$SERIALPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:5}" != "!SFRF" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n  [ "${slip_output:8:20}" ]
  rfparams="${slip_output:8:20}"
  echo " result= PASS"
  fi
  return
}

tc_5() {
  echo -n "TC 5: Set RF parameters "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'!SFRF '"$rfparams" "$SERIALPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:5}" != "?SFRF" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n  [ "${slip_output:8:20}" ]
  echo " result= PASS"
  fi
  return
}

tc_6() {
  echo -n "TC 6: Set PREFIX1 ""$PREFIX1"" "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'!SFPREFIX1 '"$PREFIX1" "$SERIALPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:10}" != "?SFPREFIX1" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n  [ "${slip_output:11:60}" ]
  echo " result= PASS"
  fi
  return
}

tc_7() {
  echo -n "TC 7: Set radio physical layer BLE_1M "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'!SFPHY '"$PHY_BLE_1M" "$SERIALPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:6}" != "?SFPHY" ]; then
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
  echo -n "TC 8: Set number of transmission per flood, NTX=""$OSF_NTX"" "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'!SFNTX '"$OSF_NTX" "$SERIALPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:6}" != "?SFNTX" ]; then
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
  echo -n "TC 9: Set number of TA pairs in the STA, NTA=""$OSF_NTA"" "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'!SFNTA '"$OSF_NTA" "$SERIALPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:6}" != "?SFNTA" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n  [ "${slip_output:7:3}" ]
  echo " result= PASS"
  fi
  return
}

tc_10() {
  echo -n "TC 10: Set channels hopping list "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'!SFCHH '"$OSF_CHH" "$SERIALPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:6}" != "?SFCHH" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n [ "${slip_output:7:30}" ]
  echo " result= PASS"
  fi
  return
}

tc_11() {
  echo -n "TC 11: Set KEY1 "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'!SFKEY1 '"$KEY1" "$SERIALPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:7}" != "?SFKEY1" ]; then
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
  echo -n "TC 12: Set KEY2 "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'!SFKEY2 '"$KEY2" "$SERIALPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:7}" != "?SFKEY2" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else  
  echo -n  [ "${slip_output:8:60}" ]
  echo " result= PASS"
  fi
  return
}

tc_13() {
  echo -n "TC 13: Set firmware version "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'!SFVER '"$FWVER" "$SERIALPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:6}" != "?SFVER" ]; then
    echo -n "slipcmd error or timeout [ "${slip_output:0:50}" ] !"
    result=$FAIL
    echo " result= FAIL"
  else
  echo -n  [ "${slip_output:7:20}" ]
  echo " result= PASS"
  fi
  return
}

tc_20() {
  echo -n "TC 20: Reboot driver in autorestart mode "
  slip_output=$(timeout --preserve-status 2s slipcmd -H -b"$BAUDRATE" -sn -R'!S3' "$SERIALPORT")
  if [ "$?" -ne 0 ] || [ -z "$slip_output" ] || [ "${slip_output:0:2}" != "?S" ]; then
    # echo -n "slipcmd error or timeout [${slip_output:0:50}] !"
    #result=$FAIL
    #echo " result= FAIL"
    # can be timeout
    echo " result= PASS"
  else  
  echo -n  [ "${slip_output:2:3}" ]
  echo " result= PASS"
  fi
  # Write configuration to flash memory and reconfigure driver can take time ...
  sleep 2
  return
}

tc_21() {
  echo -n "TC 21: Get IPV6 address of the node "
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

tc_22() {
  echo -n "TC 22: Get IPV6 address of the host "
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

tc_23() {
  echo -n "TC 23: Get timesync node_id "
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

#######################################
# Test
# Globals:
#  result
# Arguments:
#######################################
_tests() {
  echo "$0, test called" | print_log

 # call tests
 # Request osf driver state
 tc_1
 # Request firmware version
 tc_2
 # Request timesync node id
 tc_23
 # Stop osf driver
 tc_3
 # Request RF parameters
 tc_4
 # Set the same RF parameters. For support different types of boards.
 tc_5
 # Set PREFIX1
 tc_6
 # Set radio physical layer
 tc_7
 # Set NTX
 tc_8
 # Set NTA
 tc_9
 # Set channels hopping list
 tc_10
 # Set KEY1
 tc_11
 # Set KEY2
 tc_12
 # Set Firmware version
 tc_13
 # Write configuration to flash memory and reboot
 tc_20
 # Request osf driver state
 tc_1
 #
 # Request important parameters
 #
 # Request firmware version
 tc_2
 # Request RF parameters
 tc_4
 # Request IPV6 address of the node
 tc_21
 # Request IPV6 address of the host
 tc_22
 # Request timesync node id
 tc_23
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
