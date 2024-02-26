#!/bin/bash

source ../common/common.sh   # common tools
source ./osf_common.sh       # common osf tools

test_case="OSF"
description="Test osf performance BLE_125K"

# define globals
result=$PASS
server_ipaddress="$1"

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
  echo -n "TC 1: Short ping6 of ${server_ipaddress}, "
  output=$(ping6 -W2 -c5 -s32 "${server_ipaddress}" | tail -1| awk '{print $4}' | cut -d '/' -f 2)
  if [ "$?" -ne 0 ] || [ -z "$output" ]; then
    echo -n "ping6 error or timeout !"
    result=$FAIL
    echo " result= FAIL"
  else
    echo -n "average ${output} ms,"
    if (( $(echo "${output} > 1500" |bc -l) )); then
      echo " result= FAIL"
      result=$FAIL
    else 
      echo " result= PASS"
    fi  
  fi
  return
}

tc_2() {
  echo -n "TC 2: Long ping6 of ${server_ipaddress}, "
  output=$(ping6 -W2 -c5 -s1412 "${server_ipaddress}" | tail -1| awk '{print $4}' | cut -d '/' -f 2)
  if [ "$?" -ne 0 ] || [ -z "$output" ]; then
    echo -n "ping6 error or timeout !"
    result=$FAIL
    echo " result= FAIL"
  else
    echo -n "average ${output} ms,"
    if (( $(echo "${output} > 1500" |bc -l) )); then
      echo " result= FAIL"
      result=$FAIL
    else 
      echo " result= PASS"
    fi  
  fi
  return
}

tc_3() {
  echo -n "TC 3: UDP iperf3 to ${server_ipaddress} chunk 188 bytes, "
  output=$(timeout --preserve-status 8s iperf3 -6 -u -c"${server_ipaddress}" --forceflush -t5 -l188 -b15K | awk '/receiver/ {print $7}')
  if [ "$?" -ne 0 ] || [ -z "$output" ]; then
    echo -n "iperf3 error or timeout !"
    result=$FAIL
    echo " result= FAIL"
  else
    echo -n "average receiver bitrate ${output} Kbits/sec,"
    if (( $(echo "${output} < 2" |bc -l) )); then
      echo " result= FAIL"
      result=$FAIL
    else 
      echo " result= PASS"
    fi  
  fi
  return
}

tc_4() {
  echo -n "TC 4: UDP iperf3 to ${server_ipaddress} chunk 360 bytes, "
  output=$(timeout --preserve-status 8s iperf3 -6 -u -c"${server_ipaddress}" --forceflush -t5 -l360 -b15K | awk '/receiver/ {print $7}')
  if [ "$?" -ne 0 ] || [ -z "$output" ]; then
    echo -n "iperf3 error or timeout !"
    result=$FAIL
    echo " result= FAIL"
  else
    echo -n "average receiver bitrate ${output} Kbits/sec,"
    if (( $(echo "${output} < 2" |bc -l) )); then
      echo " result= FAIL"
      result=$FAIL
    else 
      echo " result= PASS"
    fi  
  fi
  return
}

tc_5() {
  echo -n "TC 5: TCP iperf3 to ${server_ipaddress} chunk 188 bytes, "
  output=$(timeout --preserve-status 8s iperf3 -6 -c"${server_ipaddress}" --forceflush -t5 -l188 -b10K | awk '/receiver/ {print $7}')
  if [ "$?" -ne 0 ] || [ -z "$output" ]; then
    echo -n "iperf3 error or timeout !"
    result=$FAIL
    echo " result= FAIL"
  else
    echo -n "average receiver bitrate ${output} Kbits/sec,"
    if (( $(echo "${output} < 1" |bc -l) )); then
      echo " result= FAIL"
      result=$FAIL
    else 
      echo " result= PASS"
    fi  
  fi
  return
}

tc_6() {
  echo -n "TC 5: TCP iperf3 to ${server_ipaddress} chunk 500 bytes, "
  output=$(timeout --preserve-status 8s iperf3 -6 -c"${server_ipaddress}" --forceflush -t5 -l500 -b10K | awk '/receiver/ {print $7}')
  if [ "$?" -ne 0 ] || [ -z "$output" ]; then
    echo -n "iperf3 error or timeout !"
    result=$FAIL
    echo " result= FAIL"
  else
    echo -n "average receiver bitrate ${output} Kbits/sec,"
    if (( $(echo "${output} < 1" |bc -l) )); then
      echo " result= FAIL"
      result=$FAIL
    else 
      echo " result= PASS"
    fi  
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
  
 # Check if iperf3 is installed
 if ! [ -x "$(type -P iperf3)" ]; then
  echo "ERROR: script requires iperf3"
  result=$FAIL
  return
 fi 

 # Call tests
 # Short ping
 tc_1
 if [ "$result" -eq "$FAIL" ]; then
   # fast exit if address not known
   return
 fi
 # Long ping
 sleep 2
 tc_2
 # UDP iperf3 small chunks
 sleep 2
 tc_3
 # UDP iperf3 large chunks
 #sleep 2
 ##tc_4
 # TCP iperf3 small chunks
 #sleep 2
 ##tc_5
 # TCP iperf3 large chunks
 #sleep 2
 ##tc_6 
 sleep 2
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
  
  # Check if ipv6 address of iperf3 server given as parameter
  if [ "$#" -ne "1" ]; then
    echo "ERROR: script require ipv6 address of iperf3 server on other host !"
    echo "FAILED  main: $test_case" | print_log result
    exit 0
  fi

  _init
  if [ "$result" -eq "$FAIL" ]; then
    echo "FAILED  _init: $test_case" | print_log result
    exit 0
  fi

  # Latency and throughput tests
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
