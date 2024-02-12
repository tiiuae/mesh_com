#!/bin/bash

source ./../common/common.sh

test_case="wifi functionality test"

description="Check for wifi functionality is working properly"


#######################################
# Init
# Globals:
# Arguments:
#######################################
_init() {
  #Globals
  interface="wlp1s0"
}

#######################################
# Test
# Globals:
# Arguments:
#######################################
_test() {
  echo "$0, test called" | print_log

  echo -e "\nchecking for $interface interface in iw dev..."

  if iw dev | grep -i -q "Interface $interface"; then
    echo "$interface interface is found in iw dev"
  else
    echo "$interface interface is not found in iw dev"
    result=$FAIL
    return
  fi

  echo -e "\nchecking for $interface interface UP and running..."

  if ifconfig "$interface" | grep -q "UP"; then
    echo "$interface interface is up and running"
  else
    echo "$interface interface not up"
    result=$FAIL
    return
  fi

  echo -e "\nchecking wpa_supplicant is running with $interface interface..."

  if ps -A | grep -w 'wpa_supplicant' | grep -q "$interface"; then
    echo "wpa_supplicant is running with $interface interface"
  else
    echo "wpa_supplicant with $interface interface is not running"
    result=$FAIL
    return
  fi

  echo -e "\nchecking $interface interface working in mesh point mode..."

  if iw dev "$interface" info | grep -wq 'type mesh point'; then
    echo "$interface interface working in mesh point mode"
    result=$PASS
  else
    echo "$interface interface is not working in mesh point mode"
    result=$FAIL
    return
  fi  
  
  echo -e "\nchecking $interface interface is added with batman routing..."

  if batctl if | grep "$interface: active"; then
    echo "$interface interface is active and attached with batman"
  else
    echo "$interface interface is not attached with batman or not active!"
    result=$FAIL
    return
  fi

  echo -e "\nchecking for tx packets on the interface $interface..."

  tx_packets=$(ifconfig $interface | grep -E "TX packets" | awk '{print $3}')
  echo "$tx_packets packets transmitted on this interface"
  
  if [ $tx_packets == "0" ]; then
    echo "Tx packets should be more than 0!"
    result=$FAIL
    return
  fi
 
  result=$PASS
}

#######################################
# Result
# Globals:
# Arguments:

#######################################
_result() {

  if [ "$result" -eq "$FAIL" ]; then
   	echo "FAILED  : $test_case" | print_log result
  else
  	echo "PASSED  : $test_case" | print_log result
  fi

}

#######################################
# DeInit
# Globals:
# Arguments:
#######################################
_deinit() {
  echo "$0, deinit called" | print_log
}

#######################################
# main
# Globals:
#  result
# Arguments:
#######################################
main() {

  _init
  help_text=0
  while getopts ":i:" opt
  do
    case "${opt}" in
      i)
        interface="$OPTARG"
        ;;
      \?)
        help_text=1;;
    esac
  done

  if [ "$help_text" -eq 1 ]; then
    echo "
        Usage: $0 [-i interface_name]
        Ex: $0 -i wlp2s0
        "
    return
  fi

  echo "interface $interface selected for test..."
  _test
  _result
  _deinit

}

main "$@"