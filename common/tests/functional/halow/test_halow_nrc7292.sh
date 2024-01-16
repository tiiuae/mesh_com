#!/bin/bash

source ./../common/common.sh

test_case="halow (nrc7292) functionality test"

description="Check for halow (nrc7292) functionality is working properly"


#######################################
# Init
# Globals:
# Arguments:
#######################################
_init() {
  echo "$0, init called" | print_log
  #Globals
  BUSNO=""
  MODULE="nrc"
  interface="halow1"
  mcs_int=-1
  freq_int=-1
  country=""
  us_freq_start=9025
  us_freq_end=9275
  eu_freq_start=8635
  eu_freq_end=8675
  nrc_formatted_version=0.0
}

#######################################
# Test
# Globals:
# Arguments:
#######################################
get_mcs() {
  mcs_str=$(cli_app show config | grep "MCS")

  if [ -z "$mcs_str" ]; then
    echo "Failed to retrieve mcs information"
    mcs_int=-1
  else
    # Parse and retrieve mcs value
    mcs=${mcs_str#*: }
    mcs_int=$((mcs))
  fi
}

#######################################
# Test
# Globals:
# Arguments:
#######################################
get_freq() {
  freq_str=$(cli_app show config | grep "Frequency")

  if [ -z "$freq_str" ]; then
    echo "Failed to retrieve frequency information"
  else
    # Parse and retrieve mcs value
    if (( $(echo "$nrc_formatted_version >= 1.5" | bc -l) )); then
      #Format is "Frequency			 : 9205 (37)"
      freq_str_1=${freq_str#*: }
      freq="${freq_str_1%% *}"
      freq_int=$((freq))
    else
      #Format is "Frequency			 : 9205
      freq=${freq_str#*: }
      freq_int=$((freq))
    fi
  fi
}

#######################################
# Test
# Globals:
# Arguments:
#######################################
get_country() {
  country_str=$(cli_app show config | grep "Country")

  if [ -z "$country_str" ]; then
    echo "Failed to retrieve country information"
  else
    # Parse and retrieve mcs value
    country=${country_str#*: }
  fi
}

#######################################
# Test
# Globals:
# Arguments:
#######################################
_test() {
  echo "$0, test called" | print_log

  echo -e "\nchecking for halow spi interface..."
  sleep 1
  for i in $(ls /sys/class/spi_master/); do
      if [[ "$(basename $(readlink /sys/class/spi_master/$i/device))" =~ ^spi-ft232h\.[0-9]+$ ]]; then
          BUSNO=($(echo $i | sed 's/[^0-9]*//g'))
          break
      fi
  done  

  if [ "$BUSNO" == "" ]; then
    echo "spi interface not found"
    result=$FAIL
    return
  fi 

  echo "spi interface found [spi$BUSNO], checking nrc module installed..."
  sleep 1

  if lsmod | grep -wq "$MODULE"; then
    echo "$MODULE module is installed!"
  else
    echo "$MODULE module is not installed!"
    result=$FAIL
    return
  fi


  echo -e "\nchecking for halow interface..."
  sleep 1

  if ifconfig "$interface" | grep -q "UP"; then
    echo "$interface interface is up and running"
  else
    echo "$interface interface not up"
    result=$FAIL
  fi

  echo -e "\nchecking for nrc driver version..."
  sleep 1

  nrc_version_str=$(cli_app show version | grep "Newracom Firmware Version")

  if [ -z "$nrc_version_str" ]; then
    echo "Failed to retrieve Newracom firmware version"
    result=$FAIL
    return
  else
    # Parse and retrieve only firmware version value
    nrc_version=${nrc_version_str#*: }
    echo "Newracom firmware version is $nrc_version"

    nrc_formatted_version=$(awk -F. '{print int($1)"."int($2)}' <<< "$nrc_version")

    if (( $(echo "$nrc_formatted_version >= 1.4" | bc -l) )); then
      echo "Newracom version check completed and pass"
    else
      echo "Newracom version should be minimum of 1.4.0"
      result=$FAIL
      return
    fi
  fi

  echo -e "\nchecking wpa_supplicant is running with halow interface..."
  sleep 1

  if ps -A | grep -w 'wpa_supplicant' | grep -q "$interface"; then
    echo "wpa_supplicant is running with halow interface"
  else
    echo "wpa_supplicant with halow interface is not running"
    result=$FAIL
    return
  fi

  echo -e "\nchecking halow interface working in mesh point mode..."
  sleep 1

  if iw dev "$interface" info | grep -wq 'type mesh point'; then
    echo "halow interface working in mesh point mode"
    result=$PASS
  else
    echo "halow interface is not working in mesh point mode"
    result=$FAIL
  fi  
  
  echo -e "\nchecking manual mcs value setting is working..."
  sleep 1

  #set rate control (rc) off
  if cli_app set rc off; then
    echo "rate control set to off"
  else
    echo "Failed to set rate control off"
    result=$FAIL
    return
  fi

  for i in {0..7} 10; do
    echo "Setting mcs $i..."
    cli_app test mcs $i
    sleep 1
    get_mcs
    if [ "$mcs_int" -eq "$i" ]; then
      echo "Verified current mcs and it is set to $i"
    else
      echo "Mismatch from the mcs value. Failed!"
      result=$FAIL
      return
    fi
  done

  sleep 1
  echo "setting rate control back to default (on)"
  cli_app set rc on

  echo -e "\nchecking halow operating frequency..."
  sleep 1

  get_country
  get_freq

  if [ "$country" = "US" ]; then
    echo "Halow is operting in US country regulation, check for operating freq"
    if [ "$freq_int" -ge "$us_freq_start" ] && [ "$freq_int" -le "$us_freq_end" ]; then 
      echo "Operating frequency $freq_int is valid"
    else
      echo "Operating frequency $freq_int is not valid"
      result=$FAIL
      return
    fi
  elif [ "$country" = "EU" ]; then
    echo "Halow is operting in EU country regulation, check for operating freq"
    if [ "$freq_int" -ge "$eu_freq_start" ] && [ "$freq_int" -le "$eu_freq_end" ]; then 
      echo "Operating frequency $freq_int is valid"
    else
      echo "Operating frequency $freq_int is not valid"
      result=$FAIL
      return
    fi
  else
    echo "Invalid/Empty Country Code"
    result=$FAIL
    return
  fi

  echo -e "\nchecking halow interface is added with batman routing..."
  sleep 1

  if batctl if | grep "$interface: active"; then
    echo "halow interface is active and attached with batman"
  else
    echo "halow interface is not attached with batman or not active!"
    result=$FAIL
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
  _test
  _result
  _deinit

}

main "$@"