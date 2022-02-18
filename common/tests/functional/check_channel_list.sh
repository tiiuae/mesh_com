#!/bin/bash

source ./common/common.sh

test_case="check available channels in 2.4/5GHZ"

description="Check channels from iwlist with following country code; FI, AE and US"

country_codes="FI AE US"
used_channels="2412 2417 2422 2427 2432 2437 2442 2447 2452 2457 2462 5180 5200 5220 5240 5260 5280 5300 5320 5745 5765 5785 5805 5825"

#######################################
# Init
# Globals:
#  result
#  device_list
# Arguments:
#######################################
_init() {
  _deinit
  echo "$0, init called" | print_log
	# detect_wifi
	# multiple wifi options --> can be detected as follows:
	# manufacturer 0x168c = Qualcomm
	# devices = 0x0034 0x003c 9462/988x  11s
	#           0x003e        6174       adhoc
	#           0x0033        Doodle
	find_wifi_device "pci" 0x168c "0x0034 0x003c 0x003e"
	phyname=${device_list[0]}  # only first pci device is used here
	wifidev="wlp1s0"
  if ! iw phy $phyname interface add $wifidev type mp; then
    result=$FAIL
    return
  fi

	# Create wpa_supplicant.conf here if needed
}

#######################################
# Test
# Globals:
#  result
#  wifidev
#  channel_list
# Arguments:
#######################################
_test() {
  echo "$0, test called" | print_log

  echo "Needed channels: $used_channels" | print_log result

	#1 make test and check all country codes
  for country in $country_codes; do
    iw reg set "$country"
    update_channel_list $wifidev
    echo "$country Channels: $channel_list" | print_log result

    for channel in $used_channels; do
      case $channel_list in
        *"$channel"*)
        ;;
        *)
        result=$FAIL
        echo "error $channel channel missing from country code $country" | print_log result
        return
        ;;
      esac
    done
  done
}

#######################################
# Result
# Globals:
#  result
# Arguments:
#######################################
_result() {
  echo "$0, result called" | print_log

  #1 analyse log

	# status alraedy set in test
}

#######################################
# DeInit
# Globals:
#  device_list
# Arguments:
#######################################
_deinit() {
  echo "$0, deinit called" | print_log

  killall iperf3
  killall wpa_supplicant
}

#######################################
# main
# Globals:
#  result
# Arguments:
#######################################
main() {
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

  _deinit
}

main