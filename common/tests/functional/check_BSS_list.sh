#!/bin/bash

source ./common.sh

test_case="check BSS list in scan results of 2.4/5GHZ MBSS"

description="Scan mesh point in all major channels in 2.4/5GHZ band."

uniq_frequencies=""

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
	wifidev="mesh0"
  if ! iw phy $phyname interface add $wifidev type mp; then
    result=$FAIL
    return
  fi

  ifconfig $wifidev up

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

  update_channel_list $wifidev
  echo "Channels: $channel_list" | print_log result

	#1 make test and scan all supported freqs
  for freq in $channel_list; do
    iw dev "$wifidev" scan freq "$freq" duration 500 |& tee logs/"$0".log
    ret=$?
	  if [ "$ret" != 0 ]; then
	    result=$FAIL
	    echo "error: with $freq" | print_log result
	    return
	  else
	    freqs=$(grep -e "freq:" logs/"$0".log | uniq -u >/dev/null)
	    uniq_frequencies="$uniq_frequencies$freqs"
	  fi
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
  low_band=0
  high_band=0

  #1 analyse log
  uniq_freqs=$(grep -e "freq:" logs/"$0".log | uniq)
  echo -e "unique freqs found: \n $uniq_freqs" | print_log

	#2 make decision ($PASS or $FAIL)
	for unit in $uniq_freqs; do
	  if [ "$unit" != "freq:" ]; then
	    if [ "$unit" -lt 5000 ]; then
  	    ((low_band=low_band+1))
	    else
	      ((high_band=high_band+1))
	    fi
	  fi
	done
	echo "Found: low_band:$low_band high_band:$high_band" | print_log result
	if [ "$low_band" -gt 0 ] && [ "$high_band" -gt 0 ]; then
		result=$PASS
	else
	  result=$FAIL
	fi
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