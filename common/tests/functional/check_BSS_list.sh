#!/bin/bash

source ./common/common.sh

test_case="check BSS list in scan results of 2.4/5GHZ MBSS"

description="Scan mesh point in all major channels in 2.4/5GHZ band."

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
#  $1 = scan duration per channel
#  $2 = meshid to search if given
#######################################
_test() {
  echo "$0, test called" | print_log

	#1 make test and scan all supported freqs
  if [ "$2" = "" ]; then
    iw dev "$wifidev" scan duration "$1" | grep -e "^BSS" -e "freq:" |& tee logs/"$0".log
  else
    iw dev "$wifidev" scan duration "$1" | grep -e "^BSS" -e "$2" -e "freq:" |& tee logs/"$0".log
  fi

  ret=$?
  if [ "$ret" != 0 ]; then
    result=$FAIL
    echo "error: with scanning frequencies" | print_log result
    return
  fi
}

#######################################
# Result
# Globals:
#  result
# Arguments:
#  $1 = scan duration per channel
#  $2 = meshid to search if given
#######################################
_result() {
  echo "$0, result called" | print_log
  low_band=0
  high_band=0

  #1 analyse log
  uniq_freqs=$(grep -e "freq:" logs/"$0".log | uniq -u)
  echo -e "unique BSS freqs found: \n $uniq_freqs" | print_log
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
	echo "BSS: low_band: $low_band high_band: $high_band" | print_log result
	if [ "$low_band" -eq 0 ] && [ "$high_band" -eq 0 ]; then
	  result=$FAIL
	  echo "Probably a problem in setup low BSS counts." | print_log result
	fi

  if [ "$2" != "" ]; then
    meshids=$(grep -e "MESH ID" logs/"$0".log | wc -l)
    echo -e "MBSS id: \"$2\" count: $meshids" | print_log
  fi
	if [ "$meshids" -eq 0 ] && [ "$2" != "" ]; then
	  result=$FAIL
	  echo "Cant find searched meshid." | print_log result
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

  killall iperf3 2>/dev/null
  killall wpa_supplicant 2>/dev/null
}

#######################################
# main
# Globals:
#  result
# Arguments:
#######################################
main() {

  duration="500"
  meshid=""
  help_text=0

  while getopts ":m:d:h" flag
  do
    case "${flag}" in
      m) meshid=${OPTARG};;
      d) duration=${OPTARG};;
      h) help_text=1;;
      *) help_text=1;;
    esac
  done

  if [ "$help_text" -eq 1 ]; then
    echo "
        Usage: sudo $0 [-i ipaddress] [-h]

        -m meshid       meshid name which will be scanned.
                        If not given, script will scan all possible BSS.
        -d duration     duration in ms to spend in channel (default:500)

        For Example: sudo $0 -m test_case_run -d 2000

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

  _test "$duration" "$meshid"

  if [ "$result" -eq "$FAIL" ]; then
  	echo "FAILED  _test: $test_case" | print_log result
  	exit 0
  fi

  _result "$duration" "$meshid"

  if [ "$result" -eq "$FAIL" ]; then
   	echo "FAILED  : $test_case" | print_log result
   	exit 0
  else
  	echo "PASSED  : $test_case" | print_log result
   	exit 0
  fi

  _deinit
}

main "$@"