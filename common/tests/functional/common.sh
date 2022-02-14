#!/bin/bash

if [[ $EUID -ne 0 ]]; then
    echo "This script must be run as root"
    exit 1
fi

PASS=1
FAIL=0
result=$PASS

phyname=0
wifidev=0
device_list=0     # includes wifi phy list when detect is called
channel_list=0    # includes supported wifi channels when update function is called

log_file="./functional_tests_debug.log"
results_file="./functional_tests_results.log"

NEEDED_EXECUTABLES="wpa_supplicant iw iwlist"

#######################################
# Add date prefix to line and write to log
# Globals:
# Arguments:
#   debug print lines
#######################################
print_log() {

    time_stamp="$(date +'%Y-%m-%dT%H:%M:%S%z')"
 
    while IFS= read -r line; do
       if [ "$1" = "result" ]; then
          printf '[%s]: %s\n' "$time_stamp" "$line" |& tee -a "$results_file" |& tee -a "$log_file"
       else
          printf '[%s]: %s\n' "$time_stamp" "$line" |& tee -a "$log_file"
       fi
    done
}
#######################################
# update supported channel list
# Globals:
#  channel_list
# Arguments:
#  $1 = wifidev name
#######################################
update_channel_list() {
  channel_list=$(iwlist "$1" frequency | awk 'NR>1{print $4*1000}' | tr '\n' ' ')
  export channel_list
}

#######################################
# Find wifi device
# Globals:
# Arguments:
#  $1 = bus (usb, pci, sdio..)
#  $2 = wifi device vendor
#  $3 = wifi device id list
#######################################
find_wifi_device()
{
  # return values: device_list
  #                format example = "phy0 phy1 phy2"

  device_list=""

  echo "## Searching WIFI card: bus=$1 deviceVendor=$2 deviceID=$3" | print_log

  case "$1" in
    pci)
      # tested only with Qualcomm cards
      phynames=$(ls /sys/class/ieee80211/)
      for device in $3; do
        for phy in $phynames; do
          device_id="$(cat /sys/bus/pci/devices/*/ieee80211/"$phy"/device/device 2>/dev/null)"
          device_vendor="$(cat /sys/bus/pci/devices/*/ieee80211/"$phy"/device/vendor 2>/dev/null)"
          if [ "$device_id" = "$device" ] && [ "$device_vendor" = "$2" ]; then
            retval_phy=$phy
            retval_name=$(ls /sys/class/ieee80211/"$phy"/device/net/ 2>/dev/null)
            if [ "$retval_name" != "" ]; then
              iw dev "$retval_name" del
            fi

            # add "phy,name" pair to device_list (space as separator for pairs)
            device_list="$device_list$retval_phy "
          fi
        done
      done
      ;;
    usb)
      device_list=""
      ;;
    sdio)
      device_list=""
      ;;
  esac
}

################### MAIN ####################

main() {
  echo "Common checks before tests.." | print_log
  for e in $NEEDED_EXECUTABLES; do
    if [ ! $(which "$e") ]; then
      echo FAIL: missing "$e" | print_log
      exit
    fi
  done
}

main


