#!/bin/bash

if [[ $EUID -ne 0 ]]; then
    echo "This script must be run as root"
    exit 1
fi

# Global constants
readonly log_file="./functional_tests_debug.log"
readonly results_file="./functional_tests_results.log"
readonly NEEDED_EXECUTABLES="wpa_supplicant iw iwlist iperf3 ping uniq grep ip ifconfig batctl bc"
readonly iperf3_port="30001" # constant port number used for iperf3
readonly PASS=1     # constant for PASS value
readonly FAIL=0     # constant for FAIL value

# Global variables which should be used from test script
result=$PASS        # Should be updated from test script as a result
phyname=0           # Should be updated from test script
wifidev=0           # Should be updated from test script
device_list=0       # includes wifi phy list when find_wifi_device() is called
channel_list=0      # includes supported wifi channels when update_channel_list() is called

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
# create wpa_supplicant configuration
# Globals:
#  channel_list
# Arguments:
#  $1 = config file name
#  $2 = frequency
#  $3 = NONE/SAE
#######################################
create_wpa_supplicant_config() {
  cat <<EOF > "$1"
ctrl_interface=DIR=/var/run/wpa_supplicant
# use 'ap_scan=2' on all devices connected to the network
# this is unnecessary if you only want the network to be created when no other networks..
ap_scan=1
country=AE
p2p_disabled=1
network={
  ssid="test_case_run"
  bssid=00:11:22:33:44:55
  mode=5
  frequency=$2
  psk="1234567890"
  key_mgmt=$3
  ieee80211w=2
  beacon_int=100
}
EOF
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

#######################################
# Execute command and print command to log
# Globals:
# Arguments:
#  $@ = command to be executed
#######################################
exe() {
  echo + "$@"
  $@ | print_log
}

#######################################
# Wait IP address until it is online
# Globals:
# Arguments:
#  $1 = IP address to wait
#######################################
wait_ip(){
  server_wait=1
  echo -n "waiting for $1 ..."
  while [ "$server_wait" -eq 1 ]; do
    if ping -c 1 -w 2 $1 &> /dev/null; then
      echo "$1 is online!"
      server_wait=0
    else
      echo -n "."
    fi
  done
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


