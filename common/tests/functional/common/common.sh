#!/bin/bash

if [[ $EUID -ne 0 ]]; then
    echo "This script must be run as root"
    exit 1
fi

# trap ctrl-c and call ctrl_c()
trap ctrl_c INT

# Global constants
readonly log_file="./functional_tests_debug.log"
readonly results_file="./functional_tests_results.log"
readonly NEEDED_EXECUTABLES="wpa_supplicant hostapd iw iwlist iperf3 ping uniq grep ip ifconfig batctl bc brctl iptables"
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
# ctrl-c trap
# Globals:
# Arguments:
#######################################
function ctrl_c() {
  echo "** Trapped CTRL-C"
  exit 0
}

#######################################
# Set Batman OGM interval
# Globals:
# Arguments:
#######################################
set_batman_orig_interval() {
  batctl orig_interval "$1"
  ret=$?
  if [ "$ret" != 0 ]; then
    echo "Batman orig_interval setting failed!!"
  fi
}

#######################################
# Set Batman routing algorithm
# Globals:
# Arguments:
#######################################
 set_batman_routing_algo(){
  batctl ra "$1"
  ret=$?
  if [ "$ret" != 0 ]; then
    echo "Batman routing algo setting failed!!"
    echo "Batman-adv Kernel configuration might not be correct."
  fi

 }
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
#  $4 = country
#######################################
create_wpa_supplicant_config() {
  cat <<EOF > "$1"
ctrl_interface=DIR=/var/run/wpa_supplicant
# use 'ap_scan=2' on all devices connected to the network
# this is unnecessary if you only want the network to be created when no other networks..
ap_scan=1
country=$4
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
# Create hostapd configuration
# Globals:
#  retval_channel, retval_band
# Arguments:
#  $1 = config file name
#  $2 = frequency
#  $3 = NONE/SAE
#  $4 = country
#  $5 = interface
#  $6 = ssid
#######################################
create_hostapd_config() {
  # Calculate frequency band and channel from given frequency
  calculate_wifi_channel "$2"

  cat <<EOF > "$1"
ctrl_interface=/var/run/hostapd
interface=$5
hw_mode=$retval_band
channel=$retval_channel
ieee80211h=1
ieee80211d=1
ieee80211w=1
country_code=$4
ssid=$6
auth_algs=1
wpa=2
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
wpa_passphrase=1234567899
wmm_enabled=1
beacon_int=100
### IEEE 802.11n
ieee80211n=1
ht_capab=[HT40+][LDPC][SHORT-GI-20][SHORT-GI-40][TX-STBC][RX-STBC1][DSSS_CCK-40]
### IEEE 802.11ac
#ieee80211ac=1
#vht_capab=[MAX-MPDU-11454][RXLDPC][SHORT-GI-80][TX-STBC-2BY1][RX-STBC-1]
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
               for val in $retval_name; do
                 iw dev "$val" del
               done
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
# Calculate wifi channel and frequency band
# Globals: retval_channel, retval_band
# Arguments:
#  $1 = frequency
#######################################
calculate_wifi_channel()
{
  # Calculate 2.4/5GHz frequency band and channel number
  if [ "$1" -ge  "5160" ] && [ "$1" -le  "5885" ]; then
      retval_band="a"
      retval_channel=$((("$1"-5000)/5))
  elif [ "$1" -ge  "2412" ] && [ "$1" -le  "2472" ]; then
      retval_band="g"
      retval_channel=$((("$1"-2407)/5))
  else
      echo "ERROR! frequency out of range!"
      exit 1
  fi
}

#######################################
# Execute command and print command to log
# Globals:
# Arguments:
#  $@ = command to be executed
#######################################
exe() {
  echo + "$@"
  # shellcheck disable=SC2068
  $@ | print_log
}

#######################################
# Wait IP address until it is online
#  - will timeout after ~60seconds
# Globals:
# Arguments:
#  $1 = IP address to wait
#######################################
wait_ip(){
  server_wait=1
  timeout=0;

  echo -n "waiting for $1 ..."
  while [ "$server_wait" -eq 1 ]; do
    if ping -c 1 -W 2 "$1" &> /dev/null; then
      echo "$1 is online!"
      server_wait=0
    else
      echo -n "."
      ((timeout=timeout+2))
      if [ "$timeout" -gt 60 ]; then
        echo "FAILED: Timeout when waiting $1!" | print_log result
        exit 0  # no reason to continue as target IP is not in network
      fi
    fi
  done
}

################### MAIN ####################

main() {
  echo "Common checks before tests.." | print_log
  for e in $NEEDED_EXECUTABLES; do
    if ! command -v "$e" >/dev/null; then
      echo FAIL: missing "$e" | print_log
      exit
    fi
  done
}

main
