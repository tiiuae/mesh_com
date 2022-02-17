#!/bin/bash

#######################################
# DeInit
# Globals:
#  device_list
# Arguments:
#######################################
_deinit() {
  echo "$0, deinit called" | print_log

  ifconfig "$wifidev" down 2>/dev/null
  ifconfig bat0 down 2>/dev/null
  ifconfig br-lan down 2>/dev/null
  brctl delbr br-lan 2>/dev/null
  batctl -m bat0 interface destroy 2>/dev/null
  killall iperf3 2>/dev/null
  killall wpa_supplicant 2>/dev/null
}