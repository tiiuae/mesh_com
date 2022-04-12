#!/bin/bash

source ./common/common.sh

test_case="mesh_com v1.0 ping latency on batman interface"

description="mesh_com v1.0 quantifying ping latency between batman-enabled  nodes"
#######################################
# Init
# Globals:
# results
# device_list
# Arguments:
#######################################
_init() {
  test_mesh_com_1.0_dependencies
  mac_addr=`python3 neigh_mac.py`
  if [ ${#mac_addr[*]} -gt 0 ]
  then
     echo "Neigbours exist and there count is : ${#mac_addr[*]} and addresses are $mac_addr "
  else
    echo "No neighbours exist, so exiting IPERF test setup"
    exit 1
  fi
}

_mesh_pinglatency_test() {
echo "  Please copy the mac address from the available list to generate ping latency statistics: $mac_addr "
read mac_ping
echo "  Provide the ping interval in seconds "
read ping_interval
echo "Provide the number of ping packets to send  "
read ping_count
batctl ping $mac_ping -i $ping_interval -c $ping_count > ping_latency_result.txt
}

_result() {
echo " Generated statistics for ping latency corresponding to the node with mac address $mac_ping are provided below  ..........."
awk 'END{print}' ping_latency_result.txt      
}



_deinit() {
echo "Ping latency measurements obtained and hence quitting..............."
}

main() {
   echo "### Test Case: $test_case" | print_log result_client
   echo "### Test Description: $description" | print_log result_client

  _init

  _mesh_pinglatency_test

  _result

  _deinit

}

main
