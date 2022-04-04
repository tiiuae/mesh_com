#!/bin/bash

source ./common/common.sh

test_case="mesh_com v1.0 IPERF Traffic Measurements"

description="mesh_com v1.0 Iperf Traffic Throughput Measurements"
#######################################
# Init
# Globals:
# results
# device_list
# Arguments:
#######################################
_init() {
  test_mesh_com_1.0_dependencies
  _deinit

  count=`python neighbour_count.py`
  if [ $count -gt 0 ]
  then
     echo "Neigbours exist and there count is : $count"
  else
    echo "No neighbours exist, so exiting IPERF test setup"
    exit 1
  fi
}

_mesh_iperf_test() {
echo "Provide IP address of the device acting as server over Bat0 interface: "
read server_ip
ssh root@$server_ip 'iperf3 -s -D'
#python server_iperf_script.py
echo "IPERF server is running in background on $server_ip"
echo "Please select the type of traffic"
PS3="Select the traffic type: "
select opt in Voice  Video  BestEffort  Background quit; do

case $opt in
    Voice)
      TOS=0xC0
      echo "Predefined TOS value is 0xC0"
      break
      ;;
    Video)
      TOS=0x80
      echo "Predefined TOS value is 0x80"
      break
      ;;
    BestEffort)
      TOS=0x00
      echo "Predefined TOS value is 0100"
      break
      ;;
    Background)
      TOS=0x40
      echo "Predefined TOS value is 0x40"
      break
      ;;
    quit)
      break
      ;;
    *)
      echo "Invalid option $REPLY"
      ;;
  esac
done

echo "Please select protocol"
PS3="Select among UDP and TCP packet: "
select opt in UDP  TCP  quit; do

  case $opt in
    UDP)
      echo "Generating stats for UDP case-----------please wait................................."
      udp=1
      iperf3 -u -c $server_ip  -b 1M -S $TOS -i 1 > iperf_test_result_udp.txt
      break
      ;;
    TCP)
      echo "Generating stats for TCP case---------please wait.................................."
      tcp=1
      iperf3 -c $server_ip -S $TOS -i 1 > iperf_test_result_tcp.txt
      break
      ;;
    quit)
      break
      ;;
    *)
      echo "Invalid option $REPLY"
      ;;
  esac
done
}

_result() {
     if [ $udp == '1' ] 
     then
      echo " IPERF test result under  UDP protocol..........."
      awk -Wi -F'[ -]+' '/sec/{print " [Interval(seconds); Transfer Rate( MBytes); Throughput(Mbps); Jitter(ms); Loss Datagram/Total Datagram]= "$3"-"$4" "$6"  "$8"  "$10"  "$12}' iperf_test_result_udp.txt | tail -n 1
     fi
     if [ $tcp == '1' ]
     then
      echo " IPERF test result under  TCP protocol..........."
      awk -Wi -F'[ -]+' '/sec/{print " [Interval(seconds); Transfer Rate( MBytes); Throughput(Mbps); Jitter(ms); Loss Datagram/Total Datagram]= "$3"-"$4" "$6"  "$8}' iperf_test_result_tcp.txt | tail -n 1
     fi
    udp=0
    tcp=0
}



_deinit() {
   ssh root@$server_ip 'killall -9  iperf3'
   echo "IPERF server  $server_ip killed"
}

main() {
   echo "### Test Case: $test_case" | print_log result_client
   echo "### Test Description: $description" | print_log result_client

  _init

  _mesh_iperf_test

  _result

  _deinit
}

main
