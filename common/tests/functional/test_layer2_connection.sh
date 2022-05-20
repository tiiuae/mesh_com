#!/bin/bash

source ./common/common.sh   # common tools
source ./common/init.sh     # network init
source ./common/deinit.sh   # network de-init

test_case="batman-adv:  Layer2 Traceroute/Ping/OGM_verification"

description="""
Traceroute/ping to another device in static topology
between > 3 nodes (batctl) and verify OGM interval from tcpdump"""

# global
mac_address_list=""

#######################################
# Test
# Globals:
#  result
# Arguments:
#  $1 = server ipaddress
#######################################
_test() {
  echo "$0, test called" | print_log

  #1 make test and return with fail if command fails
  log_dir="logs/"

  # my_command executed and pass logs to logs/-folder
  wait_ip "$1"

  # collect OGM packets for orig_interval check
  (batctl td wlp1s0 -p1 |tee ${log_dir}tcp_dump_ogm.log 2>/dev/null)&

  mac_address_list=$(batctl n | awk '/\t/{print$2}')
  echo "MAC addresses found from Mesh network: $mac_address_list" | print_log result
  if [ -z "$mac_address_list" ]; then
    result=$FAIL
    echo "Command fails: mac_address_list empty" | print_log result
    return
  fi

  echo '' > ${log_dir}batctl_ping.log
  for mac in $mac_address_list; do
    echo "batctl ping $mac" | print_log
    echo "$mac" >> ${log_dir}batctl_ping.log
    batctl ping -c10 "$mac" |grep rtt >> ${log_dir}batctl_ping.log
    ret=$?
    if [ "$ret" != 0 ]; then
      result=$FAIL
      echo "Command fails: batctl ping $mac" | print_log result
    fi
  done

  echo '' > ${log_dir}batctl_traceroute.log
  for mac in $mac_address_list; do
    echo "batctl traceroute $mac" | print_log
    echo "${mac}_start" >> ${log_dir}batctl_traceroute.log
    batctl traceroute "$mac" >> ${log_dir}batctl_traceroute.log
    ret=$?
    if [ "$ret" != 0 ]; then
      result=$FAIL
      echo "Command fails: batctl traceroute $mac" | print_log result
    fi
    echo "${mac}_end" >> ${log_dir}batctl_traceroute.log
  done

  killall batctl
}

#######################################
# Check OGM interval from TCP dump
# Globals:
# Arguments:
#  $1 = from mac
#  $2 = to mac
#  $3 = orig_interval
#######################################
check_ogm_interval() {
  # variables for calculation
  count=0; first=0; sum=0

  # find time stamps from mymac to mac
  ogm_time_stamps=$(grep -e "^[0-9]" ${log_dir}tcp_dump_ogm.log |grep "${1}: OGM IV via neigh $2" | cut -d ' ' -f 1)
  for stamp in $ogm_time_stamps; do
    value=$(date --date "$stamp" +%s%N) # convert time stamp to nanoseconds
    if [ $first -eq 0 ]; then
      first=$value
    else
      # timestamp difference to previous stamp
      ((diff=value-first)); first=$value
      # calculate sum and count for average
      ((sum=sum+diff)); ((count=count+1))
    fi
  done
  ((average=(sum/count)/1000000)) # from nanos to millis
  echo "*** Measured OGM interval from $1 to $2 = $average ms (set value: $3 ms)" | print_log result
}

#######################################
# Result
# Globals:
#  result
# Arguments:
#  $1 = requested country code
#  $2 = orig_interval
#######################################
_result() {
  echo "$0, result called" | print_log

  #1 analyse logs from logs/ - folder
  log_dir="logs/"
  update_wifidev_from_batctl_if
  my_mac=$(cat /sys/class/net/"$wifidev"/address)

  # analyse ping results
  for mac in $mac_address_list; do
    echo "***" | print_log result
    ping_result=$(grep -e "$mac" -A1 ${log_dir}batctl_ping.log)
    echo "Result for $ping_result" | print_log result
    #echo -e "# ping result:\n$ping_result" | print_log result
    ping_avg=$(echo "$ping_result" | grep -e rtt | cut -d ' ' -f 4 | cut -d '/' -f 2)
    if (( $(echo "$ping_avg > 3.0" |bc -l) )) || (( $(echo "$ping_avg == 0" |bc -l) )); then
      result=$FAIL
      echo "FAILED  : in ping test $mac average $ping_avg ms" | print_log result
    else
      echo "Average ping $ping_avg" | print_log result
    fi
  done

  for mac in $mac_address_list; do
    echo "***" | print_log result
    trace_route=$(sed -n "/${mac}_start/,/${mac}_end/p" "${log_dir}batctl_traceroute.log" |grep -e "^ ")
    trace_route_hops=$(echo "$trace_route" | wc -l)
    echo "Result for route to $mac : $trace_route" | print_log result
    echo "Result for route hops: $trace_route_hops" | print_log result
    if [ "$trace_route_hops" -lt 3 ]; then
      echo "FAILED  : less than 3 hops to $mac" | print_log result
    fi
  done
  echo "***" | print_log result


  ((high_limit=$2+500))
  ((low_limit=$2-500))

  for mac in $mac_address_list; do
    average=0
    check_ogm_interval "$my_mac" "$mac" "$2"
    if [ "$average" -ge "$high_limit" ] || [ "$average" -le "$low_limit" ] ; then
      result=$FAIL
      echo "FAILED  : OGM interval verification test from $my_mac to $mac: $average ms (set value $2 )" | print_log result
    fi
    check_ogm_interval "$mac" "$my_mac" "unknown"

  done
  echo "***" | print_log result
}

#######################################
# main
# Globals:
#  result
# Arguments:
#######################################
main() {

  help_text=0
  encryption="NA"
  country="fi"
  routing_algo="BATMAN_IV"
  orig_interval="1000"
  frequency=0
  ipaddress="255.255.255.255"
  server_ipaddress="255.255.255.255"
  mode="NA"
  net_setup="mesh"

  while getopts ":m:s:r:n:o:c:e:f:i:h" flag
  do
    case "${flag}" in
      e) encryption=${OPTARG};;
      f) frequency=${OPTARG};;
      c) country=${OPTARG};;
      i) ipaddress=${OPTARG};;
      s) server_ipaddress=${OPTARG};;
      m) mode=${OPTARG};;
      r) routing_algo=${OPTARG};;
      o) orig_interval=${OPTARG};;
      n) net_setup=${OPTARG};;
      h) help_text=1;;
      *) help_text=1;;
    esac
  done


  if [ "$net_setup" = "skip" ] && \
     [ "$ipaddress" != "255.255.255.255" ]; then
    frequency="skip"
    encryption="skip"
  elif [ "$help_text" -eq 1 ] || \
     [ "$mode" = "NA" ] || \
     [ "$frequency" -eq 0 ] || \
     [ "$ipaddress" = "255.255.255.255" ] || \
     [ "$encryption" = "NA" ]; then
    echo "
        Usage: sudo $0 [-i ipaddress] [-e encryption] [-f freq] [-m mode] [-c country] [-o orig_int] [-r ra] [-h]

        -i ipaddress    IP address to be used for node
        -e encryption   SAE or NONE
        -f frequency    Wi-Fi frequency in MHz
        -c country      2-letter country code e.g. fi, ae, us..  (default:fi)
        -m mode         server or client.
                        server = test node which provides iperf3
                        client = test node which runs iperf3/ping tests against server
        -s address      server address to be used for test connection.

        -n net_setup    mesh      = normal Batman mesh node (default)
                        mesh_ap   = Batman mesh node + Access-point in same channel (not implemented)
                        mesh_vlan = Batman mesh node using VLAN (easy chain setup)
                        skip      = skip network setup and just execute tests

        Optional Batman-adv tweaks:
        -o orig_int     OGM (orig) interval tuning in milliseconds (default:1000)
        -r routing      Batman routing algorithm selection (BATMAN_IV, BATMAN_V)
                        (default:BATMAN_IV)

        For Example: sudo $0 -i 192.168.1.2 -e NONE -f 2412 -m server

        -h              Help"
        return
  fi

  echo "### Test Case: $test_case" | print_log result
  echo "### Test Description: $description" | print_log result
  echo "### Test Settings:
        Node IP address:          $ipaddress
        Iperf3 port:              $iperf3_port
        Encryption:               $encryption
        Wifi Frequency:           $frequency
        Wifi Country Regulatory:  $country
        mode:                     $mode
        Network Setup:            $net_setup
        Batman orig_interval:     $orig_interval
        Batman routing algorithm: $routing_algo" | print_log result

  if  [ "$server_ipaddress" != "255.255.255.255" ]; then
    echo "        Server IP address:        $server_ipaddress" | print_log result
  fi

  case $net_setup in
    "mesh")
    _init "$ipaddress" "$encryption" "$frequency" "$country" "$orig_interval" "$routing_algo"
    ;;
    "mesh_vlan")
    _init_vlan "$ipaddress" "$encryption" "$frequency" "$country" "$orig_interval" "$routing_algo"
    ;;
    "mesh_ap")
    _init_mesh_ap "$ipaddress" "$encryption" "$frequency" "$country" "$orig_interval" "$routing_algo"
    ;;
    "skip") # some other service is taking care of network setup
    killall iperf3 2>/dev/null
    iperf3 -s -D -p "$iperf3_port" --forceflush
    ;;
    *)
    echo "Network Setup option: $net_setup not implemented" | print_log
    ;;
  esac

  if [ "$result" -eq "$FAIL" ]; then
    echo "FAILED  _init: $test_case" | print_log result
    exit 0
  fi

  if [ "$mode" = "client" ]; then
    _test "$server_ipaddress"

    if [ "$result" -eq "$FAIL" ]; then
      echo "FAILED  _test: $test_case" | print_log result
      exit 0
    fi

    _result "$country" "$orig_interval"

    if [ "$net_setup" = "mesh_vlan" ]; then
      mesh_hop=$(("${server_ipaddress##*.}"-"${ipaddress##*.}"))
      echo "Hop count: ${mesh_hop#-}" | print_log result
    fi

    if [ "$result" -eq "$FAIL" ]; then
      echo "FAILED  : $test_case" | print_log result
      echo "        : " | print_log result
      echo "HOX!!   : Throughput/ping depends on distance and hop count. Even
        : test case is set as Failed, more analysis needed for the reason. e.g
        : if multihops were tested this can be a PASS." | print_log result
      exit 0
    else
      echo "PASSED  : $test_case" | print_log result
      exit 0
    fi
  else # server activity
    echo "###" | print_log result
    echo "###: Please start tests in Client node" | print_log result
    echo "###" | print_log result
  fi
}

main "$@"