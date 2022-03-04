#!/bin/bash

source ./common/common.sh   # common tools
source ./common/init.sh     # network init
source ./common/deinit.sh   # network de-init

test_case="DFS Test"

description="""
Test CAC, Radar detection, channel switch and
mesh re-association after channel switch.
"""

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

  #1 verify that requested channel was DFS channel
  #2 Find CAC Start and wait cac period (cac period can change)
  # e.g. from wpa_log:
  #      wlp1s0: DFS-CAC-START freq=5300 chan=60 sec_chan=0, width=0, seg0=0, seg1=0, cac_time=60s
  # from (grep -e "DFS-CAC-START" $wpa_log_filename) if mesh
  # from $hostapd_log_filename if ap

  #3 check that mesh is up running after cac
  # DFS-CAC-COMPLETED success=1   or

  #4 cause radar event with iw test command?? or wait radar (fi=5300) and verify
  # DFS-RADAR-DETECTED freq=5300...

  #5 possible channel switch due to radar event --> verify
  # DFS-NEW-CHANNEL freq=5500....

  #6 connection verification after channel change
  # ping ap/sta or mesh_node

}

#######################################
# Result
# Globals:
#  result
# Arguments:
#  $1 = requested country code
#######################################
_result() {
  echo "$0, result called" | print_log

  #1 analyse logs from logs/ - folder
  log_dir="logs/"

  result=$FAIL

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