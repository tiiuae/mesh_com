#!/bin/bash

source ./common/common.sh   # common tools
source ./common/init.sh     # network init
source ./common/deinit.sh   # network de-init

test_case="Mesh connection and Iperf/ping-N hop"

description="Able to connect and ping in N hop topology."

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

  rm -f ${log_dir}tcp.log
  exe "iperf3 -f m  -c $1 -i 1 -t 10 -p $iperf3_port --logfile ${log_dir}tcp.log"
  ret=$?
  if [ "$ret" != 0 ]; then
    result=$FAIL
    echo "Command fails: iperf3" | print_log result
    return
  fi

  rm -f ${log_dir}udp.log
  exe "iperf3 -u -f m -c $1 -i 1 -t 10 -p $iperf3_port -b 500M --logfile ${log_dir}udp.log"
  ret=$?
  if [ "$ret" != 0 ]; then
    result=$FAIL
    echo "Command fails: iperf3" | print_log result
    return
  fi

  ping "$1" -c 10 > ${log_dir}ping.log  # exe can't be used
  ret=$?
  if [ "$ret" != 0 ]; then
    result=$FAIL
    echo "Command fails: ping" | print_log result
    return
  fi
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

  # analyse TCP log file
  if ! grep "connection refused" logs/tcp.log; then
    tcp_result=$(grep sender -B1 logs/tcp.log)
    echo -e "# TCP result:\n$tcp_result" | print_log result
    tcp_avg=$(awk -F " " '($NF=="sender") {print $7}' logs/tcp.log)
  else
    echo "iperf3 tcp connection refused" | print_log result
    tcp_avg="0"
  fi

  # analyse UDP log file
  if ! grep "connection refused" logs/udp.log; then
    udp_result=$(grep sender -B1 logs/udp.log)
    echo -e "# UDP result:\n$udp_result" | print_log result
    udp_avg=$(awk -F " " '($NF=="sender") {print $7}' logs/udp.log)
  else
    echo "iperf3 udp connection refused" | print_log result
    udp_avg="0"
  fi

  # analyse ping results
  ping_result=$(grep -e rtt -e round-trip -B1 logs/ping.log)
  echo -e "# ping result:\n$ping_result" | print_log result
  ping_avg=$(echo "$ping_result" | grep -e rtt -e round-trip | cut -d ' ' -f 4 | cut -d '/' -f 2)

  # with skip-setup phyname is unknown as _init() is not executed
  if [ "$net_setup" = "skip" ]; then
    echo "SKIPPED   : Regulatory setting test skipped as no _init()" | print_log result
  else
    # compare requested CC to set CC
    iw_reg_country=$(iw phy $phyname reg get | grep global -A1 | awk 'END{print $2}')
    iw_reg_country=${iw_reg_country::-1}
    echo -e "# Regulatory check:\nRequested=${1^^} and iw reg get=$iw_reg_country" | print_log result
    if [ "$iw_reg_country" != "${1^^}" ]; then
      result=$FAIL
      echo "FAILED  : Regulatory setting failed" | print_log result
    fi
  fi

	#2 make decision ($PASS or $FAIL)
  # ping limit 3.0milliseconds
  if (( $(echo "$ping_avg > 3.0" |bc -l) )); then
    result=$FAIL
    echo "FAILED  : in ping test" | print_log result
  fi

  if (( $(echo "$tcp_avg < 75.0" |bc -l) )); then
    result=$FAIL
    echo "FAILED  : in tcp test" | print_log result
  fi

  if (( $(echo "$udp_avg < 75.0" |bc -l) )); then
    result=$FAIL
    echo "FAILED  : in udp test" | print_log result
  fi
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

  if [ "$help_text" -eq 1 ] || \
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
                        mesh+ap   = Batman mesh node + Access-point in same channel (not implemented)
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

    _result "$country"

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
  else
    echo "###" | print_log result
    echo "###: Please start tests in Client node" | print_log result
    echo "###" | print_log result
  fi
}

main "$@"