#!/bin/bash

source ./common.sh

test_case="Mesh connection and Iperf/ping-N hop"

description="Able to connect and ping in N hop topology."

#######################################
# Init
# Globals:
#  result
#  device_list
# Arguments:
#  $1 = ipaddress
#  $2 = encryption
#  $3 = frequency
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
  find_wifi_device "pci" 0x168c "0x0034 0x003c 0x003e 0x0033"
  phyname=${device_list[0]}  # only first pci device is used here
  wifidev="mesh0"
  if ! iw phy $phyname interface add $wifidev type mp; then
    result=$FAIL
    return
  fi

  # setup iperf3 server
  iperf3 -s -p "$iperf3_port"&

  ifconfig "$wifidev" mtu 1560
  ip link set "$wifidev" up
  batctl if add "$wifidev"
  ifconfig bat0 up
  ifconfig bat0 "$1" netmask 255.255.255.0
  ifconfig bat0 mtu 1460

  conf_filename="./tmp/wpa_supplicant_11s_$3_$2.conf"
  log_filename="./logs/wpa_supplicant_11s_$3_$2.log"

  # Create wpa_supplicant.conf here
  create_wpa_supplicant_config "$conf_filename" "$3" "$2"

  # start wpa_supplicant
  wpa_supplicant -Dnl80211 -B -i"$wifidev" -C /var/run/wpa_supplicant/ -c "$conf_filename" -f "$log_filename"
  sleep 5
  iw dev "$wifidev" set mesh_param mesh_fwding 0
  iw dev "$wifidev" set mesh_param mesh_ttl 1

  sleep 5
  wifi_type=$(iw dev mesh0 info | grep type)
  echo "$wifi_type mode" | print_log
  case $wifi_type in
    *"managed"*)
    result=$FAIL
    ;;
    *"mesh"*)
    result=$PASS
    ;;
    *)
    result=$FAIL
    return
    ;;
  esac
}

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
#######################################
_result() {
  echo "$0, result called" | print_log

  sub_result=$PASS

  #1 analyse logs from logs/ - folder
  tcp_result=$(grep sender -B1 logs/tcp.log)
  echo -e "# TCP result:\n$tcp_result" | print_log result
  tcp_avg=$(awk -F " " '($NF=="sender") {print $7}' logs/tcp.log)

  udp_result=$(grep sender -B1 logs/udp.log)
  echo -e "# UDP result:\n$udp_result" | print_log result
  udp_avg=$(awk -F " " '($NF=="sender") {print $7}' logs/udp.log)

  ping_result=$(grep rtt -B1 logs/ping.log)
  echo -e "# ping result:\n$ping_result" | print_log result
  ping_avg=$(echo "$ping_result" | grep rtt | cut -d '/' -f 5)

	#2 make decision ($PASS or $FAIL)

  # ping limit 4.0milliseconds
  if (( $(echo "$ping_avg < 3.0" |bc -l) )); then
    result=$PASS
  else
    sub_result=$FAIL
    echo "FAILED  :in ping test" | print_log result
  fi

  if (( $(echo "$tcp_avg > 75.0" |bc -l) )); then
    result=$PASS
  else
    sub_result=$FAIL
    echo "FAILED  :in tcp test" | print_log result
  fi

  if (( $(echo "$udp_avg > 75.0" |bc -l) )); then
    result=$PASS
  else
    sub_result=$FAIL
    echo "FAILED  :in udp test" | print_log result
  fi
  result=$sub_result
}

#######################################
# DeInit
# Globals:
#  device_list
# Arguments:
#######################################
_deinit() {
  echo "$0, deinit called" | print_log

  killall iperf3
  killall wpa_supplicant
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
  frequency=0
  ipaddress="255.255.255.255"
  server_ipaddress="255.255.255.255"
  mode="NA"

  while getopts ":m:s:e:f:i:h" flag
  do
    case "${flag}" in
      e) encryption=${OPTARG};;
      f) frequency=${OPTARG};;
      i) ipaddress=${OPTARG};;
      s) server_ipaddress=${OPTARG};;
      m) mode=${OPTARG};;
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
        Usage: sudo $0 [-i ipaddress] [-e encryption] [-f freq] [-m mode] [-h]

        -i ipaddress    IP address to be used for node
        -e encryption   SAE or NONE
        -f frequency    Wi-Fi frequency in MHz
        -m mode         server or client.
                        server = test node which provides iperf3
                        client = test node which runs iperf3/ping tests against server
        -s address      server address to be used for test connection.

        For Example: sudo $0 -i 192.168.1.2 -e NONE -f 2412 -m server

        -H              Help"
        return
  fi

  echo "### Test Case: $test_case" | print_log result
  echo "### Test Description: $description" | print_log result
  echo "### Test Settings: | print_log result
        Node IP address: $ipaddress
        Iperf3 port: $iperf3_port
        Encryption: $encryption
        Wifi Frequency: $frequency
        mode: $mode" | print_log result

  if  [ "$server_ipaddress" != "255.255.255.255" ]; then
    echo "        Server IP address: $server_ipaddress" | print_log result
  fi

  _init "$ipaddress" "$encryption" "$frequency"

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

    _result

    if [ "$result" -eq "$FAIL" ]; then
      echo "FAILED  : $test_case" | print_log result
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