#!/bin/bash

tc_1() {
  echo "TC 1: Test if MPTCP protocol enabled in the HOS"
  if sysctl net.mptcp.enabled; then
    echo "result= PASS";
  else
    echo "result= FAIL";
  fi
  return
}
tc_2() {
  echo "TC 2: Test if mptcpize tool installed in the HOS"
  if command -v mptcpize > /dev/null 2>&1; then
      echo "result = PASS";
    else
      echo "result= FAIL";
  fi
  return
}
tc_3() {
  echo "TC 3: Test if shadowsocks installed in the HOS"
  if command -v ss-server > /dev/null 2>&1; then
      echo "result = PASS";
    else
      echo "result= FAIL";
  fi
  return
}
tc_4() {
  echo "TC 4: Test if required version of iperf3 installed in the HOS"
  if command iperf3 -v | grep "3.9"; then
      echo "result = PASS";
    else
      echo "result= FAIL";
  fi
  return
}
tc_5() {
  echo "TC 5: Test if required version of ip tool installed in the HOS"
  if command ip -V | grep "6.3.0"; then
      echo "result = PASS";
    else
      echo "result= FAIL";
  fi
  return
}
tc_6() {
  echo "TC 6: Test MPTCP throughput aggregation"
  echo "Its a manual test and follow the steps below:"
  echo "1. Start iperf server on the other mesh device by executing 'mptcpize run iperf3 -s' "
  echo "2. Monitor MPTCP related message exchange by executing 'ip mptcp monitor &' on server/client device"
  echo "3. Start ifstat at the background to monitor simultaneous transmission over multiple interfaces by executing 'while true; do ifstat | grep -E '(Interface|br-lan|bat1)'; sleep 1; done;'"
  echo "4. Start iperf client on the other mesh device by executing 'mptcpize run iperf3 -c <ip address of server> -t 60'"
  echo "5. If you observe aggregated throughput from ifstat, the test passed"
  return
}
tc_7() {
  echo "TC7 : Test MPTCP resiliency"
  echo "Its a manual test and follow the steps below:"
  echo "1. Start iperf server on the other mesh device by executing 'mptcpize run iperf3 -s' "
  echo "2. Monitor MPTCP related message exchange by executing 'ip mptcp monitor &' on server/client device"
  echo "3. Start ifstat at the background to monitor simultaneous transmission over multiple interfaces by executing 'while true; do ifstat | grep -E '(Interface|br-lan|bat1)'; sleep 1; done;'"
  echo "4. Start iperf client on the other mesh device by executing 'mptcpize run iperf3 -c <ip address of server> -t 60'"
  echo "5. In order to check resiliency put down one of the interfaces (ifconfig br-lan/bat1 down)"
  echo "6. If you observe throughput over iperf without any drop, the test passed"
  return
}
tc_8() {
  echo "TC8 : Test MPTCP Proxy Converter for TCP"
  echo "Its a manual test and follow the steps below:"
  echo "1. Connect two laptops via ethernet to two mesh devices respectively "
  echo "2. Start iperf server on one laptop - 'iperf3 -s'"
  echo "3. Monitor MPTCP related message exchange by executing 'ip mptcp monitor &' on one of the mesh device"
  echo "4. Start ifstat at the background to monitor simultaneous transmission over multiple interfaces by executing 'while true; do ifstat | grep -E '(Interface|br-lan|bat1)'; sleep 1; done;'"
  echo "5. Start iperf client on the other laptop by executing 'iperf3 -c <ip address of server laptop> -t 60'"
  echo "6. TCP redirected over MPTCP radio links can be observed over 'ip mptcp monitor' and 'ifstat' - if yes test passed"
  return
}
tc_9() {
  echo "TC9 : Test MPTCP Proxy Converter for UDP"
  echo "Its a manual test and follow the steps below:"
  echo "1. Connect two laptops via ethernet to two mesh devices respectively "
  echo "2. Start iperf server on one laptop - 'iperf3 -s'"
  echo "3. Start iperf client to generate UDP traffic from the other laptop by executing 'iperf3 -c <ip address of server laptop> -t 60 -u'"
  echo "5. UDP traffic should bypass MPTCP proxy and reach the server laptop' - if yes test passed"
  return
}



#######################################
# main
# Globals:
#  result
# Arguments:
#######################################
main() {

echo "Testing MPTCP Feature"
echo "Select Test Case (Usage: TC1)"
echo "TC1 : Test if MPTCP protocol enabled in the HOS"
echo "TC2 : Test if mptcpize tool installed in the HOS"
echo "TC3 : Test if shadowsocks-libev tool installed in the HOS"
echo "TC4 : Test if required version of iperf3 installed in the HOS"
echo "TC5 : Test if required version of ip tool installed in the HOS"
echo "TC6 : Test MPTCP throughput aggregation"
echo "TC7 : Test MPTCP resiliency"
echo "TC8 : Test MPTCP Proxy Converter for TCP"
echo "TC9 : Test MPTCP Proxy Converter for UDP"
echo "all: Execute all test cases (TC1 to TC5)"
read input

case "$input" in
    TC1|all)
    tc_1
    ;;&
    TC2|all)
    tc_2
    ;;&
    TC3|all)
    tc_3
    ;;&
    TC4|all)
    tc_4
    ;;&
    TC5|all)
    tc_5
    ;;&
    TC6)
    tc_6
    ;;
    TC7)
    tc_7
    ;;
    TC8)
    tc_8
    ;;
    TC9)
    tc_9
    ;;

esac
}

main "$@"