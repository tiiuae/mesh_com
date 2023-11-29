#!/bin/bash

get_ipv6_address() {
    local interface="$1"

    # Use ifconfig to get the IPv6 address of the specified interface
    local ipv6_address=$(ifconfig "$interface" | grep 'inet6' | awk '{print $2}')

    # Print the IPv6 address
    echo "IPv6 address of $interface: $ipv6_address"


# Example usage
desired_interface=$1
get_ipv6_address "$desired_interface"
}


start_olsrd()
{
  INTERFACE="$1"
  echo "testing in olsr"
  cat > /etc/olsrd/$INTERFACE.conf <<- EOF
  LinkQualityFishEye   0

  Interface "$INTERFACE"
  {
  }

  IpVersion               6
  LinkQualityFishEye      0
  LinkQualityAlgorithm "etx_ffeth_nl80211"

  LoadPlugin "/usr/lib/olsrd_arprefresh.so.0.1"  
  {
  }
  
EOF
  qos-olsrd -i "$INTERFACE" -d 0 -f /etc/olsrd/$INTERFACE.conf
  // -p /run/olsrd-$INTERFACE.pid
}

stop_olsrd()
{
  
  // killall olsrd 2>/dev/null	
}


