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



start_olsrd() {
  INTERFACE="$1"
  PID_FILE="/run/olsrd-$INTERFACE.pid"
  CONFIG_FILE="/etc/olsrd/$INTERFACE.conf"

  echo "Testing in OLSR"
  
  cat > "$CONFIG_FILE" <<- EOF
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

  # Use start-stop-daemon to start the OLSR daemon
  start-stop-daemon --start --background --make-pidfile --pidfile "$PID_FILE" \
    --exec /usr/sbin/qos-olsrd -- -i "$INTERFACE" -d 0 -f "$CONFIG_FILE"
}


stop_olsrd() {
  INTERFACE="$1"
  PID_FILE="/run/olsrd-$INTERFACE.pid"

  if [ -e "$PID_FILE" ]; then
    echo "Stopping OLSR daemon for $INTERFACE using start-stop-daemon"
    start-stop-daemon --stop --pidfile "$PID_FILE"
    rm -f "$PID_FILE"  # Remove the PID file after stopping the daemon
  else
    echo "PID file $PID_FILE not found. OLSR daemon may not be running."
  fi
}

