#!/bin/bash -e

source /opt/ros/${ROS_DISTRO}/setup.bash

# I don't know what we're doing wrong, but Python isn't able to resolve mesh packages without this.
# (other Python packages seem to reside under /usr/lib/python3/dist-packages)
export PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.8/site-packages

# Needed in order to make ROS2 nodes exit gracefully.
# SIGTERM signal is converted to SIGINT.
_term() {
    # FILL UP PROCESS SEARCH PATTERN HERE TO FIND PROPER PROCESS FOR SIGINT:
    pattern="mesh_com/mesh_publisher"
    pid_value="$(ps -ax | grep $pattern | grep -v grep | awk '{ print $1 }')"
    if [ "$pid_value" != "" ]; then
        pid=$pid_value
        echo "Send SIGINT to pid $pid"
    else
        pid=1
        echo "Pattern not found, send SIGINT to pid $pid"
    fi
    kill -s SIGINT $pid

    pattern="mesh_com/mesh_subscriber"
    pid_value="$(ps -ax | grep $pattern | grep -v grep | awk '{ print $1 }')"
    if [ "$pid_value" != "" ]; then
        pid=$pid_value
        echo "Send SIGINT to pid $pid"
    else
        pid=1
        echo "Pattern not found, send SIGINT to pid $pid"
    fi
    kill -s SIGINT $pid
}
# Use SIGTERM or TERM, does not seem to make any difference.
trap _term TERM

if [ "$1" == "init" ]; then
    echo "Start mesh executor"

    # Enable IPv6. Required to run Alfred.
    sysctl -w net.ipv6.conf.all.disable_ipv6=0
    sysctl -w net.ipv6.conf.default.disable_ipv6=0

    # Check if mesh is in "default" mode
    if [[ "$MESH_ESSID" == *default* ]]; then
        echo "Proceeding to DEFAULT mesh..."
        # Call default_mesh.sh
        chmod 755 /opt/ros/${ROS_DISTRO}/share/bin/default-mesh.sh
        /opt/ros/${ROS_DISTRO}/share/bin/default-mesh.sh

    else
        echo "Mesh is not DEFAULT"
    fi


    # Start mesh executor 
    #                     1      2    3      4        5     6       7      8         9         10          11        12             13         14
    # Usage: mesh-11s.sh <mode> <ip> <mask> <AP MAC> <key> <essid> <freq> <txpower> <country> <interface> <phyname> <routing_algo> <mtu_size> <log_dir>
    # 
    # Number:      Parameter:                    Environment Variable:
    # 1            <mode>                        $MESH_MODE
    # 2            <ip>                          $MESH_IP
    # 3            <mask>                        $MESH_MASK
    # 4            <AP MAC>                      $MESH_MAC
    # 5            <key>                         $MESH_KEY
    # 6            <essid>                       $MESH_ESSID
    # 7            <freq>                        $MESH_FREQ
    # 8	           <txpower>                     $MESH_TX
    # 9	           <country>                     $MESH_COUNTRY
    # 10           <interface> - optional        $MESH_IFACE
    # 11           <phyname> - optional          $MESH_PHY
    # 12           <routing_algo> - optional     $MESH_RTALG
    # 13           <mtu_dir>   - optional        $MESH_MTU
    # 14           <log_dir>   - optional        $MESH_LOG
    #
    # example:
    #     mesh-11s.sh mesh 192.168.1.2 255.255.255.0 00:11:22:33:44:55 1234567890 mymesh 5220 30 fi wlan1 phy1
    #     mesh-11s.sh ap

    #starting Default mesh
    # /opt/ros/${ROS_DISTRO}/share/bin/mesh-11s.sh $MESH_MODE $MESH_IP $MESH_MASK $MESH_MAC $MESH_KEY $MESH_ESSID $MESH_FREQ $MESH_TX $MESH_COUNTRY
    # gateway_ip=$(python3 /usr/bin/default_mesh_router_select.py)
    # route add default gw $gateway_ip bat0
    # sleep 86400
else
    echo "INFO: Start mesh pub&sub"

    mkdir -p ~/.ros/log

    # Start mesh publisher
    ros-with-env ros2 run mesh_com mesh_publisher --ros-args -r __ns:=/$DRONE_DEVICE_ID &
    pub_child=$!
    # Quick fix. Give time to the publisher to stop using the keys.
    sleep 4
    # Start mesh subscriber
    ros-with-env ros2 run mesh_com mesh_subscriber --ros-args -r __ns:=/$DRONE_DEVICE_ID &
    sub_child=$!
    echo "INFO: Waiting for publisher pid $pub_child and subscriber pid $sub_child."
    # * Calling "wait" will then wait for the job with the specified by $child to finish, or for any signals to be fired.
    #   Due to "or for any signals to be fired", "wait" will also handle SIGTERM and it will shutdown before
    #   the node ends gracefully.
    #   The solution is to add a second "wait" call and remove the trap between the two calls.
    # * Do not use -e flag in the first wait call because wait will exit with error after catching SIGTERM.
    set +e
    wait $sub_child
    trap - TERM
    wait $sub_child
    wait $pub_child
    RESULT=$?
    set -e

    if [ $RESULT -ne 0 ]; then
        echo "ERROR: Mesh pub&sub node failed with code $RESULT" >&2
        exit $RESULT
    else
        echo "INFO: Mesh pub&sub node finished successfully, but returning 125 code for docker to restart properly." >&2
        exit 125
    fi

fi
