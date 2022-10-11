#!/bin/bash -e

# . /enclave/drone_device_id

# export DRONE_DEVICE_ID

source /opt/ros/galactic/setup.bash

# I don't know what we're doing wrong, but Python isn't able to resolve mesh packages without this.
# (other Python packages seem to reside under /usr/lib/python3/dist-packages)
export PYTHONPATH=/opt/ros/galactic/lib/python3.8/site-packages

if [ "$1" == "init" ]; then

    echo "Start mesh executor"
    # Start mesh executor 
    #                     1      2    3      4        5     6       7      8         9         10          11        12             13         14
    # Usage: mesh-11s.sh <mode> <ip> <mask> <AP MAC> <key> <essid> <freq> <txpower> <country> <interface> <phyname> <routing_algo> <mtu_size> <log_dir>
    # 
    # Number:      Parameter:                    Environment Variable:
    # 1            <mode>                        $DEFAULT_MESH_MODE
    # 2            <ip>                          $DEFAULT_MESH_IP
    # 3            <mask>                        $DEFAULT_MESH_MASK
    # 4            <AP MAC>                      $DEFAULT_MESH_MAC
    # 5            <WEP key>                     $DEFAULT_MESH_KEY
    # 6            <essid>                       $DEFAULT_MESH_ESSID
    # 7            <freq>                        $DEFAULT_MESH_FREQ
    # 8	           <txpower>                     $DEFAULT_MESH_TX
    # 9	           <country>                     $DEFAULT_MESH_COUNTRY
    # 10           <interface> - optional        $DEFAULT_MESH_IFACE
    # 11           <phyname> - optional          $DEFAULT_MESH_PHY
    # 12           <routing_algo> - optional     $DEFAULT_MESH_RTALG
    # 13           <mtu_dir>   - optional        $DEFAULT_MESH_MTU
    # 14           <log_dir>   - optional        $DEFAULT_MESH_LOG
    #
    # example:
    #     mesh-11s.sh mesh 192.168.1.2 255.255.255.0 00:11:22:33:44:55 1234567890 mymesh 5220 30 fi wlan1 phy1
    #     mesh-11s.sh ap

    #starting Default mesh
    /opt/ros/galactic/share/bin/mesh-11s.sh $DEFAULT_MESH_MODE $DEFAULT_MESH_IP $DEFAULT_MESH_MASK $DEFAULT_MESH_MAC $DEFAULT_MESH_KEY $DEFAULT_MESH_ESSID $DEFAULT_MESH_FREQ $DEFAULT_MESH_TX $DEFAULT_MESH_COUNTRY
    /opt/ros/galactic/lib/mesh_com/mesh_executor
    # flawed injection: sleep 86400
else
    echo "Start mesh pub&sub"

    mkdir -p ~/.ros/log

    # Start mesh publisher
    ros2 run mesh_com mesh_publisher --ros-args -r __ns:=/$DRONE_DEVICE_ID &
    # Start mesh subscriber
    ros2 run mesh_com mesh_subscriber --ros-args -r __ns:=/$DRONE_DEVICE_ID

fi
