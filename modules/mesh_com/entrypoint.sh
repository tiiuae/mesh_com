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
    # Usage: mesh-ibss.sh <mode> <ip> <mask> <AP MAC> <key> <essid> <freq> <txpower> <country> <interface> <phyname> <routing_algo> <mtu_size> <log_dir>
    # 
    # Number:      Parameter:                    Environment Variable:
    # 1            <mode>                        $DEFAUTLT_MESH_MODE
    # 2            <ip>                          $DEFAUTLT_MESH_IP
    # 3            <mask>                        $DEFAUTLT_MESH_MASK
    # 4            <AP MAC>                      $DEFAUTLT_MESH_MAC
    # 5            <WEP key>                     $DEFAUTLT_MESH_KEY
    # 6            <essid>                       $DEFAUTLT_MESH_ESSID
    # 7            <freq>                        $DEFAUTLT_MESH_FREQ
    # 8	           <txpower>                     $DEFAUTLT_MESH_TX
    # 9	           <country>                     $DEFAUTLT_MESH_COUNTRY
    # 10           <interface> - optional        $DEFAUTLT_MESH_IFACE
    # 11           <phyname> - optional          $DEFAUTLT_MESH_PHY
    # 12           <routing_algo> - optional     $DEFAUTLT_MESH_RTALG
    # 13           <mtu_dir>   - optional        $DEFAUTLT_MESH_MTU
    # 14           <log_dir>   - optional        $DEFAUTLT_MESH_LOG
    #
    # example:
    #     mesh-ibss.sh mesh 192.168.1.2 255.255.255.0 00:11:22:33:44:55 1234567890 mymesh 5220 30 fi wlan1 phy1
    #     mesh-ibss.sh ap

    #starting Default mesh
    /opt/ros/galactic/share/bin/mesh-ibss.sh $DEFAUTLT_MESH_MODE $DEFAUTLT_MESH_IP $DEFAUTLT_MESH_MASK $DEFAUTLT_MESH_MAC $DEFAUTLT_MESH_KEY $DEFAUTLT_MESH_ESSID $DEFAUTLT_MESH_FREQ $DEFAUTLT_MESH_TX $DEFAUTLT_MESH_COUNTRY
    /opt/ros/galactic/lib/mesh_com/mesh_executor

else
    echo "Start mesh pub&sub"

    mkdir -p ~/.ros/log

    # Start mesh publisher
    ros2 run mesh_com mesh_publisher --ros-args -r __ns:=/$DRONE_DEVICE_ID &
    # Start mesh subscriber
    ros2 run mesh_com mesh_subscriber --ros-args -r __ns:=/$DRONE_DEVICE_ID

fi
