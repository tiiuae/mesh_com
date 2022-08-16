#!/bin/bash -e

. /enclave/drone_device_id

export DRONE_DEVICE_ID

source /opt/ros/galactic/setup.bash

# I don't know what we're doing wrong, but Python isn't able to resolve mesh packages without this.
# (other Python packages seem to reside under /usr/lib/python3/dist-packages)
export PYTHONPATH=/opt/ros/galactic/lib/python3.8/site-packages

if [ "$1" == "init" ]; then

    echo "Start mesh executor"
    # Start mesh executor 1          2        3          4         5         6           7          8         9             10          11        12             13         14
    # Usage: mesh-ibss.sh <mode>     <ip>     <mask>     <AP MAC>  <key>     <essid>     <freq>     <txpower> <country>     <interface> <phyname> <routing_algo> <mtu_size> <log_dir>
    # Env variables:      $mesh_mode $mesh_ip $mesh_mask $mesh_mac $mesh_key $mesh_essid $mesh_freq $mesh_tx  $mesh_country $mesh_iface $mesh_phy $mesh_rtalg    $mesh_mtu  $mesh_log
    # Parameters:
    #    <mode>
    #	 <ip>
    #	 <mask>
    #	 <AP MAC>
    #	 <WEP key>
    #	 <essid>
    #	 <freq>
    #	 <txpower>
    #	 <country>
    #	 <interface> - optional
    #	 <phyname> - optional
    #	 <routing_algo> - optional
    #	 <mtu_dir>   - optional
    #	 <log_dir>   - optional
    #
    # example:
    #     mesh-ibss.sh mesh 192.168.1.2 255.255.255.0 00:11:22:33:44:55 1234567890 mymesh 5220 30 fi wlan1 phy1
    #     sudo mesh-ibss.sh ap

    #starting Default mesh
    /opt/ros/galactic/share/bin/mesh-ibss.sh $mesh_mode $mesh_ip $mesh_mask $mesh_mac $mesh_key $mesh_essid $mesh_freq $mesh_tx $mesh_country
    /opt/ros/galactic/lib/mesh_com/mesh_executor

else
    echo "Start mesh pub&sub"

    mkdir -p ~/.ros/log

    # Start mesh publisher
    ros2 run mesh_com mesh_publisher --ros-args -r __ns:=/$DRONE_DEVICE_ID &
    # Start mesh subscriber
    ros2 run mesh_com mesh_subscriber --ros-args -r __ns:=/$DRONE_DEVICE_ID

fi
