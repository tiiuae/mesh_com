#!/bin/bash -e

if [ "$1" == "init" ]; then

    echo "Start mesh executor"
    # Start mesh executor
    /opt/ros/galactic/share/bin/mesh-ibss.sh ap
    /opt/ros/galactic/lib/mesh_com/mesh_executor

else

    echo "Start mesh pub&sub"

    source /opt/ros/galactic/setup.bash
    mkdir -p ~/.ros/log

    # Start mesh publisher
    ros2 run mesh_com mesh_publisher --ros-args -r __ns:=/$DRONE_DEVICE_ID &
    # Start mesh subscriber
    ros2 run mesh_com mesh_subscriber --ros-args -r __ns:=/$DRONE_DEVICE_ID

fi
