#!/bin/bash -e

. /enclave/drone_device_id

export DRONE_DEVICE_ID

source /opt/ros/galactic/setup.bash

# I don't know what we're doing wrong, but Python isn't able to resolve mesh packages without this.
# (other Python packages seem to reside under /usr/lib/python3/dist-packages)
export PYTHONPATH=/opt/ros/galactic/lib/python3.8/site-packages

if [ "$1" == "init" ]; then

    echo "Start mesh executor"
    # Start mesh executor
    /opt/ros/galactic/share/bin/mesh-ibss.sh ap
    /opt/ros/galactic/lib/mesh_com/mesh_executor

else
    echo "Start mesh pub&sub"

    mkdir -p ~/.ros/log

    # Start mesh publisher
    ros2 run mesh_com mesh_publisher --ros-args -r __ns:=/$DRONE_DEVICE_ID &
    # Start mesh subscriber
    ros2 run mesh_com mesh_subscriber --ros-args -r __ns:=/$DRONE_DEVICE_ID

fi
