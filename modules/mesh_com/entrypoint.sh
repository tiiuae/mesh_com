#!/bin/bash

source /opt/ros/galactic/setup.bash

# Start mesh publisher
ros2 run mesh_com mesh_publisher --ros-args -r __ns:=/$DRONE_DEVICE_ID &

# Start mesh subscriber
ros2 run mesh_com mesh_subscriber --ros-args -r __ns:=/$DRONE_DEVICE_ID &
