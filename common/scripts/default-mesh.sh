#!/bin/bash -e

# Set IP based on drone role/type.
if [ "$DRONE_TYPE" == "recon" ]; then
    # 192.168.240.1-192.168.246.254
    DEFAULT_MESH_IP="192.168.$[ $RANDOM % 7 + 240 ].$[ $RANDOM % 254 + 1 ]"
    /opt/ros/${ROS_DISTRO}/share/bin/mesh-11s.sh $MESH_MODE $MESH_IP $MESH_MASK $MESH_MAC $MESH_KEY $MESH_ESSID $MESH_FREQ $MESH_TX $MESH_COUNTRY
    echo "mesh setup done"
    gateway_ip="192.168.247.10" # FIXME: hardcoded for now. later detect automatically.
    route add default gw $gateway_ip bat0
    echo "INFO: Checking if drone is provisioned..."
    while true; do
        if [ "$DRONE_DEVICE_ID" = "bootstrap" ]; then
            sleep 10
        else
            echo "INFO: Drone is provisioned, continuing"
            break
        fi
    done
    # Start executor. Required to publish ROS2 topic.
    sleep 120
    echo "INFO: Starting ROS topic"
    /opt/ros/${ROS_DISTRO}/lib/mesh_com/mesh_executor &
    sleep 604800
elif [ "$DRONE_TYPE" == "groundstation" ]; then
    MESH_IP="192.168.248.1"
    /opt/ros/${ROS_DISTRO}/share/bin/mesh-11s.sh $MESH_MODE $MESH_IP $MESH_MASK $MESH_MAC $MESH_KEY $MESH_ESSID $MESH_FREQ $MESH_TX $MESH_COUNTRY
    echo "mesh setup done"
    echo "INFO: Checking if drone is provisioned..."
    while true; do
        if [ "$DRONE_DEVICE_ID" = "bootstrap" ]; then
            sleep 10
        else
            echo "INFO: Drone is provisioned, continuing"
            break
        fi
    done
    # Start executor. Required to publish ROS2 topic.
    sleep 120
    echo "INFO: Starting ROS topic"
    /opt/ros/${ROS_DISTRO}/lib/mesh_com/mesh_executor &
    sleep 604800
elif [ "$DRONE_TYPE" == "fog" ]; then
    if [ "$MESH_CLASS" == "edge" ]; then
        MESH_IP="192.168.247.10"
    else
        # mesh class is gs
        MESH_IP="192.168.248.10"
    fi
    /opt/ros/${ROS_DISTRO}/share/bin/mesh-11s.sh $MESH_MODE $MESH_IP $MESH_MASK $MESH_MAC $MESH_KEY $MESH_ESSID $MESH_FREQ $MESH_TX $MESH_COUNTRY
    echo "mesh setup done"
    if [ "$MESH_CLASS" == "gs" ]; then
        gateway_ip="192.168.248.1" # FIXME: hardcoded for now. later detect automatically.
        route add default gw $gateway_ip bat0
    fi
    echo "INFO: Checking if drone is provisioned..."
    while true; do
        if [ "$DRONE_DEVICE_ID" = "bootstrap" ]; then
            sleep 10
        else
            echo "INFO: Drone is provisioned, continuing"
            break
        fi
    done
    # Start executor. Required to publish ROS2 topic.
    sleep 120
    echo "INFO: Starting ROS topic"
    /opt/ros/${ROS_DISTRO}/lib/mesh_com/mesh_executor &
    sleep 604800
elif [ "$DRONE_TYPE" == "singlemesh" ]; then
    # 192.168.248.11-192.168.248.253
    MESH_IP="192.168.248.$[ $RANDOM % 243 + 11 ]"
    /opt/ros/${ROS_DISTRO}/share/bin/mesh-11s.sh $MESH_MODE $MESH_IP $MESH_MASK $MESH_MAC $MESH_KEY $MESH_ESSID $MESH_FREQ $MESH_TX $MESH_COUNTRY
    echo "mesh setup done"
        # mesh class is gs
    gateway_ip="192.168.248.1" # FIXME: hardcoded for now. later detect automatically.
    route add default gw $gateway_ip bat0
    echo "INFO: Checking if drone is provisioned..."
    while true; do
        if [ "$DRONE_DEVICE_ID" = "bootstrap" ]; then
            sleep 10
        else
            echo "INFO: Drone is provisioned, continuing"
            break
        fi
    done
    # Start executor. Required to publish ROS2 topic.
    sleep 120
    echo "INFO: Starting ROS topic"
    /opt/ros/${ROS_DISTRO}/lib/mesh_com/mesh_executor &
    sleep 604800
else
    echo "drone type not implemented: $DRONE_TYPE"
    exit 1
fi