FROM ghcr.io/tiiuae/fog-ros-baseimage:sha-f8defd3

# Not really needed for now because ROS2 components are disabled.
#ENV ROS_DISTRO=humble

RUN apt update -y && apt install -y --no-install-recommends \
    alfred \
    batctl \
    keepalived \
    iproute2 \
    iw \
    net-tools \
    pcsc-lite \
    rfkill \
    wpa-supplicant=2.9-r0 && \
    rm -rf /var/lib/apt/lists/*

COPY modules/mesh_com/entrypoint.sh /entrypoint.sh
# Replicate the ROS2 directory structure to so scripts do not need to be refactored.
#RUN mkdir /opt/ros/humble/share/bin/
#COPY common/scripts/mesh-11s.sh /opt/ros/humble/share/bin/mesh-11s.sh
#COPY common/scripts/mesh-ibss.sh /opt/ros/humble/share/bin/mesh-ibss.sh

ENTRYPOINT [ "/entrypoint.sh" ]
