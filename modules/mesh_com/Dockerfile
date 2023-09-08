FROM ghcr.io/tiiuae/fog-ros-baseimage-builder:sha-5d4476d AS builder

# Install build dependencies
RUN apt update -y && apt install -y --no-install-recommends \
    curl \
    python3-pytest \
    ament-flake8 \
    ament-pep257 \
    && rm -rf /var/lib/apt/lists/*

# Build mesh_com
COPY . "$SRC_DIR"/mesh_com

# this:
# 1) builds the application
# 2) packages the application as .deb in /main_ws/
RUN cd "$SRC_DIR/mesh_com/modules/mesh_com" && /packaging/build_colcon.sh

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:sha-5d4476d

RUN apt update -y \
    && apt install -y --no-install-recommends \
        alfred \
        batctl \
        iproute2 \
        iw \
        keepalived \
        net-tools \
        pcsc-lite \
        rfkill \
        wpa-supplicant=2.9-r0 \
        wifi-firmware \
    && rm -rf /var/lib/apt/lists/*

ENTRYPOINT [ "ros-with-env", "/entrypoint.sh" ]

COPY modules/mesh_com/entrypoint.sh /entrypoint.sh

WORKDIR $WORKSPACE_DIR
COPY --from=builder /main_ws/src/mesh_com/modules/install $INSTALL_DIR

# Replicate the ROS2 directory structure to so scripts do not need to be refactored.
COPY common/scripts/mesh-11s.sh /opt/ros/humble/share/bin/mesh-11s.sh
COPY common/scripts/mesh-ibss.sh /opt/ros/humble/share/bin/mesh-ibss.sh
