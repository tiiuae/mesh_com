# Given dynamically from CI job.
FROM --platform=${BUILDPLATFORM:-linux/amd64} ghcr.io/tiiuae/fog-ros-sdk:sha-6d67ecf-${TARGETARCH:-amd64} AS builder

# Install build dependencies
RUN apt update -y && apt install -y --no-install-recommends \
    curl \
    python3-bloom \
    fakeroot \
    dh-make \
    dh-python \
    python3-pytest \
    ros-${ROS_DISTRO}-ament-flake8 \
    ros-${ROS_DISTRO}-ament-pep257 \
    && rm -rf /var/lib/apt/lists/*

# Build mesh_com
COPY . /main_ws/src/

# this:
# 1) builds the application
# 2) packages the application as .deb in /main_ws/
RUN /packaging/build.sh

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:sha-6d67ecf

ENTRYPOINT [ "/entrypoint.sh" ]

COPY modules/mesh_com/entrypoint.sh /entrypoint.sh

COPY --from=builder /main_ws/src/modules/ros-*-mesh-com_*_amd64.deb /mesh-com.deb

COPY common/scripts/mesh-11s.sh /opt/ros/humble/share/bin/mesh-11s.sh

# WORKSPACE_DIR environment variable is defined in the fog-ros-baseimage.
# The same installation directory is used by all ROS2 components.
# See: https://github.com/tiiuae/fog-ros-baseimage/blob/main/Dockerfile
WORKDIR $WORKSPACE_DIR

COPY --from=builder $WORKSPACE_DIR/install install

# Colcon places mesh_com scripts on /main_ws and creating link here so that entrypoint.sh finds it
RUN ln -sf /main_ws/install/lib/ /opt/ros/humble/lib
