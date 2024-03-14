# Given dynamically from CI job.
FROM --platform=${BUILDPLATFORM:-linux/amd64} ghcr.io/tiiuae/fog-ros-sdk:v3.2.0-${TARGETARCH:-amd64} AS builder

# Must be defined another time after "FROM" keyword.
ARG TARGETARCH

# SRC_DIR environment variable is defined in the fog-ros-sdk image.
# The same workspace path is used by all ROS2 components.
# See: https://github.com/tiiuae/fog-ros-baseimage/blob/main/Dockerfile.sdk_builder
COPY . $SRC_DIR/mesh_com

# Tar directories so they are easier to handle when doing installation.
RUN /packaging/build_colcon_sdk.sh ${TARGETARCH:-amd64}
# Even though it is possible to tar the install directory for retrieving it later in runtime image,
# the tar extraction in arm64 emulated on arm64 is still slow. So, we copy the install directory instead

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:v3.2.0

ENTRYPOINT [ "/entrypoint.sh" ]

RUN apt update \
    && apt install -y --no-install-recommends \
        alfred \
        batctl \
        iproute2 \
        iw \
        net-tools \
        libnl \
        pcsc-lite \
        rfkill \
        wpa-supplicant=2.9-r0 \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

COPY modules/mesh_com/entrypoint.sh /entrypoint.sh

# Replicate the ROS2 directory structure to so scripts do not need to be refactored.
RUN mkdir -p /opt/ros/humble/share/bin/

COPY common/scripts/ /opt/ros/humble/share/bin/

# WORKSPACE_DIR environment variable is defined in the fog-ros-baseimage.
# The same installation directory is used by all ROS2 components.
# See: https://github.com/tiiuae/fog-ros-baseimage/blob/main/Dockerfile
WORKDIR $WORKSPACE_DIR

COPY --from=builder $WORKSPACE_DIR/install install

# Colcon places mesh_com scripts on /main_ws and creating link here so that entrypoint.sh finds it
RUN ln -sf /main_ws/install/lib/ /opt/ros/humble/lib
