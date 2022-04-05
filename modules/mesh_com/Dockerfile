FROM ghcr.io/tiiuae/fog-ros-baseimage:builder-3dcb78d AS builder

# Install build dependencies
RUN apt-get update -y && apt-get install -y --no-install-recommends \
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
WORKDIR /mesh_com/src
ADD . /mesh_com/src

COPY modules/mesh_com/packaging/rosdep.yaml* modules/mesh_com/packaging/rosdep.sh modules/mesh_com/packaging/build_deps.sh modules/mesh_com/packaging/
COPY modules/mesh_com/underlay.repos modules/mesh_com/package.xml modules/mesh_com/packaging/

RUN modules/mesh_com/packaging/rosdep.sh modules/mesh_com

RUN rosdep update

RUN modules/mesh_com/packaging/build_deps.sh modules/mesh_com

RUN modules/mesh_com/package.sh 0 focal

FROM ghcr.io/tiiuae/fog-ros-baseimage:sha-3dcb78d

# wpasupplicant pinned as to receive SSRC version (and not Ubuntu) of it which has some mesh-related customizations:
# https://github.com/tiiuae/wpa
RUN apt-get update -y && apt-get install -y --no-install-recommends \
    wpasupplicant=2.9.0-20-6~git20210701.43d8971 \
    wifi-firmware \
    iw \
    batctl \
    alfred \
    rfkill \
    iproute2 \
    && rm -rf /var/lib/apt/lists/*

ENTRYPOINT [ "/entrypoint.sh" ]

COPY modules/mesh_com/entrypoint.sh /entrypoint.sh

COPY --from=builder /main_ws/src/modules/ros-*-mesh-com_*_amd64.deb /mesh-com.deb

RUN dpkg -i /mesh-com.deb && rm /mesh-com.deb
