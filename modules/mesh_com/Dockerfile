FROM ghcr.io/tiiuae/fog-ros-baseimage-builder:dp-4266_humble_upgrade AS builder

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

FROM ghcr.io/tiiuae/fog-ros-baseimage:dp-4266_humble_upgrade

# wpasupplicant pinned as to receive SSRC version (and not Ubuntu) of it which has some mesh-related customizations:
# https://github.com/tiiuae/wpa
RUN wget https://ssrc.jfrog.io/artifactory/ssrc-deb-public-local/wpasupplicant_2.9.0-20-6~git20210701.43d8971_amd64.deb \
    && dpkg -i wpasupplicant_*.deb \
    && rm -rf wpasupplicant_*.deb
RUN apt-get update -y && apt-get install -y --no-install-recommends \
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
