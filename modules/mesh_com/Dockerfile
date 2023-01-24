FROM ghcr.io/tiiuae/fog-ros-baseimage-builder:dp-4266_humble_upgrade AS builder

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

FROM ghcr.io/tiiuae/fog-ros-baseimage:dp-4266_humble_upgrade

RUN apt update -y && apt install -y --no-install-recommends \
    iw \
    batctl \
    alfred \
    rfkill \
    iproute2 && \
    rm -rf /var/lib/apt/lists/*

# Ubuntu 22.04 has upgraded libssl to 3 and does not propose libssl1.1.
# We can force the installation of libssl1.1 by adding the ubuntu 20.04 source.
RUN echo "deb http://security.ubuntu.com/ubuntu focal-security main" | tee /etc/apt/sources.list.d/focal-security.list && \
    apt update -y && \
    apt install -y --no-install-recommends \
        wget \
        libdbus-1-3 \
        libnl-genl-3-200 \
        libnl-route-3-200 \
        libpcsclite1 \
        libssl1.1 \
        # ifconfig not installed by default anymore
        net-tools && \
    rm -rf /var/lib/apt/lists/* && \
    wget \
        https://ssrc.jfrog.io/artifactory/ssrc-deb-public-local/wpasupplicant_2.9.0-20-6~git20210701.43d8971_amd64.deb \
        https://ssrc.jfrog.io/artifactory/ssrc-deb-public-local/wifi-firmware_1.0.0-5~git20210816.fecb07b_amd64.deb && \
    dpkg -i wpasupplicant_*.deb && \
    rm -rf wpasupplicant_*.deb && \
    rm -rf /etc/apt/sources.list.d/focal-security.list

ENTRYPOINT [ "/entrypoint.sh" ]

COPY modules/mesh_com/entrypoint.sh /entrypoint.sh

COPY --from=builder /main_ws/src/modules/ros-*-mesh-com_*_amd64.deb /mesh-com.deb

RUN dpkg -i /mesh-com.deb && rm /mesh-com.deb
