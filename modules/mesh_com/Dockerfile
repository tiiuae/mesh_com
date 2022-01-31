# mesh_com BUILDER
FROM ros:galactic-ros-base as builder

# Install build dependencies
RUN apt-get update -y && apt-get install -y --no-install-recommends \
    curl \
    python3-bloom \
    fakeroot \
    dh-make \
    dh-python \
    python3-pytest \
    ros-galactic-ament-flake8 \
    ros-galactic-ament-pep257 \
    && rm -rf /var/lib/apt/lists/*

# Build mesh_com
WORKDIR /mesh_com/src
ADD . /mesh_com/src
RUN modules/mesh_com/package.sh 0 focal



# tii-mesh-com image
FROM ros:galactic-ros-core

RUN echo "deb [trusted=yes] https://ssrc.jfrog.io/artifactory/ssrc-debian-public-remote focal fog-sw" >> /etc/apt/sources.list

RUN apt-get update -y && apt-get install -y --no-install-recommends \
    wpasupplicant \
    wifi-firmware \
    iw \
    batctl \
    alfred \
    && rm -rf /var/lib/apt/lists/*

ENV PACKAGE_NAME mesh_com
ENTRYPOINT ["/mesh_com/entrypoint.sh"]

COPY --from=builder /mesh_com/src/modules/mesh_com/entrypoint.sh /mesh_com/entrypoint.sh

# Install mesh_com
WORKDIR /mesh_com
COPY --from=builder /mesh_com/src/modules/*.deb .
RUN dpkg -i *.deb && rm -f *.deb
