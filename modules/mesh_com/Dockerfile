# Given dynamically from CI job.
FROM --platform=${BUILDPLATFORM:-linux/amd64} ghcr.io/tiiuae/fog-ros-sdk:v3.2.0-${TARGETARCH:-amd64} AS builder

# Must be defined another time after "FROM" keyword.
ARG TARGETARCH

# Needed for apt to authenticate with the custom private repo
ARG ARTIFACTORY_CLOUD_TOKEN
ENV ARTIFACTORY_CLOUD_TOKEN=${ARTIFACTORY_CLOUD_TOKEN}

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
    && apt install -y --no-install-recommends squid-openssl \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*  \
    && mkdir -p /etc/squid/ssl_cert \
    && openssl req -new -newkey rsa:2048 -sha256 -days 365 -nodes -x509 -extensions v3_ca -keyout /etc/squid/ssl_cert/myCA.pem -out /etc/squid/ssl_cert/myCA.pem -batch \
    && openssl x509 -in /etc/squid/ssl_cert/myCA.pem -outform PEM -out /usr/local/share/ca-certificates/squid.crt \
    && update-ca-certificates \
    && mkdir -p /var/lib/squid \
    && /usr/lib/squid/security_file_certgen -c -s /var/lib/squid/ssl_db -M 4MB

COPY common/tools/squid/ /etc/squid/

# Squid proxy needed to add Authorization: Bearer <token> header for apt to authenticate with priv repo
RUN echo "deb [trusted=yes] https://artifactory.ssrcdevops.tii.ae/artifactory/debian-public-local focal fog-sw" >> /etc/apt/sources.list \
    && sed -i "s/<token>/$ARTIFACTORY_CLOUD_TOKEN/" /etc/squid/squid.conf \
    && squid \
    && apt -o "acquire::http::proxy=http://127.0.0.1:3128" update \
    && apt -o "acquire::http::proxy=http://127.0.0.1:3128" install -y --no-install-recommends \
        alfred \
        batctl \
        iproute2 \
        iw \
        net-tools \
        libnl \
        pcsc-lite \
        rfkill \
        wpa-supplicant=2.9-r0 \
    && pkill squid \
    && apt clean \
    && rm /etc/squid/squid.conf \
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
