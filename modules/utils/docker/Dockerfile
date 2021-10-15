FROM ubuntu

ENV TZ=Asia/Dubai
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
ARG DEBIAN_FRONTEND=noninteractive
RUN echo "resolvconf resolvconf/linkify-resolvconf boolean false" | debconf-set-selections


WORKDIR /opt
RUN apt update -y && apt install batctl net-tools sudo git nano gcc python3-dev python3-yaml iproute2 iw wireless-tools isc-dhcp-client inetutils-ping wpasupplicant lsb-release systemd iptables -y

RUN wget -L https://github.com/tiiuae/mesh_com/raw/develop/modules/sc-mesh-secure-deployment/install.sh
RUN mv install.sh /opt/install.sh
RUN /bin/bash /opt/install.sh
WORKDIR /opt/mesh_com/modules/sc-mesh-secure-deployment/
