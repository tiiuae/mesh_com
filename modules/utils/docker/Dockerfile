FROM ubuntu

ENV TZ=Asia/Dubai
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
ARG DEBIAN_FRONTEND=noninteractive
RUN echo "resolvconf resolvconf/linkify-resolvconf boolean false" | debconf-set-selections


WORKDIR /opt
RUN apt update -y && apt install batctl net-tools sudo git nano gcc python3-dev python3-yaml iproute2 iw wireless-tools isc-dhcp-client inetutils-ping wpasupplicant lsb-release systemd iptables clang libssl-dev -y
RUN git clone https://github.com/tiiuae/mesh_com.git /opt/mesh_com
RUN cp /opt/mesh_com/modules/sc-mesh-secure-deployment/install.sh /opt/install.sh
RUN rm -r /opt/mesh_com
RUN sed -i "95s+--recursive+common/security/cryptolib/+" /opt/install.sh
RUN /bin/bash /opt/install.sh
WORKDIR /opt/mesh_com/
RUN git checkout feature/docker
RUN git submodule update --init common/security/cryptolib
WORKDIR /opt/mesh_com/modules/sc-mesh-secure-deployment/
RUN pip3 install -r requirements/server-requirements.txt
RUN pip3 install -r requirements/client-requirements.txt
RUN cd ../../common/security/cryptolib/; make ; cd -
