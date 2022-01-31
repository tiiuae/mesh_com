#! /bin/bash
if [ -f /etc/os-release ]; then
    . /etc/os-release
    OS=$NAME
    VER=$VERSION_ID
elif [ -f /etc/lsb-release ]; then
    . /etc/lsb-release
    OS=$DISTRIB_ID
    VER=$DISTRIB_RELEASE
fi
if [ "$OS" == "Ubuntu" ]; then
    docker build -t comms_vm .
    docker run -it --privileged --net="host" --rm comms_vm /bin/bash
elif [ "$OS" == "Buildroot" ]; then
    # change rootfs location once its mounted in dedicated partation
    docker import - comms_vm < /rootfs.tar
    docker run --env EXECUTION_CTX='docker' -it --privileged --net="host" -v /opt:/opt comms_vm /bin/bash
fi
