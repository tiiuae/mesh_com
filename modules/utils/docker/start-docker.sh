#! /bin/bash
function help
{
    echo
    echo "Usage: sudo ./start-docker.sh <docker_env"
    echo "Parameters:"
    echo "      <docker_env>"
    echo "example:"
    echo "sudo ./start-docker.sh ubuntu/sec_os"
    exit
}

echo "sudo  start-docker.sh $1"
    if [[ -z "$1" ]]; then
        echo "check arguments..."
        help
    fi

docker_exec_env=$1

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
    if [ "$docker_exec_env" == "sec_os" ]; then
        if [ ! -f "/opt/container-data/mesh" ]; then
            # create persist mesh container data partation
            # data/container-data/mesh
            # ├── state.json
            # └── wpa_supplicant_11s.conf
            # └── wpa_supplicant_sta.conf
            # └── wpa_supplicant_ap.conf
            mkdir -p /opt/container-data/mesh
        fi
        # change rootfs location once its mounted in dedicated partation
        docker import - comms_vm < /rootfs.tar
        docker run --env EXECUTION_CTX='docker' -it --privileged --net="host" -v /opt/container-data/mesh:/opt comms_vm /bin/bash
        #Add restart policy of the container if it stops or device rebooted
        docker update --restart unless-stopped comms_vm
    elif [ "$docker_exec_env" == "ubuntu" ]; then
        docker build -t comms_vm .
        docker run -it --privileged --net="host" --rm comms_vm /bin/bash
    fi
fi
