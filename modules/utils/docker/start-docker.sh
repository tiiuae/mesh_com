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
        if [ ! -d "/opt/container-data/mesh" ]; then
            echo "create persist mesh container data partition"
            # create persist mesh container data partition
            # data/container-data/mesh
            # ├── state.json
            # └── wpa_supplicant_11s.conf
            # └── wpa_supplicant_sta.conf
            # └── wpa_supplicant_ap.conf
            mkdir -p /opt/container-data/mesh
            mv /opt/mesh_com* /opt/container-data/mesh/
        fi
        # Copy hardware identification file into container-data
        if [ ! -d "/opt/container-data/mesh/hardware" ]; then
            if [ -f "/etc/comms_pcb_version" ]; then
                mkdir -p /opt/container-data/mesh/hardware
                cp /etc/comms_pcb_version /opt/container-data/mesh/hardware/comms_pcb_version
            fi
        fi
        # change rootfs location once its mounted in dedicated partition
        if [ -f "/root/rootfs.tgz" ]; then
            echo "import rootfs.tgz commms vm"
            # shellcheck disable=SC2002
            cat /root/rootfs.tgz | docker import - comms_vm
            docker build -t comms_vm .
        else
            docker import - comms_vm < /rootfs.tar
        fi
        meshcom_path="/opt/container-data/mesh/mesh_com/"
        echo "$meshcom_path"
        if [ ! -f "/opt/container-data/mesh/mesh.conf" ]; then
		cp /opt/mesh_default.conf /opt/container-data/mesh/mesh.conf
		cp $meshcom_path/common/scripts/mesh-ibss.sh  /opt/container-data/mesh/.
		chmod 755 /opt/container-data/mesh/mesh-ibss.sh
		cp $meshcom_path/modules/sc-mesh-secure-deployment/services/initd/S90mesh /opt/container-data/mesh/.
		chmod 755 /opt/container-data/mesh/S90mesh
		cp $meshcom_path/common/scripts/mesh-11s.sh  /opt/container-data/mesh/.
		chmod 755 /opt/container-data/mesh/mesh-11s.sh
		cp $meshcom_path/modules/sc-mesh-secure-deployment/services/initd/S9011sMesh /opt/container-data/mesh/.
		chmod 755 /opt/container-data/mesh/S9011sMesh
        fi
        cp /etc/umurmur.conf $meshcom_path/modules/utils/docker/umurmur.conf

        running=$(docker container inspect -f '{{.State.Running}}' "mesh_comms_vm")
        if [ "$running" != "true" ]; then
           docker rm -f mesh_comms_vm
           docker run --name mesh_comms_vm -d --env EXECUTION_CTX='docker' -it --privileged --net="host" -v /opt/container-data/mesh:/opt comms_vm
           #Add restart policy of the container if it stops or device rebooted
           docker update --restart unless-stopped mesh_comms_vm
        fi
    elif [ "$docker_exec_env" == "ubuntu" ]; then
        docker build -t comms_vm .
        docker run -it --privileged --net="host" --rm comms_vm /bin/bash
    fi
fi
