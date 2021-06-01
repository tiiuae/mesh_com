# Common mesh_com repository

Every module has own instructions about their usage:

- ROS2 Node module usage instructions:
https://github.com/tiiuae/mesh_com/modules/mesh_com

- sc-mesh-secure-deployment module usage instructions:
https://github.com/tiiuae/mesh_com/modules/sc-mesh-secure-deployment

## Install & Clone

### Clone repositories:
```
$ git clone https://github.com/tiiuae/mesh_com.git
$ cd mesh_com
$ git submodule update --init --recursive
$ cd ..
```

## Build Common Libraries

### Cryptolib
```
$ make certificate
$ make server
$ make client
```

### core/os/ubuntu/wpa_supplicant

Update and install dependencies:
```
$ sudo apt update
$ sudo apt-get install build-essential fakeroot dpkg-dev
```

Build and create debian packages:

```
$ cd common/core/os/ubuntu/wpa_supplicant
$ sudo dpkg-buildpackage -rfakeroot -b
```

## Build Modules

### mesh_com

#### Integration to fog_sw

Mesh_com dependecies:
```
$ sudo apt update
$ sudo apt install \
    dh-python \
    batctl \
    alfred

```
systemd mesh.service:
- currently up-to-date versions for fogsw are in:

    https://github.com/tiiuae/fogsw_systemd/tree/main/system

- example:
```
[Unit]
Description="Mesh Service"

[Service]
User=root
Group=root
Type=idle
ExecStart=/bin/sh -c ". /opt/ros/foxy/setup_fog.sh;/opt/ros/foxy/share/bin/mesh-ibss.sh ap; ros2 launch mesh_com mesh_com.launch"

[Install]
WantedBy=multi-user.target
```

Bloom-generate integration to package.sh:
```
$ pushd ../ros2_ws/src/mesh_com/modules/mesh_com
$ bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy &&\
    fakeroot debian/rules binary && mv ../*.deb ../../../../../packaging/
$ popd
```

#### Build in fog_sw

fog_sw installation and build [guide](https://github.com/tiiuae/fog_sw#readme)

colcon:
```
$ pushd .
$ cd fog_sw/ros2_ws
$ colcon build
$ popd
```

Bloom-generate (generate rosdebian package):
```
$ cd ros2_ws/src/mesh_com/modules/mesh_com
$ bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy &&\
    fakeroot debian/rules binary
```

### sc-mesh-secure-deployment
An in-depth README on how to set up sc-mesh-secure-deployment can be accessed [here](modules/sc-mesh-secure-deployment/README.md)
