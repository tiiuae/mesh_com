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

## Build Modules

### mesh_com
```
$ TBD

```

### sc-mesh-secure-deployment
```
$ TBD

```