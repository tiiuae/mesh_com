# Docker to run SecComm Solution on an Ubuntu container

## Install Docker

```
$ sudo apt update
$ sudo apt install docker.io
$ sudo systemctl enable --now docker
```

## Run Docker
```
./start-docker.sh
```

once it finishes all the steps:

- run the server as

```
./configure -s
```

- run the client as
```
./configure -c
```
