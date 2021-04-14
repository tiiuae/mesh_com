#!/bin/bash
if [ $# -eq 0 ]
  then 
      echo "Error:"
      echo "Please give docker container name/hostname and image tag name!"
      echo "usage: ./2_run_docker.sh <my_docker_space_name> <image tag name>"
      echo
      echo "example: ./2_run_docker.sh my_build my_ubuntu_image"
      exit
fi
docker run -it --hostname "$1" --name "$1" -v "${PWD}"/build:/build "$2"
