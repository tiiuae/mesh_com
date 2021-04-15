#!/bin/bash
if [ $# -eq 0 ]
  then
      echo "Error:"
      echo "Please give docker image tag name!"
      echo "example. ./1_create_docker_image.sh <image tag name>"
      echo
      echo "example. ./1_create_docker_image.sh my_ubuntu_image"
      exit
fi
docker build -t "$1" .
