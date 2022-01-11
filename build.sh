#!/bin/bash

set -euxo pipefail

output_dir=$1

git_commit_hash=${2:-$(git rev-parse HEAD)}

git_version_string=${3:-$(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1)}

build_number=${GITHUB_RUN_NUMBER:=0}

ros_distro=${ROS_DISTRO:=galactic}

iname=${PACKAGE_NAME:=mesh_com}

iversion=${PACKAGE_VERSION:=latest}

docker build \
  --build-arg UID=$(id -u) \
  --build-arg GID=$(id -g) \
  --build-arg ROS_DISTRO=${ros_distro} \
  --build-arg PACKAGE_NAME=${iname} \
  --pull \
  -f ./modules/mesh_com/Dockerfile.build_env -t "${iname}_build:${iversion}" .

docker run \
  --rm \
  -v $(pwd):/${iname}/sources \
  ${iname}_build:${iversion} \
  modules/mesh_com/package.sh \
  ${build_number} \
  ${git_version_string}

mkdir -p ${output_dir}
cp modules/*.deb ${output_dir}
rm -Rf modules/*.deb

exit 0
