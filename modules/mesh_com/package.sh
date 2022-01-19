#!/bin/bash -e

if [ "${ROS_DISTRO}" = "" ]; then
   echo "ROS_DISTRO not set!!"
   exit 1
fi

build_nbr=$1
git_version_string=$2

pushd modules/mesh_com
bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro ${ROS_DISTRO} --place-template-files \
        && sed -i "s/@(DebianInc)@(Distribution)/@(DebianInc)/" debian/changelog.em \
        && [ ! "$distr" = "" ] && sed -i "s/@(Distribution)/${distr}/" debian/changelog.em || : \
        && bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro ${ROS_DISTRO} --process-template-files -i ${build_nbr}${git_version_string} \
        && sed -i 's/^\tdh_shlibdeps.*/& --dpkg-shlibdeps-params=--ignore-missing-info/g' debian/rules \
        && sed -i 's!dh_auto_test || true!dh_auto_test!g' debian/rules \
        && fakeroot debian/rules clean \
        && fakeroot debian/rules "binary --parallel"
popd
