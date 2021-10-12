#! /bin/bash
meshcom_path=$(pwd | awk -F 'mesh_com' '{print $1 FS "/"}')
sc_path=$(pwd | awk -F 'sc-mesh-secure-deployment' '{print $1 FS "/"}')

mesh_interface=$1
mesh_address=$2
cp $meshcom_path/common/scripts/mesh-ibss.sh  /usr/sbin/.
chmod 744 /usr/sbin/mesh-ibss.sh
cp $sc_path/services/initd/S90mesh /etc/init.d/.
chmod 777 /etc/init.d/S90mesh
/etc/init.d/S90mesh start $mesh_interface $address
sleep 2