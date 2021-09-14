#! /bin/bash
meshcom_path=$(pwd | cut -d'/' -f-3)
sc_path=$(pwd | cut -d'/' -f-5)

mesh_interface=$1
mesh_address=$2
cp $meshcom_path/common/scripts/mesh-ibss.sh  /usr/sbin/.
chmod 744 /usr/sbin/mesh-ibss.sh
cp $sc_path/services/initd/S90mesh /etc/init.d/.
chmod 777 /etc/init.d/S90mesh
/etc/init.d/S90mesh start $mesh_interface $address
sleep 2