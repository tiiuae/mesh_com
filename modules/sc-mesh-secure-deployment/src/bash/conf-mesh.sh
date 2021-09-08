#! /bin/bash
meshcom_path=$(pwd | cut -d'/' -f-3)
sc_path=$(pwd | cut -d'/' -f-5)

mesh_interface=$1
mesh_address=$2
cp $meshcom_path/common/scripts/mesh-init.sh  /usr/sbin/.
chmod 744 /usr/local/bin/mesh-init.sh
cp $sc_path/services/initd/S90mesh /etc/init.d/.
chmod 777 /etc/init.d/S90mesh
/etc/init.d/S99mesh start $mesh_interface $address
sleep 2