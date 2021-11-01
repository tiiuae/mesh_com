#! /bin/bash
meshcom_path=$(pwd | awk -F 'mesh_com' '{print $1 FS "/"}')
sc_path=$(pwd | awk -F 'sc-mesh-secure-deployment' '{print $1 FS "/"}')

cp $meshcom_path/common/scripts/mesh-11s.sh  /usr/sbin/.
chmod 744 /usr/sbin/mesh-11s.sh
cp $sc_path/services/initd/S9011sMesh /etc/init.d/.
chmod 755 /etc/init.d/S9011sMesh
chown root:root /etc/init.d/S9011sMesh
update-rc.d S9011sMesh defaults
update-rc.d S9011sMesh enable
/etc/init.d/S9011sMesh start
sleep 2
