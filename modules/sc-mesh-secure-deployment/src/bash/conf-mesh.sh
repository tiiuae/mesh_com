#! /bin/bash
meshcom_path=$(pwd | awk -F 'mesh_com' '{print $1 FS "/"}')
sc_path=$(pwd | awk -F 'sc-mesh-secure-deployment' '{print $1 FS "/"}')

cp $meshcom_path/common/scripts/mesh-ibss.sh  /usr/sbin/.
chmod 744 /usr/sbin/mesh-ibss.sh
cp $sc_path/services/initd/S90mesh /etc/init.d/.
chmod 755 /etc/init.d/S90mesh
chown root:root /etc/init.d/S90mesh
update-rc.d S90mesh defaults
update-rc.d S90mesh enable
/etc/init.d/S90mesh start
sleep 2
