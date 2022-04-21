#! /bin/bash
meshcom_path=$(pwd | awk -F 'mesh_com' '{print $1 FS "/"}')
sc_path=$meshcom_path"/modules/sc-mesh-secure-deployment"

cp $meshcom_path/common/scripts/mesh-ibss.sh  /opt/.
chmod 744 /opt/mesh-ibss.sh
cp $sc_path/services/initd/S90mesh /opt/.
chmod 755 /opt/S90mesh
chown root:root /opt/S90mesh
update-rc.d S90mesh defaults
update-rc.d S90mesh enable
/opt/S90mesh start
sleep 2
