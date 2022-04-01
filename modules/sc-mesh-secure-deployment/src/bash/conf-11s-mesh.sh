#! /bin/bash
meshcom_path=$(pwd | awk -F 'mesh_com' '{print $1 FS "/"}')
sc_path=$(pwd | awk -F 'sc-mesh-secure-deployment' '{print $1 FS "/"}')

cp $meshcom_path/common/scripts/mesh-11s.sh  /opt/.
chmod 744 /opt/mesh-11s.sh
cp $sc_path/services/initd/S9011sMesh /opt/.
chmod 755 /opt/S9011sMesh
chown root:root /opt/S9011sMesh
update-rc.d S9011sMesh defaults
update-rc.d S9011sMesh enable
/opt/S9011sMesh start
sleep 2
