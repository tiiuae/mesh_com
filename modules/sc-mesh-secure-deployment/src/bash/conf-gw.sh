#!/bin/bash
#gw_info=$2
meshcom_path=$(pwd | awk -F 'mesh_com' '{print $1 FS "/"}')
sc_path=$(pwd | awk -F 'sc-mesh-secure-deployment' '{print $1 FS "/"}')
cp $meshcom_path/common/scripts/mesh-gw.sh /usr/sbin/.
chmod 744 /usr/sbin/mesh-gw.sh
cp $sc_path/services/initd/S92gw /etc/init.d/.
chmod 777 /etc/init.d/S92gw
/etc/init.d/S92gw start
# IF inf is wl, auto connect wlx to AP at boot using wpa_supplicant
#if [[ $gw_info == 'wl*' ]]
# then    # True if $a starts with a "z" (wildcard matching).
# cp conf/ap.conf "/etc/wpa_supplicant/wpa_supplicant-${gw_info}.conf"
# chmod 600 "/etc/wpa_supplicant/wpa_supplicant-${gw_info}.conf"
# #/etc/init.d/wpa_supplicant+ $gw_info + '.service'
#fi