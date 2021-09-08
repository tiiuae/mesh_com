#!/bin/bash
gateway=$1
meshcom_path=$(pwd | cut -d'/' -f-3)
sc_path=$(pwd | cut -d'/' -f-5)

route add default gw $gateway bat0
cp $meshcom_path/common/scripts/mesh-default-gw.sh /usr/sbin/.
chmod 744 /usr/sbin/mesh-default-gw.sh
cp $sc_path/services/initd/S91defaultroute /etc/initd/.
chmod 777 /etc/initd/S91defaultroute
/etc/initd/S91defaultroute start $gateway
