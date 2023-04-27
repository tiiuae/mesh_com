#!/bin/bash

source /opt/mesh-helper.sh

# sources mesh configuration and sets start_opts
source_configuration
# set bridge ip, sets br_lan_ip
generate_br_lan_ip

echo "starting 11s mesh service"
/opt/S9011sMesh start

echo "starting AP service"
/opt/S90APoint start

# wait for bridge to be up
while ! (ifconfig | grep -e "$br_lan_ip") > /dev/null; do
    sleep 1
done

# Start nats server and client nodes
/opt/S90nats_server start $br_lan_ip
/opt/S90comms_controller start $br_lan_ip

#start gw manager
nohup python -u /opt/mesh_com/modules/sc-mesh-secure-deployment/src/gw/main.py
