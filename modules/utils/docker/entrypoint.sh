#!/bin/bash

source /opt/mesh-helper.sh

# sources mesh configuration and sets start_opts
source_configuration
# set bridge ip, sets br_lan_ip
generate_br_lan_ip

echo "starting 11s mesh service"
/opt/S9011sMesh start          # todo: this should be started in separate container

echo "starting AP service"
/opt/S90APoint start           # todo: this should be started in separate container

# wait for bridge to be up
while ! (ifconfig | grep -e "$br_lan_ip") > /dev/null; do
  sleep 1
done

# Start nats server and client nodes
/opt/S90nats_discovery start "$ROLE"  # todo: this should be started in separate container

# wait for nats.conf to be created
until [ -f /var/run/nats.conf ]; do
  sleep 1
done
/opt/S90nats_server start      # todo: this should be started in separate container
/opt/S90comms_controller start # todo: this should be started in separate container

#start gw manager
nohup python -u /opt/mesh_com/modules/sc-mesh-secure-deployment/src/gw/main.py
