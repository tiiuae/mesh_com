#!/bin/bash

source /opt/mesh-helper.sh

# sources mesh configuration and sets start_opts
source_configuration

if [ "$MSVERSION" != "nats" ]; then
  cp /opt/mesh_default.conf /opt/mesh.conf
  /bin/bash entrypoint_ms.sh
else

  if [ ! -f "/opt/identity" ]; then
      # generates identity id (mac address + cpu serial number)
      generate_identity_id
  fi

  # set bridge ip, sets br_lan_ip
  generate_br_lan_ip

  echo "starting 11s mesh service"
  /opt/S9011sNatsMesh start

  echo "starting AP service"
  /opt/S90APoint start

  # wait for bridge to be up
  while ! (ifconfig | grep -e "$br_lan_ip") > /dev/null; do
    sleep 1
  done

  # Start nats server and client nodes
  /opt/S90nats_discovery start "$ROLE"

  # wait for nats.conf to be created
  until [ -f /var/run/nats.conf ]; do
    sleep 1
  done
  /opt/S90nats_server start
  /opt/S90comms_controller start

  #start gw manager TBD
  # nohup python -u /opt/mesh_com/modules/sc-mesh-secure-deployment/src/gw/main.py
fi