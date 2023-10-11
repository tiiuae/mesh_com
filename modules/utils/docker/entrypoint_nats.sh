#!/bin/bash

source /opt/mesh-helper.sh

# sources mesh configuration and sets start_opts
source_configuration

if [ "$MSVERSION" != "nats" ]; then
  if [ -f "/usr/local/bin/entrypoint.sh" ]; then
     /bin/bash /usr/local/bin/entrypoint.sh
  else
     /bin/bash /opt/mesh_com/modules/utils/docker/entrypoint.sh
  fi
else

  if [ ! -f "/opt/identity" ]; then
      echo "generates identity id"
      generate_identity_id
  fi

  echo "set bridge ip"
  generate_br_lan_ip

  echo "starting 11s mesh service"
  /opt/S9011sNatsMesh start id0
  if [ -f "opt/1_mesh.conf" ]; then
    /opt/S9011sNatsMesh start id1
  fi
  if [ -f "opt/2_mesh.conf" ]; then
    /opt/S9011sNatsMesh start id2
  fi


  echo "starting AP service"
  /opt/S90APoint start id0

  echo "wait for bridge to be up..."
  while ! (ifconfig | grep -e "$br_lan_ip") > /dev/null; do
    sleep 1
  done

  echo "starting provisioning agent"
  # blocks execution until provisioning is done or timeout (30s)
  # IP address and port are passed as arguments and hardcoded. TODO: mDNS
  python /opt/nats/src/comms_provisioning.py -t 30 -s 192.168.1.254 -p 8080 -o /opt > /opt/comms_provisioning.log 2>&1

  echo "Start nats server and client nodes"
  /opt/S90nats_discovery start

  echo "wait for nats.conf to be created"
  until [ -f /var/run/nats.conf ]; do
    sleep 1
  done

  echo "starting nats server"
  /opt/S90nats_server start

  echo "starting radvd & socat"
  radvd -C /etc/radvd.conf  # TODO: for some reason init.d is not working

  echo "starting comms services"
  /opt/S90comms_controller start

  # alive
  nohup /bin/bash -c "while true; do sleep infinity; done"
fi
