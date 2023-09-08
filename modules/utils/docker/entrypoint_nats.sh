#!/bin/bash

source /opt/mesh-helper.sh

# sources mesh configuration and sets start_opts
source_configuration

if [ "$MSVERSION" != "nats" ]; then
  /bin/bash /usr/local/bin/entrypoint.sh
else

  if [ ! -f "/opt/identity" ]; then
      echo "generates identity id"
      generate_identity_id
  fi

  echo "set bridge ip"
  generate_br_lan_ip

  echo "starting 11s mesh service"
  /opt/S9011sNatsMesh start

  echo "starting AP service"
  /opt/S90APoint start

  echo "wait for bridge to be up..."
  while ! (ifconfig | grep -e "$br_lan_ip") > /dev/null; do
    sleep 1
  done

  echo "Start nats server and client nodes"
  /opt/S90nats_discovery start

  echo "wait for nats.conf to be created"
  until [ -f /var/run/nats.conf ]; do
    sleep 1
  done

  echo "starting nats server"
  /opt/S90nats_server start

  echo "starting radvd & socat"
  radvd -C /etc/radvd.conf  # for some reason init.d is not working
  /opt/S90socat start  # socat is used to provide IPv6 NATS IF

  echo "starting comms services"
  /opt/S90comms_controller start
  echo "starting provisioning agent"
  /opt/S90provisioning_agent start

  # alive
  nohup /bin/bash -c "while true; do sleep infinity; done"
fi
