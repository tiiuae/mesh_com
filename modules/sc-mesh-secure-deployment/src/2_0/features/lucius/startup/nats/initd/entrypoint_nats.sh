#!/bin/bash

source /opt/mesh-helper.sh

# sources mesh configuration and sets start_opts
source_configuration "0"

if [ "$MSVERSION" != "nats" ]; then
  if [ -f "/usr/local/bin/entrypoint.sh" ]; then
     /bin/bash /usr/local/bin/entrypoint.sh
  else
     /bin/bash /opt/mesh_com/modules/utils/docker/entrypoint.sh
  fi
else

  #######################################
  # BC needs to be on place before this #
  #######################################

  # TODO: Identity from BC or HSM?
  if [ ! -f "/opt/identity" ]; then
      echo "generates identity id"
      generate_identity_id
  fi

  #######################################
  # Initialise default radios and IF    #
  #######################################

  # Do not continue in case halow init has not finished
  # Halow is slow to start, so we wait for it to finish
  while ps aux | grep "[i]nit_halow" > /dev/null; do
      sleep 1
  done

  #######################################
  # Enable MDM stuff                    #
  #######################################
  echo "starting mdm agent"
  /opt/S90mdm_agent start

  # Start jamming service
  JAMMING=$(extract_features_value "jamming" $YAML_FILE)
  if [ "$JAMMING" == "true" ]; then
    echo "starting jamming avoidance service"
    /opt/S99jammingavoidance start
  fi

  #######################################
  # Enable FMO stuff                    #
  #######################################
  # FMO can be configured in the features.yaml file
  FMO=$(extract_features_value "FMO" $YAML_FILE)
  if [ "$FMO" = "true" ]; then
    echo "starting Alfred"
    /opt/S90Alfred start

    echo "starting provisioning agent"
    # blocks execution until provisioning is done or timeout (30s)
    # IP address and port are passed as arguments and hardcoded.
    python /opt/nats/src/comms_provisioning.py -t 30 -s 192.168.1.254 -p 8080 -o /opt > /opt/comms_provisioning.log 2>&1

    echo "Start nats server and client nodes"
    /opt/S90nats_discovery start

    echo "wait for nats.conf to be created"
    until [ -f /var/run/nats.conf ]; do
      sleep 1
    done

    echo "starting nats server"
    /opt/S90nats_server start

    echo "starting comms services"
    /opt/S90comms_controller start
  fi # FMO

  echo "starting mptcp"
  if [ -f "/var/run/mptcp.conf" ]; then
    /opt/S90mptcp start
  fi

  # /opt/wait_for_bat_if.sh

  /opt/S90Alfred start
  /opt/S90AlfredUpper start
  /opt/S91Socat start

  /opt/S92Lucius start

  # alive
  nohup /bin/bash -c "while true; do sleep infinity; done"
fi
