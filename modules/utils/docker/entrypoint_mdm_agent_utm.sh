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
  while ps aux | grep [i]nit_halow > /dev/null; do
      sleep 1
  done

  echo "Starting 11s mesh service"
  # Loop for mesh service
  for i in {0..2}; do
    if [ "$i" -eq 0 ] || [ -f "/opt/${i}_mesh.conf" ]; then
      /opt/S9011sNatsMesh start id"$i"
    fi
  done

  sleep 3

  #######################################
  # Enable MDM stuff                    #
  #######################################
  echo "starting mdm agent for testing purposes"
  /opt/S90mdm_agent start
fi
