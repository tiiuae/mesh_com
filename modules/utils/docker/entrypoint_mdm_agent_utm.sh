#!/bin/bash

source /opt/mesh-helper.sh

  #######################################
  # BC needs to be on place before this #
  #######################################

  # TODO: Identity from BC or HSM?
  if [ ! -f "/opt/identity" ]; then
      echo "generates identity id"
      generate_identity_id
  fi

  sleep 3

  #######################################
  # Enable MDM stuff                    #
  #######################################
  echo "starting mdm agent for testing purposes"
  /opt/S90mdm_agent start
