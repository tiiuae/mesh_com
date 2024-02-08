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

#######################################
# Enable MDM stuff                    #
#######################################
echo "starting mdm agent"
/opt/S90mdm_agent start
