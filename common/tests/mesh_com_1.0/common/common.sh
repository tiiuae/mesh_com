#!/bin/bash

PASS=1
FAIL=0
result=$PASS
MESH_COM_ROOTDIR="../../../"
source ./common/env.sh
log_file="./mesh_com_1.0_debug"
results_file="./mesh_com_1.0_results"
server="_Server_Case"
client="_Client_Case"
log=".log"

#######################################
# Add date prefix to line and write to log
# Globals:
# Arguments:
#   debug print lines
#######################################
print_log() {

    time_stamp="$(date +'%Y-%m-%dT%H:%M:%S%z')"
    while IFS= read -r line; do
       if [ "$1" = "result_server" ] ; then
          printf '[%s]: %s\n' "$time_stamp" "$line" |& tee -a "$results_file$server$log" |& tee -a "$log_file$server$log"
       elif [ "$1" = "log_server" ] ; then
          printf '[%s]: %s\n' "$time_stamp" "$line" |& tee -a "$log_file$server$log"
       elif [ "$1" = "result_client" ] ; then
          printf '[%s]: %s\n' "$time_stamp" "$line" |& tee -a "$results_file$client$log" |& tee -a "$log_file$client$log"
       elif [ "$1" = "log_client" ] ; then
          printf '[%s]: %s\n' "$time_stamp" "$line" |& tee -a "$log_file$client$log"
        fi
    done
}

#######################################
# Check mesh_com_1.0 prerequisites
# Globals:
# Arguments:
#   debug print lines
#######################################
test_mesh_com_1.0_dependencies()
{
  if ! [ -x "$(command -v python)" ] || ! [ -x "$(command -v avahi-daemon)" ] ||! [ -x "$(command -v avahi-dnsconfd)" ]; then
    echo -e "mesh_com_1.0 dependencies are missing!" | print_log
    result=$FAIL
    return
  fi
}


#######################################
# start_mesh_server
# Globals:
# Arguments:
#   debug print lines
#######################################
start_mesh_server()
{
  # make sure flask server is running in seperate thread
  source ./$MESH_COM_ROOTDIR/modules/sc-mesh-secure-deployment/configure.sh -s -t & 2>&1 |& tee logs/"$0".log
}

start_mesh_client()
{
  source ./$MESH_COM_ROOTDIR/modules/sc-mesh-secure-deployment/configure.sh -c -t & 2>&1 |& tee logs/"$0".log
}


