#!/bin/bash -eu

# this file previously defined at: https://github.com/tiiuae/fogsw_configs/blob/d9c24cb475449a968c6484b8a01803288ad87e93/setup_fog.sh

# fallback to old way of resolving DRONE_DEVICE_ID
if [ -z "${DRONE_DEVICE_ID+ }" ]; then
	echo "WARN: DRONE_DEVICE_ID not set, trying to resolve it from /enclave/drone_device_id" >&2

	# Source local variables (DRONE_DEVICE_ID, RTSP_SERVER_ADDRESS) for this script
	. /enclave/drone_device_id

	# Export global environment variable (the above sourced script doesn't export anything)
	export DRONE_DEVICE_ID
fi

# Source ROS paths
set +u # poorly coded script don't survive erroring on unbound variables (mentions AMENT_TRACE_SETUP_FILES)
. /opt/ros/${ROS_DISTRO}/setup.bash
set -u

# run actual ros command with now hopefully all the right environment details filled in
exec -- $@
