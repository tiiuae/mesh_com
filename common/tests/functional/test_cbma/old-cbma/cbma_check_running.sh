#!/bin/sh

mdm_constants='/opt/mesh_com/modules/sc-mesh-secure-deployment/src/nats/src/constants.py'
lower_or_upper="$(awk "BEGIN{print toupper(\"$1\")}")"

if [ $# -ne 1 ] || case "$lower_or_upper" in LOWER|UPPER) false;; esac; then
    echo "Usage: $0 <lower|upper>"
    exit 1
fi

port="$(awk "/^[[:space:]]*CBMA_PORT_${lower_or_upper}/{print \$NF}" $mdm_constants)"

if ! ss -lp | grep -q "\\b$port\\b.*python"; then
    echo "Fail"
    exit 1
fi
echo "Pass"
