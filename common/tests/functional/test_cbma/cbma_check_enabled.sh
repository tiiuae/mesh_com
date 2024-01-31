#!/bin/sh

features_yaml="/opt/mesh_com/modules/sc-mesh-secure-deployment/src/2_0/features.yaml"
state="$(awk '/CBMA/{print $2}' $features_yaml)"

if [ "$state" != "true" ]; then
    echo "Fail"
    exit 1
fi
echo "Pass"
