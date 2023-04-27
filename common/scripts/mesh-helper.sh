#!/usr/bin/bash
# shellcheck disable=SC2034

generate_br_lan_ip() {
  mesh_if_mac="$(ip -brief link | grep "$MESH_VIF" | awk '{print $3; exit}')"
  ip_random="$(echo "$mesh_if_mac" | cut -b 16-17)"
  br_lan_ip="192.168.1."$((16#$ip_random))
}

source_configuration() {
  if [ -f "/opt/mesh.conf" ]; then
      source /opt/mesh.conf
  elif [ -f "/opt/mesh_default.conf" ]; then
      source /opt/mesh_default.conf
  else
      echo "no valid mesh configuration found!"
      exit 1
  fi
}
