#!/usr/bin/bash
# shellcheck disable=SC2034

generate_br_lan_ip() {
  mesh_if_mac="$(ip -brief link | grep "$MESH_VIF" | awk '{print $3; exit}')"
  ip_random="$(echo "$mesh_if_mac" | cut -b 16-17)"
  br_lan_ip="192.168.1."$((16#$ip_random))
}

source_configuration() {
  if [ -f "/opt/mesh.conf" ]; then
    # Calculate hash for mesh.conf file
    hash=$(sha256sum /opt/mesh.conf | awk '{print $1}')
    # Compare calculated hash with the one created when setting has been applied
    if diff <(printf %s "$hash") /opt/mesh.conf_hash; then
      source /opt/mesh.conf
    else
      # Revert to default mesh
      if [ -f "/opt/mesh_default.conf" ]; then
        source /opt/mesh_default.conf
      else
        echo "no valid mesh configuration found!"
		exit 1
      fi
    fi
  elif [ -f "/opt/mesh_default.conf" ]; then
      source /opt/mesh_default.conf
  else
      echo "no valid mesh configuration found!"
      exit 1
  fi
}
