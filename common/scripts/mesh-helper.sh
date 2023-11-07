#!/usr/bin/bash
# shellcheck disable=SC2034

# Function to retrieve MAC address
__get_mac_address() {
  local mac_address
  for interface in "$@"; do
    mac_address=$(ip link show "${interface}" 2>/dev/null | awk '/ether/ {print $2}')
    if [[ -n "${mac_address}" ]]; then
      mac_address=${mac_address//:/}  # Remove colons from MAC address
      echo "${mac_address}"
      return
    fi
  done
  echo ""  # Return empty string if no MAC address is found
}

# Function to retrieve CPU serial number
__get_cpu_serial_number() {
  local cpuinfo_file="/proc/cpuinfo"
  local serial_number
  serial_number=$(grep -m1 "Serial" "${cpuinfo_file}" | awk -F': ' '{print $2}')
  echo "${serial_number}"
}

# this function generates the identity id from the mac address and the cpu serial number
generate_identity_id() {
  # Split the input into an array of interface names
  IFS=' ' read -ra interface_array <<< "wlp1s0 wlp2s0 wlp3s0 halow1 eth0 wlan0 wlan1"

  mac_address=$(__get_mac_address "${interface_array[@]}")
  cpu_serial_number=$(__get_cpu_serial_number)

  identity_id="${mac_address}${cpu_serial_number}"

  if [ -n "$identity_id" ]; then
cat <<EOF > /opt/identity
${identity_id}
EOF
    echo "Identity ID: ${identity_id}"
  else
    exit 1
  fi
}

generate_lan_bridge_ip() {
  local mesh_if_mac

  bridge_name=$(echo "$BRIDGE" | cut -d' ' -f1)

  mesh_if_mac=$(cat /sys/class/net/"$id0_MESH_VIF"/address)
  if [ -z "$mesh_if_mac" ]; then
     echo "generate_lan_bridge_ip: MAC not found for id0_MESH_VIF! Configuration error?" > /dev/kmsg
    mesh_if_mac="$(cat /sys/class/net/eth0/address)"
  fi
  local ip_random
  ip_random="$(echo "$mesh_if_mac" | cut -b 16-17)"
  bridge_ip="192.168.1."$((16#$ip_random))

  # configure slaac
  if [ -f /opt/slaac.sh ]; then
    echo "Configuring SLAAC on $BRIDGE..."
    source /opt/slaac.sh
    /opt/slaac.sh $BRIDGE
  else
    echo "Error: slaac.sh does not exist."
    exit 1
  fi

  # legacy support
  br_lan_ip=$bridge_ip
}

source_configuration() {
  # $1: index of the mesh.conf file to be used
  CONFIGURATION_INDEX=$1

  if [ -f "/opt/${CONFIGURATION_INDEX}_mesh.conf" ]; then
    # Calculate hash for mesh.conf file
    hash=$(sha256sum /opt/"${CONFIGURATION_INDEX}"_mesh.conf | awk '{print $1}')
    # Compare calculated hash with the one created when setting has been applied
    if diff <(printf %s "$hash") /opt/"${CONFIGURATION_INDEX}"_mesh.conf_hash; then
      # shellcheck disable=SC1090
      source /opt/"${CONFIGURATION_INDEX}"_mesh.conf
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
