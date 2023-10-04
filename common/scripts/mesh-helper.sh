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
  IFS=' ' read -ra interface_array <<< "wlp1s0 eth0 wlan0 wlan1"

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

generate_br_lan_ip() {
  local mesh_if_mac
  mesh_if_mac="$(ip -brief link | grep "$MESH_VIF" | awk '{print $3; exit}')"
  local ip_random
  ip_random="$(echo "$mesh_if_mac" | cut -b 16-17)"
  br_lan_ip="192.168.1."$((16#$ip_random))


  cat > /etc/radvd.conf <<- EOF
interface br-lan {
  AdvSendAdvert on;
  MinRtrAdvInterval 3;
  MaxRtrAdvInterval 10;
  prefix 2001:db8:1234:5678::/64 {
      AdvOnLink on;
      AdvAutonomous on;
  };
};
EOF
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

install_python_packages() {
  # install the python packages
  MESH_FOLDER="/opt/mesh_com"
  if [ -d "$MESH_FOLDER/modules/utils/package/python_packages" ]; then
     echo "Directory $MESH_FOLDER/modules/utils/package/python_packages exists."
  else
    #!/bin/bash
    tar -C $MESH_FOLDER/modules/utils/package/ -zxvf $MESH_FOLDER/modules/utils/package/python_packages.tar.gz
    cd $MESH_FOLDER/modules/utils/package/python_packages || return

    for f in {*.whl,*.gz};
      do
        name="$(echo "$f" | cut -d"-" -f1)"
        if python -c 'import pkgutil; exit(not pkgutil.find_loader("$name"))'; then
          echo "$name" "installed"
        else
          echo "$name" "not found"
          echo "installing" "$name"
          pip install --no-index "$f" --find-links .;
        fi
    done
  fi
}