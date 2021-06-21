#!/bin/bash

EPATH=$(find /home -type d -name "sc-mesh-secure-deployment") #find installation path
function client {
  echo '> Configuring the client...'
  # Make the server
  cd $EPATH
  pushd .
  cd ../..
  make mesh_tb_client
  popd
  # Connect to the same AP as the server
  read -p "> We need to be connect to the same network as the server... Connect to an Access Point? (Y/N): " confirm
  if [[ $confirm == [yY] || $confirm == [yY][eE][sS] ]]; then
    ap_connect
  fi
  echo -n '> Server discovery...'
  # get server IPv4 and hostname
  while ! [ "$aux" ] ; do
    aux=($(avahi-browse -rptf _http._tcp | awk -F';' '$1 == "=" && $3 == "IPv4" && $4 ~ /^mesh_server/ {print $8 " " $7}'))
  done
  delete=()
  delete+=(127.0.0.1)
  delete+=($(hostname).local)
  local_ip=$(ip addr show wlan0 | grep "inet\b" | awk '{print $2}' | cut -d/ -f1) #assuming it's wlan0
  delete+=($local_ip)
  for target in "${delete[@]}"; do # removing local server
    for i in "${!aux[@]}"; do
      if [[ ${aux[i]} = $target ]]; then
        unset 'aux[i]'
      fi
    done
  done
  for i in "${!aux[@]}"; do #to set new index
    servers+=( "${aux[i]}" )
  done
  if [ "${#servers[@]}" -gt 2 ]; then
    echo "More than one server found"
    echo "Measuring delay";
    fastest_response=2147483647 # largest possible integer
    for index in "${!servers[@]}"
    do :
      if [ $((index%2)) -eq 0 ]
      then
          avg=$(ping -c 4 "${servers[index]}" | tail -1| awk '{print $4}' | cut -d '/' -f 2) #measuring latency
          if (( $(bc -l <<< "$avg < $fastest_response") )) ; then
              fastest_response=$avg
              fastest_site=${servers[index]}
              fastest_index=$index
          fi
      fi
    done
    server_ip=$fastest_site
    server_host=${servers[$((fastest_index+1))]}
  else
        server_ip=${servers[0]}
        server_host=${servers[1]}
  fi
  echo "> We will use src/ecc_key.der if it already exists, or we can try and fetch it..."
  read -p "> Do you want to fetch the certificate from the server $server_host@$server_ip? (Y/N): " confirm

  if [[ $confirm == [yY] || $confirm == [yY][eE][sS] ]]; then
    echo '> Fetching certificate from server...'
    read -p "- Server Username: " server_user
    # pull the key from the server
    scp $server_user@$server_ip:/home/$server_user/mesh_com/modules/sc-mesh-secure-deployment/src/ecc_key.der $EPATH/src/ecc_key.der
    cp $EPATH/src/ecc_key.der /etc/ssl/certs/ecc_key.der
  fi

  echo '> Configuring the client and connecting to server...'
  sudo python3 src/client-mesh.py -c /etc/ssl/certs/ecc_key.der -s http://$server_ip:5000
}
client