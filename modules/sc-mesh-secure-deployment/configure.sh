#!/bin/bash

# TODO: We should do the bash n bash and the python in python as much
#       as possible

function help {
echo " ./configure.sh"
echo "     -s               configure as server"
echo "     -c               configure as client"
echo "     -ap              connect/configure Access Point"
echo "     -gw              create/remove gateway"
echo "     --help           this help menu"
echo ""
exit 1
}

#-----------------------------------------------------------------------------#

# Check if command(s) exists and fail otherwise.
function command_exists {
  for cmd in $1; do
    echo -n "> Checking $cmd installed... "
    INSTALLED=($(apt -qq list $cmd 2>/dev/null | awk -F[ '{print $2}'))
    if $INSTALLED &> /dev/null; then
      echo "FALSE"
      echo "> Installing..."
      sudo apt-get install -y -qq $cmd --no-install-recommends
      echo ""
    else
      echo "TRUE"
    fi
  done
}


function menu_from_array()
{
  select choice; do
  # Check the selected menu item number
  if [ 1 -le "$REPLY" ] && [ "$REPLY" -le $# ];
  then
  break;
  else
  echo "Wrong selection: Select any number from 1-$#"
  fi
  done
}


#-----------------------------------------------------------------------------#
function ap_connect {
echo '> Connecting to Access Point...'
read -p "- SSID: " ssid
read -p "- Password: " password
cat <<EOF > conf/ap.conf
network={
  ssid="$ssid"
  psk="$password"
}
EOF
echo '> Please choose from the list of available interfaces...'
interfaces_arr=($(ip link | awk -F: '$0 !~ "lo|vir|doc|eth|bat|^[^0-9]"{print $2}'))
menu_from_array "${interfaces_arr[@]}"

#mesh(IBSS) and sta interface are not supported concurrently.
#Stop mesh(IBSS) service on the selected vif, if already running.
#Below condition will be false for dual radio using seperate sta/ibss vif.
wlan_if="$(iw dev | awk '$1=="Interface"{print $2}')"
wlan_if_count="$(echo $wlan_if | wc -l)"
if [ "$wlan_if_count" = "1" ]; then
  choice="wlan0"
  mesh_service="mesh@""$choice"".service"
  STATUS="$(systemctl is-active $mesh_service)"
  if [ "${STATUS}" = "active" ]; then
    echo "Disabling" $mesh_service
    sudo systemctl stop $mesh_service
    sudo systemctl disable $mesh_service
    sudo ifconfig bat0 down
    sudo rmmod batman_adv
    killall wpa_supplicant
    ifconfig $choice down
    ifconfig $choice up
  fi
fi

sudo wpa_supplicant -B -i $choice -c conf/ap.conf
sudo dhclient -v $choice
}

function ap_create {
echo '> Creating an Access Point...'
command_exists 'isc-dhcp-server net-tools'
echo '> Please choose from the list of available interfaces...'
interfaces_arr=($(ip link | awk -F: '$0 !~ "lo|vir|doc|eth|bat|^[^0-9]"{print $2}'))
menu_from_array "${interfaces_arr[@]}"
read -p "- SSID: " ssid
#read -p "- Password: " password
password=$(systemd-ask-password "Please type your Password:")
read -p "- Custom IP (e.g. XX.0.0.1): " ip
cat <<EOF > conf/ap.conf
  network={
    ssid="$ssid"
    mode=2
    key_mgmt=WPA-PSK
    psk="$password"
    frequency=2437
}
EOF
# FIXME: Each time you echo this it's gonna add to the end of the files and
#        create duplicate lines
echo "INTERFACES=$1" >> /etc/default/isc-dhcp-server
if [ ! -d "/etc/mesh_com" ]; then
  sudo mkdir /etc/mesh_com
fi
echo "AP_INF=$choice" >> /etc/mesh_com/ap.conf
# Create Gateway Service
sudo cp ../../common/scripts/mesh-ap.sh /usr/local/bin/.
sudo chmod 744 /usr/local/bin/mesh-ap.sh
sudo cp services/ap@.service /etc/systemd/system/.
sudo chmod 644 /etc/systemd/system/ap@.service
sudo systemctl enable ap@$ip.service
# Setup wlx at boot using wpa_supplicant
sudo cp conf/ap.conf /etc/wpa_supplicant/wpa_supplicant-$choice.conf
sudo chmod 600 /etc/wpa_supplicant/wpa_supplicant-$choice.conf
sudo systemctl enable wpa_supplicant@$choice.service
#sleep 2
#reboot
}

function ap_remove {
  echo '> Remove an Access Point...'
  echo '> Please choose from the list of available interfaces...'
  interfaces_arr=($(ip link | awk -F: '$0 !~ "lo|vir|doc|eth|bat|^[^0-9]"{print $2}'))
  menu_from_array "${interfaces_arr[@]}"
  sudo systemctl disable wpa_supplicant@$choice.service
  # sudo systemctl enable ap@$ip.service
  sudo rm /etc/wpa_supplicant/wpa_supplicant-$choice.conf
  sudo rm /usr/local/bin/mesh-ap.sh
  sudo rm /etc/systemd/system/ap@.service
  sudo rm /etc/mesh_com/ap.conf
#  reboot
}

function ap_menu {
  echo '> Do you wish to...'
  ap_arr=('Connect to an Access Point?' 'Create an Access Point?' 'Remove an Access Point?')
  menu_from_array "${ap_arr[@]}"
  if [ $REPLY == "1" ]; then
    ap_connect
  elif [[ $REPLY == "2" ]]; then
    ap_create
  elif [[ $REPLY == "3" ]]; then
    ap_remove
  fi
}

#-----------------------------------------------------------------------------#
# TODO: Gateway creation and removal currently creates and completely removes
#       services. Not sure if this is the best approach or if we should check
#       if files already exist / just disable and don't remove scripts.
#-----------------------------------------------------------------------------#
function gw_create {
  echo '> Configuring gateway... Choose a gateway interface:'
  interfaces_arr=($(ip link | awk -F: '$0 !~ "lo|vir|doc|bat|^[^0-9]"{print $2}'))
  menu_from_array "${interfaces_arr[@]}"
  # Create Gateway Service
  echo '> Copying mesh-gw.sh to /usr/local/bin/.'
  sudo cp ../../common/scripts/mesh-gw.sh /usr/local/bin/.
  sudo chmod 744 /usr/local/bin/mesh-gw.sh
  echo "> Copying gw@.service to /etc/systemd/system/"
  sudo cp services/gw@.service /etc/systemd/system/.
  sudo chmod 644 /etc/systemd/system/gw@.service
  echo "> Enabling gw@$choice.service"
  sudo systemctl enable gw@$choice.service
  # If gw inf is a wlan then auto connect to AP at boot using wpa_supplicant
  if [[ $choice = wl* ]]; then
    echo "> Enabling wpa_supplicant@$choice.service"
    sudo cp conf/ap.conf /etc/wpa_supplicant/wpa_supplicant-$choice.conf
    chmod 600 /etc/wpa_supplicant/wpa_supplicant-$choice.conf
    sudo systemctl enable wpa_supplicant@$choice.service
  fi
}

function gw_remove {
  echo '> Removing gateway... Choose a gateway interface:'
  interfaces_arr=($(ip link | awk -F: '$0 !~ "lo|vir|doc|bat|^[^0-9]"{print $2}'))
  menu_from_array "${interfaces_arr[@]}"
  # Create Gateway Service
  echo '> Removing /usr/local/bin/mesh-gw.sh'
  sudo rm /usr/local/bin/mesh-gw.sh
  echo '> Removing /etc/systemd/system/gw@.service'
  sudo systemctl disable gw@$choice.service
  sudo rm /etc/systemd/system/gw@.service
  # If gw inf is a wlan then remove wpa_supplicant configuration
  if [[ $choice = wl* ]]; then
    echo "> Disabling wpa_supplicant-$choice.service"
    sudo systemctl disable wpa_supplicant@$choice.service
    echo "> Removing /etc/wpa_supplicant/wpa_supplicant-$choice.conf"
    sudo rm /etc/wpa_supplicant/wpa_supplicant-$choice.conf
  fi
}

function gw_menu {
  echo '> Do you wish to...'
  ap_arr=('Create a gateway?' 'Remove a gateway?')
  menu_from_array "${ap_arr[@]}"
  if [ $REPLY == "1" ]; then
    gw_create
  elif [[ $REPLY == "2" ]]; then
    gw_remove
  fi
}


#-----------------------------------------------------------------------------#
function server {
  echo '> Configuring the server...'
  pushd .
  cd ../..
  # Make the server
  make mesh_tb_server
  popd
  # Advertise the server using avahi (zeroconf)
  avahi-publish-service mesh_server _http._tcp 5000 &
  # Install the server and provision the certificate generated by cryptolib
  sudo python3 src/server-mesh.py -c src/ecc_key.der
}


function client {
  echo '> Configuring the client...'
  # Make the server
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
  while ! [ "$server_details" ] ; do
    server_details=$(timeout 7 avahi-browse -rptf _http._tcp | awk -F';' '$1 == "=" && $3 == "IPv4" && $4 == "mesh_server" {print $8 " " $7}')
  done
  # split ip/host into separate vars
  server_details=($(sed -r 's/\b.local\b//g' <<< $server_details))
  server_ip=${server_details[0]}
  server_host=${server_details[1]}
  echo "> We will use src/ecc_key.der if it already exists, or we can try and fetch it..."
  read -p "> Do you want to fetch the certificate from the server $server_host@$server_ip? (Y/N): " confirm
  if [[ $confirm == [yY] || $confirm == [yY][eE][sS] ]]; then
    echo '> Fetching certificate from server...'
    read -p "- Server Username: " server_user
    # pull the key from the server
    scp $server_user@$server_ip:/home/$server_user/mesh_com/modules/sc-mesh-secure-deployment/src/ecc_key.der src/ecc_key.der
  fi

  echo '> Configuring the client and connecting to server...'
  sudo python3 src/client-mesh.py -c src/ecc_key.der -s http://$server_ip:5000
}

#-----------------------------------------------------------------------------#
echo '=== sc-mesh-secure-deployment-configure ==='
if [[ $# -eq 0 ]] ; then
    help
    exit 0
fi
PARAMS=""
while (( "$#" )); do
  case "$1" in
    -s)
      server
      shift
      ;;
    -c)
      client
      NOAP=1
      shift
      ;;
    -ap)
      ap_menu
      shift
      ;;
    -gw)
      gw_menu
      shift
      ;;
    --help)
      help
      shift 2
      ;;
    *) # preserve positional arguments
      PARAMS="$PARAMS $1"
      shift
      ;;
  esac
done
