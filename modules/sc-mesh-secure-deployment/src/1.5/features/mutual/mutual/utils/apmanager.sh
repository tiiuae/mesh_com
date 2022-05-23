#!/bin/bash
#-----------------------------------------------------------------------------#
[ ! -d "conf" ] && mkdir conf
function ap_connect {
  #read -p "- Password: " password
  cat <<EOF > conf/ap.conf
  network={
    ssid="$1"
    psk="ssrcpassword"
  }
EOF
  echo '> Connecting to Access Point:'
  echo $ssid
  sudo wpa_supplicant -B -i wlan0 -c conf/ap.conf
  sudo dhclient -v wlan0 #to change to uhdpd on secure OS
}
function ap_create {
echo '> Creating an Access Point...'
cat <<EOF > conf/ap.conf
  network={
    ssid="AuthAP_$1"
    mode=2
    key_mgmt=WPA-PSK
    psk="ssrcpassword"
    frequency=2437
}
EOF
# FIXME: Each time you echo this it's gonna add to the end of the files and
#        create duplicate lines
  echo "INTERFACES=wlan0" >> /etc/default/isc-dhcp-server
  if [ ! -d "/etc/mesh_com" ]; then
     mkdir /etc/mesh_com
  fi
  #echo "AP_INF=$choice" >> /etc/mesh_com/ap.conf
  # Create Gateway Service
  cp utils/mesh-ap.sh /usr/sbin/.
  chmod 744 /usr/sbin/mesh-ap.sh
  cp ../../../services/initd/S93meshAP /etc/init.d/.
  chmod 700 /etc/init.d/S93meshAP
  /etc/init.d/S93meshAP start wlan0
  cp conf/ap.conf /etc/wpa_supplicant/wpa_supplicant-wlan0.conf
}

# shellcheck disable=SC1073
function ap_remove {
  echo '> Remove an Access Point...'
  echo '> Please choose from the list of available interfaces...'
  interfaces_arr=$(ip link | awk -F: '$0 !~ "lo|vir|doc|eth|bat|^[^0-9]"{print $2}')
  menu_from_array "${interfaces_arr[@]}"
  /etc/init.d/S93meshAP stop
  # sudo systemctl enable ap@$ip.service
  sudo rm /etc/wpa_supplicant/wpa_supplicant-$choice.conf
  sudo rm /usr/sbin/mesh-ap.sh
  sudo rm /etc/init.d/S93meshAP
  sudo rm /etc/mesh_com/ap.conf
#  reboot
}
PARAMS=''
while (( "$#" )); do
  case "$1" in
    -ap_remove)
      ap_remove
      shift
      ;;
    -ap_connect)
      ap_connect $2
      shift
      ;;
    -ap_create)
      ap_create $2
      shift
      ;;
    *) # preserve positional arguments
      PARAMS="$PARAMS $1"
      shift
      ;;
  esac
done