#!/bin/bash

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
sudo wpa_supplicant -B -i $choice -c ap.conf
sudo dhclient -v $choice
}

ap_connect