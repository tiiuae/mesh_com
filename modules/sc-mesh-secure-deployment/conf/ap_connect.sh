#!/bin/bash

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
if printf '%s\n' "${interfaces_arr[@]}" | grep -q -P '^wlan0$'; then
  choice='wlan0'
else
  menu_from_array "${interfaces_arr[@]}"
fi
sudo wpa_supplicant -B -i $choice -c ap.conf
sudo dhclient -v $choice
}

ap_connect