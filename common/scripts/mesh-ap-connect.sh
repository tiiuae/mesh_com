#!/bin/bash


function ap_connect {
echo '> Connecting to Access Point...'
choice='wlan0'
sudo wpa_supplicant -B -i $choice -c ap.conf
sudo dhclient -v $choice
}

ap_connect