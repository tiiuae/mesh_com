#!/bin/bash
echo " ----------------- Initiating Clean Up and Access point Connectivity----------------------------------------"
if [ -f ap.conf ]; then
   rm ap.conf
   echo " Removing existing ap.conf file"
fi
read -p 'SSID: ' SSID
read -p 'Password: ' PSWD
touch ap.conf
#chmod 777 ap.conf
echo "network={
    ssid=\"$SSID\"
    psk=\"$PSWD\"
}" >> ap.conf
 echo " New ap.conf file created with desired SSID and Password set "
 echo " Cleanup Started ---  Provisioning " 
killall hostapd
killall wpa_supplicant
ifconfig br-lan down
brctl delbr br-lan
ifname_ap="$(ifconfig -a | grep wlan* | awk -F':' '{ print $1 }')"
ifname_mp="$(ifconfig -a | grep wlp1* | awk -F':' '{ print $1 }')"
ifconfig $ifname_ap down
ifconfig $ifname_mp down
ifconfig $ifname_ap up
ifconfig $ifname_mp up
wpa_supplicant -B -i wlan1 -c ap.conf
dhclient -v wlan1
