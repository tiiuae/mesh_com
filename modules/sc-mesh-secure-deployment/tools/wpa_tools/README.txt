*** To connect to the access point ***
(Be sure to set the correct SSID and password)

run:
wpa_supplicant -B -i <interface> -c wpa_supplicant_client_AP.conf

*** To create an access point ***

run:
chmod +x access_point_wpa_supplicant.sh
./access_point_wpa_supplicant.sh

