#!/bin/bash
#-----------------------------------------------------------------------------#

[ ! -d "conf" ] && mkdir conf

#assuming we are running from main (1_5)
feature_folder=$(dirname "$(pwd)")
features_folder=$(dirname $feature_folder)
folder=$(dirname $features_folder)
src=$(dirname $folder)


[ ! -d "/etc/wpa_supplicant/" ] && mkdir "/etc/wpa_supplicant/"

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


function ap_connect {
killall wpa_supplicant
default $1 $2
ifconfig $choice up
if [[ "$2" == "-ci" ]]; then
  password = "root"
  echo "- Password: $password"
  # AP hostapd config
cat <<EOF > conf/ap.conf
  network={
    ssid="WirelessLab"
    psk="ssrcpassword"
}
EOF
else
cat <<EOF > conf/ap.conf
  network={
    ssid="$ssid"
    psk="$password"
  }
EOF
fi
echo '> Connecting to Access Point:'
echo $ssid
wpa_supplicant -B -i $sta_if -c conf/ap.conf
#/usr/sbin/hostapd -B /var/run/hostapd.conf -f /tmp/hostapd.log
sleep 2
execution_ctx=$(echo $HOSTNAME)
if [ $execution_ctx = "br_hardened" ]; then
    udhcpc -i $sta_if
else
    dhclient -v $sta_if
fi
}

function ap_create {
default $1 $2
echo '> Creating an Access Point...'
#command_exists 'isc-dhcp-server net-tools'
cat <<EOF >/var/run/hostapd.conf
country_code=AE
interface=$choice
ssid=$ssid
hw_mode=g
channel=7
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=$password
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
EOF

if [ ! -d "/etc/mesh_com" ]; then
   mkdir /etc/mesh_com
fi
#echo "AP_INF=$choice" >> /etc/mesh_com/ap.conf
# Create Gateway Service
cp $src/common/scripts/mesh-ap.sh /usr/sbin/.
chmod 744 /usr/sbin/mesh-ap.sh
cp $folder/sc-mesh-secure-deployment/services/initd/S94meshAP /etc/init.d/.
chmod 700 /etc/init.d/S94meshAP
/etc/init.d/S94meshAP start $choice $ip
echo "start :"  $choice $ip
#cp conf/ap.conf /etc/wpa_supplicant/wpa_supplicant-$choice.conf
ifconfig $choice $ip netmask 255.255.255.0
killall dhcpd
}

# shellcheck disable=SC1073
function ap_remove {
  default $1 $2
  subnet=$(awk -F"." '{print $1"."$2"."$3".0"}'<<<$2)
  echo '> Remove an Access Point...'
  /etc/init.d/S94meshAP stop
  # sudo systemctl enable ap@$ip.service
  sudo rm /etc/wpa_supplicant/wpa_supplicant-$choice.conf
  sudo rm /usr/sbin/mesh-ap.sh
  sudo rm /etc/init.d/S94meshAP
  sudo rm /etc/mesh_com/ap.conf
  ip route del $subnet/24 via 0.0.0.0 dev $choice
  ip route del 0.0.0.0/0 via $ip dev $choice

#  reboot
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


function default() {
  if [ -z "$1" ] # SSID, password, IP
  then
    read -p "- SSID: " ssid
    #read -p "- Password: " password
    password=$(systemd-ask-password "Please type your Password:")
    read -p "- Custom IP (e.g. XX.0.0.1): " ip
  else
      if [[ $1 == "AuthAP"* ]];
        then
          ssid=$1
      else
        ssid="AuthAP_$1"
      fi
      password="ssrcpassword"
      ip='50.10.10.1'
fi
if [ -z "$2" ] # interface
  then
    echo '> Please choose from the list of available interfaces...'
    interfaces_arr=($(ip link | awk -F: '$0 !~ "lo|vir|doc|eth|bat|^[^0-9]"{print $2}'))
    menu_from_array "${interfaces_arr[@]}"
    sta_if=$choice
  else
    choice=$2
    sta_if=$choice
fi
}

PARAMS=''
while (( "$#" )); do
  case "$1" in
    -ap_remove)
      ap_remove  $2 $3
      shift
      ;;
    -ap_connect)
      ap_connect $2 $3
      shift
      ;;
    -ap_create)
      ap_create $2 $3
      shift
      ;;
    *) # preserve positional arguments
      PARAMS="$PARAMS $1"
      shift
      ;;
  esac
done