
#! /bin/bash


create_dhcpd_config()
{
  SUBNET="$1"

  cat > /etc/dhcp/dhcpd.conf <<- EOF
  default-lease-time 600;
  max-lease-time 7200;
  ddns-update-style none;
  authoritative;

  subnet $SUBNET.0 netmask 255.255.255.0 {
    range $SUBNET.2 $SUBNET.254;
    option routers $SUBNET.1;
  }
EOF
  cp /dev/null /var/lib/dhcp/dhcpd.leases
}

create_olsrd_config()
{
  wifidev="$1"
  SUBNET="$2"
  cat > /etc/olsrd/olsrd.conf <<- EOF
  LinkQualityFishEye   0

  Interface "$WIFI_INTERFACE"
  {
  }

  IpVersion               4
  LinkQualityFishEye      0
  LinkQualityAlgorithm "etx_ffeth_nl80211"

  LoadPlugin "/usr/lib/olsrd_arprefresh.so.0.1"  
  {
  }

  Hna4
  {
    $SUBNET.0 255.255.255.0
  }
EOF
}

create_olsrd_config6()
{
  wifidev="$1"
  IPV6_PREFIX="$2"
  cat > /etc/olsrd/olsrd6.conf <<- EOF
  LinkQualityFishEye   0

  Interface "$WIFI_INTERFACE"
  {
  }

  IpVersion               6
  LinkQualityFishEye      0
  LinkQualityAlgorithm "etx_ffeth_nl80211"

  LoadPlugin "/usr/lib/olsrd_arprefresh.so.0.1"  
  {
  }
  
  Hna6
  {
    $IPV6_PREFIX:0 64
  }
EOF
}


create_radvd_config()
{
  IPV6_PREFIX="$1"
  cat > /etc/radvd.conf <<- EOF
  interface br-lan
  {
    AdvSendAdvert on;
    MinRtrAdvInterval 3;
    MaxRtrAdvInterval 10;
    prefix $IPV6_PREFIX:0/64 {
    };
  };
EOF
}
