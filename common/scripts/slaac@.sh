#! /bin/sh



if [ 1 -ne $# ]; then
	>&2 echo "Usage: $0 interface"
	exit 1
fi
INTERFACE="$1"



if [ 'test' = "$INTERFACE" ]; then
	set -x
	set -e
	TEST_OR_NOT='--configtest'
	ip link add test type dummy
	ip link set test address 00:20:91:4e:53:41
else
	TEST_OR_NOT=''
fi



MAC_ADDRESS_FILE="/sys/class/net/$INTERFACE/address"
if ! grep -Eiq '[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}' "$MAC_ADDRESS_FILE"; then
	>&2 echo "Error: can't get $INTERFACE's MAC address from $MAC_ADDRESS_FILE"
	exit 2
fi
MAC_ADDRESS=`cat "$MAC_ADDRESS_FILE"`
AABB=`echo "$MAC_ADDRESS" | cut -d ':' -f 1,2 | tr -d ':'`
CCDD=`echo "$MAC_ADDRESS" | cut -d ':' -f 3,4 | tr -d ':'`
EEFF=`echo "$MAC_ADDRESS" | cut -d ':' -f 5,6 | tr -d ':'`
IPV6_SLAAC_PREFIX="fdff:$AABB:$CCDD:$EEFF"



RADVD_CONF_FILE="/run/radvd@$INTERFACE.conf"
if ! cp /dev/null "$RADVD_CONF_FILE"; then
	>&2 echo "Error: can't create the configuration file"
	exit 3
fi
chmod 644 "/run/radvd@$INTERFACE.conf"
cat > "$RADVD_CONF_FILE" << EOF
interface $INTERFACE
{
        IgnoreIfMissing on;
        AdvSendAdvert on;
        MaxRtrAdvInterval 60;
        AdvDefaultLifetime 0;

        prefix $IPV6_SLAAC_PREFIX::/64
        {
                AdvAutonomous on;
                AdvValidLifetime 7200;
                AdvPreferredLifetime 1200;
                DeprecatePrefix on;
        };

        route fdff::/16
        {
                AdvRoutePreference high;
                RemoveRoute on;
        };
};
EOF



NOLOGIN=`which nologin` || NOLOGIN='/bin/false'
if grep -Eq '^radvd:' /etc/passwd || useradd --home-dir /nonexistent --shell "$NOLOGIN" radvd; then
	SETUID_OR_NOT='--username radvd'
elif grep -Eq '^nobody:' /etc/passwd; then
	SETUID_OR_NOT='--username nobody'
else
	SETUID_OR_NOT=''
fi



if [ 'test' = "$INTERFACE" ]; then
	TEST_OR_NOT='--configtest'
else
	TEST_OR_NOT=''
fi



RADVD_PID_FILE="/run/radvd@$INTERFACE.pid"
radvd --config "$RADVD_CONF_FILE" --pidfile "$RADVD_PID_FILE" $CHROOT_OR_NOT $SETUID_OR_NOT $TEST_OR_NOT



if [ 'test' = "$INTERFACE" ]; then
	ip link del test
	echo "694ba3b642a2d5ee7154bbeb3929c53727651eda2361823065aa3c2cb3cedab3  $RADVD_CONF_FILE" | sha256sum --check
	echo 'All good! :-)'
fi
