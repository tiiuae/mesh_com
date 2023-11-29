#! /bin/sh

echo "Starting the script..."

# Check if the number of arguments is not equal to 1
if [ 1 -ne $# ]; then
	# Print usage error message to stderr
	>&2 echo "Usage: $0 interface"
	exit 1
fi

# Store the first argument in INTERFACE variable
INTERFACE="$1"
echo "Configuring for interface: $INTERFACE"

# Check if the INTERFACE is set to 'test'
if [ 'test' = "$INTERFACE" ]; then
	echo "Test mode detected. Setting up a dummy interface..."
	# Enable debugging mode
	set -x
	# Exit immediately if a command exits with a non-zero status
	set -e
	TEST_OR_NOT='--configtest'
	# Create a dummy network interface named 'test'
	ip link add test type dummy
	# Set MAC address for the 'test' interface
	ip link set test address 00:20:91:4e:53:41
else
	TEST_OR_NOT=''
fi

# Define the path to the MAC address file for the given interface
MAC_ADDRESS_FILE="/sys/class/net/$INTERFACE/address"

# Check if the MAC address file has a valid MAC address format
if ! grep -Eiq '[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}' "$MAC_ADDRESS_FILE"; then
	>&2 echo "Error: can't get $INTERFACE's MAC address from $MAC_ADDRESS_FILE"
	exit 2
fi

echo "MAC address for $INTERFACE found and validated."

# Extract the MAC address and its segments
MAC_ADDRESS=`cat "$MAC_ADDRESS_FILE"`
AABB=`echo "$MAC_ADDRESS" | cut -d ':' -f 1,2 | tr -d ':'`
CCDD=`echo "$MAC_ADDRESS" | cut -d ':' -f 3,4 | tr -d ':'`
EEFF=`echo "$MAC_ADDRESS" | cut -d ':' -f 5,6 | tr -d ':'`

# Construct the IPv6 SLAAC prefix using the MAC address segments
# TODO:
# fd1d:8a65:4c32:e330::/64 let's say for WiFi QoS-OLSR
# fdbd:5175:25cd:bbf3::/64 let's say for HaLow QoS-OLSR
# fd2e:868e:e806:a27a::/64 let's say for all-radios batman-adv
IPV6_SLAAC_PREFIX="fdff:$AABB:$CCDD:$EEFF"
echo "Generated IPv6 SLAAC prefix: $IPV6_SLAAC_PREFIX"

# Define the radvd configuration file path for the given interface
RADVD_CONF_FILE="/run/radvd@$INTERFACE.conf"

# Create or clear the radvd configuration file
if ! cp /dev/null "$RADVD_CONF_FILE"; then
	>&2 echo "Error: can't create the configuration file"
	exit 3
fi

# Set permissions for the radvd configuration file
chmod 644 "/run/radvd@$INTERFACE.conf"

# Write the radvd configuration to the file
echo "Writing configuration to $RADVD_CONF_FILE..."
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
echo "Configuration written successfully."

# Check if the 'radvd' or 'nobody' user exists, or create the 'radvd' user
NOLOGIN=`which nologin` || NOLOGIN='/bin/false'
if grep -Eq '^radvd:' /etc/passwd; then
	SETUID_OR_NOT='--username radvd'
	echo "Using 'radvd' user for radvd daemon."
elif grep -Eq '^nobody:' /etc/passwd; then
	SETUID_OR_NOT='--username nobody'
	echo "Using 'nobody' user for radvd daemon."
else
	SETUID_OR_NOT=''
fi

# Define the radvd PID file path for the given interface
RADVD_PID_FILE="/var/run/radvd@$INTERFACE.pid"

# Start the radvd daemon with the specified configuration and options
echo "Starting radvd daemon..."
radvd --config "$RADVD_CONF_FILE" --pidfile "$RADVD_PID_FILE" $CHROOT_OR_NOT $SETUID_OR_NOT $TEST_OR_NOT

# If the INTERFACE is 'test', clean up and perform additional checks
if [ 'test' = "$INTERFACE" ]; then
	echo "Cleaning up test interface..."
	# Delete the 'test' interface
	ip link del test
	# Check the integrity of the radvd configuration file
	echo "Validating configuration file integrity..."
	echo "694ba3b642a2d5ee7154bbeb3929c53727651eda2361823065aa3c2cb3cedab3  $RADVD_CONF_FILE" | sha256sum --check
	echo 'All good! Test completed successfully. :-)'
fi

echo "Script execution completed."
