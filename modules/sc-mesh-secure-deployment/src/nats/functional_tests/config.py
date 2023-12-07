"""
This file contains the configuration for the NATS client.
"""
try:
    from identity import MODULE_IDENTITY as IDENTITY
except ImportError:
    IDENTITY = "no_identity"
    print("No identity found!!!!!!!!!!!!!!!!!!!!!!!!")
    print("Please run _cli_command_get_identity.py to get the identity (creates identity.py)")

MODULE_IP = "10.10.20.2"   # - brlan ip address
MODULE_PORT = "4222"

# IPv6 connection example
# MODULE_IP = "[2001:db8:1234:5678:2e0:4cff:fe68:7a73]"
# MODULE_PORT = "6222"

MODULE_ROLE = "drone"  # drone, sleeve or gcs
MODULE_IDENTITY = IDENTITY  # messages are sent to this device
