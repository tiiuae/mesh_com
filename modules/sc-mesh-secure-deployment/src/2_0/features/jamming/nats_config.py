from log_config import logger

"""
This file contains the configuration for the NATS client.
"""
try:
    from identity import MODULE_IDENTITY as IDENTITY
except ImportError:
    IDENTITY = "no_identity"
    logger.error("No identity found!")

MODULE_IP = "10.10.20.2"  # or 192.168.1.x - brlan ip address
MODULE_PORT = "4222"

MODULE_ROLE = "sleeve"  # drone, sleeve or gcs
MODULE_IDENTITY = IDENTITY  # messages are sent to this device
