"""
Commonly used comms settings/command definitions
"""

class STATUS:  # pylint: disable=too-few-public-methods
    """
    Comms status values
    """
    no_status: str = "MESH_NO_STATUS"  # no stat available
    mesh_default: str = "MESH_DEFAULT"  # mesh default settings
    mesh_default_connected: str = "MESH_DEFAULT_CONNECTED"  # mesh default connected
    mesh_cfg_not_stored: str = "MESH_CONFIGURATION_NOT_STORED"  # mesh configuration not stored
    mesh_cfg_stored: str = "MESH_CONFIGURATION_PENDING"  # new mission onfiguration stored but not active
    mesh_cfg_applied: str = "MESH_CONFIGURATION_APPLIED"  # mission mesh configuration has been applied
    mesh_cfg_tampered: str = "MESH_CONFIGURATION_TAMPERED"  # mission mesh configuration file doesn't match with hash
    mesh_mission_not_connected: str = "MESH_MISSION_NOT_CONNECTED"  # mesh mission not connected
    mesh_mission_connected: str = "MESH_MISSION_CONNECTED"  # mesh mission connected
    mesh_fail: str = "MESH_FAIL"  # fails

    security_provisioned: str = "SECURITY_PROVISIONED"  # security provisioned
    security_non_provisioned: str = "SECURITY_NON_PROVISIONED"  # security non provisioned
    security_compromised: str = "SECURITY_COMPROMISED"
    # TBD add more status

class COMMAND:  # pylint: disable=too-few-public-methods
    """
    Comms control commands
    """
    revoke: str = "REVOKE"   # Activate default mesh and revoke/delete mission keys
    apply: str = "APPLY"     # Take mission config into use
    wifi_down: str = "DOWN"  # Deactivate Wi-Fi transmitter
    wifi_up: str = "UP"      # Activate Wi-Fi transmitter
    reboot: str = "REBOOT"   # Reboot comms module
    get_logs: str = "LOGS"   # Command to send basic logs as response
    enable_visualisation: str = "ENABLE_VISUALISATION"
    disable_visualisation: str = "DISABLE_VISUALISATION"
    get_config: str = "GET_CONFIG"
    get_identity: str = "GET_IDENTITY"
    debug: str = "DEBUG"     # DEBUG command allows to run any subprocess, it is disabled by default
