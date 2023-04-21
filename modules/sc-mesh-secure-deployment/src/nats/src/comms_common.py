"""
Commonly used comms settings/command definitions
"""
from enum import Enum


class STATUS(Enum):  # pylint: disable=too-few-public-methods
    """
    Comms status values
    """
    no_status = "MESH_NO_STATUS"  # no stat available
    mesh_default = "MESH_DEFAULT"  # mesh default settings
    mesh_default_connected = "MESH_DEFAULT_CONNECTED"  # mesh default connected
    mesh_configuration_not_stored = "MESH_CONFIGURATION_NOT_STORED"  # mesh configuration not stored
    mesh_configuration_stored = "MESH_CONFIGURATION_STORED"  # mesh configuration stored
    mesh_mission_not_connected = "MESH_MISSION_NOT_CONNECTED"  # mesh mission not connected
    mesh_mission_connected = "MESH_MISSION_CONNECTED"  # mesh mission connected
    wifi_off = "WIFI_OFF"  # Wi-Fi transmitter is deactivated
    wifi_on = "WIFI_ON"  # Wi-Fi transmitter is active
    visualisation_enabled = "VISUALISATION_ENABLED"
    visualisation_disabled = "VISUALISATION_DISABLED"
    invalid_command = "INVALID_COMMAND"  # Invalid/not supported command
    mesh_fail = "MESH_FAIL"  # fails


class COMMAND:  # pylint: disable=too-few-public-methods
    """
    Comms control commands
    """
    revoke = "REVOKE"   # Activate default mesh and revoke/delete mission keys
    apply = "APPLY"     # Take mission config into use
    wifi_down = "DOWN"  # Deactivate Wi-Fi transmitter
    wifi_up = "UP"      # Activate Wi-Fi transmitter
    reboot = "REBOOT"   # Reboot comms module
    get_logs = "LOGS"   # Command to send basic logs as response
    enable_visualisation = "ENABLE_VISUALISATION"
    disable_visualisation = "DISABLE_VISUALISATION"
