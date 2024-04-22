"""
This module defines the FsmEvent class which is used to represent an event in the FSM.
"""
import enum
from typing import Any


class Events(enum.Enum):
    # MDM
    MDM_START_EVENT: str = "MDM_START_EVENT"
    MDM_STOP_EVENT: str = "MDM_STOP_EVENT"

    # Callbacks
    MDM_SERVICE_MONITOR_CB: str = "MDM_SERVICE_MONITOR_CB"
    MDM_INTERFACE_MONITOR_CB: str = "MDM_INTERFACE_MONITOR_CB"

    # MDM config actions
    MDM_CONFIG_EVENT: str = "MDM_CONFIG_EVENT"
    MDM_CERTIFICATES_EVENT: str = "MDM_CERTIFICATES_EVENT"
    MDM_FEATURES_EVENT: str = "MDM_FEATURES_EVENT"
    MDM_CERTIFICATES_UPLOAD_EVENT: str = "MDM_CERTIFICATES_UPLOAD_EVENT"
    MDM_CERTIFICATES_DOWNLOAD_EVENT: str = "MDM_CERTIFICATES_DOWNLOAD_EVENT"
    MDM_HANDLER_TIMEOUT_EVENT: str = "MDM_HANDLER_TIMEOUT_EVENT"


class Event(object):
    def __init__(self, event_name: str, event_data: Any, priority: int = 0):
        self.event_name: str = event_name
        self.event_data: Any = event_data
        self.priority: int = priority

    def __str__(self) -> str:
        return f"Event: {self.event_name}, Data: {self.event_data}"

    def __repr__(self) -> str:
        return f"Event: {self.event_name}, Data: {self.event_data}"

