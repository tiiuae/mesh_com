"""
State store for DownloadStatus
"""
import yaml
from typing import Any
from . import constants # for testing

class ConfigStore:
    """
    YAML DownloadStatus
    """
    def __init__(self, filename: str = None):
        self.filename = filename
        self.state = {}

        try:
            with open(self.filename, 'r', encoding="utf8") as file:
                self.state = yaml.safe_load(file) or {}
        except FileNotFoundError:
            pass

    def store(self, key: str, value: Any):
        """
        Store a value in the state file
        param key: Key to store
        param value: Value to store
        return: None
        """
        self.state[key] = value

        with open(self.filename, 'w', encoding="utf8") as file:
            yaml.dump(self.state, file)

    def read(self, key: str) -> Any:
        """
        Read a value from the state file
        param key: Key to read
        return: Value of the key or None if not found
        """
        try:
            return self.state[key]
        except (KeyError, AttributeError):
            return None

if __name__ == '__main__':
    # Example usage

    state_manager = ConfigStore("/tmp/test.yaml")
    print(state_manager.read(constants.ConfigType.MESH_CONFIG.value))
    state_manager.store(constants.ConfigType.DEBUG_CONFIG.value, True)
    print(state_manager.read(constants.ConfigType.DEBUG_CONFIG.value))
