import unittest
from unittest.mock import patch, mock_open
from src.comms_config_store import ConfigStore

class TestConfigStore(unittest.TestCase):
    def test_store_and_read_values(self):
        with patch("builtins.open", mock_open()) as mock_file:
            config_store = ConfigStore("test.yaml")
            config_store.store("test_key", "test_value")
            assert config_store.read("test_key") == "test_value"
            mock_file.assert_called_with("test.yaml", 'w', encoding="utf8")

    def test_read_non_existent_key_returns_none(self):
        with patch("builtins.open", mock_open()) as mock_file:
            config_store = ConfigStore("test.yaml")
            assert config_store.read("non_existent_key") == None

    def test_store_and_read_multiple_values(self):
        with patch("builtins.open", mock_open()) as mock_file:
            config_store = ConfigStore("test.yaml")
            config_store.store("test_key1", "test_value1")
            config_store.store("test_key2", "test_value2")
            assert config_store.read("test_key1") == "test_value1"
            assert config_store.read("test_key2") == "test_value2"

    def test_read_from_non_existent_file(self):
        with patch("builtins.open", mock_open()) as mock_file:
            mock_file.side_effect = FileNotFoundError
            config_store = ConfigStore("non_existent_file.yaml")
            assert config_store.read("any_key") == None