import queue
import pytest
from unittest.mock import Mock, patch, MagicMock

import add_syspath
from tools.monitoring_wpa import WPAMonitor

@pytest.fixture
def mock_ctrl_path():
    return "/path/to/ctrl"



def test_handle_event_handles_mesh_peer_connected(mock_ctrl_path):
    event_queue = queue.Queue()
    monitor = WPAMonitor(mock_ctrl_path)
    monitor._handle_event("MESH-PEER-CONNECTED some_mac_address", event_queue)

    # Assert that the event was logged and added to the queue
    #mock_queue.put.assert_called_once_with(("WPA", "some_mac_address"))
    assert event_queue.get() == ("WPA", "some_mac_address")

# Remove log directory during teardown
import shutil
import os
def remove_logs_directory():
    logs_directory = 'logs'
    if os.path.exists(logs_directory) and os.path.isdir(logs_directory):
        shutil.rmtree(logs_directory)

def teardown_module():
    remove_logs_directory()