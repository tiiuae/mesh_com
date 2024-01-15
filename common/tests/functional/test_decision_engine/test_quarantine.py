import pytest
import os
import pandas as pd
from tempfile import TemporaryDirectory

import add_syspath
from quarantine import Quarantine
from unittest.mock import MagicMock, patch

@pytest.fixture
def quarantine_env():
    with TemporaryDirectory() as temp_dir:
        blacklist_file = os.path.join(temp_dir, "blacklist.csv")
        yield temp_dir, blacklist_file

def test_add_to_blacklist(quarantine_env):
    temp_dir, blacklist_file = quarantine_env
    quarantine = Quarantine("00:11:22:33:44:55", "192.168.1.1", 3600, MagicMock(), set(), blacklist_dir=temp_dir)

    quarantine.add_to_blacklist()
    assert os.path.isfile(blacklist_file)
    df = pd.read_csv(blacklist_file)
    assert len(df) == 1
    assert df['IP'][0] == "192.168.1.1"


def test_remove_from_blacklist(quarantine_env):
    temp_dir, blacklist_file = quarantine_env
    quarantine = Quarantine("00:11:22:33:44:55", "192.168.1.1", 3600, MagicMock(), set(), blacklist_dir=temp_dir)

    quarantine.add_to_blacklist()
    df = pd.read_csv(blacklist_file)
    assert len(df) == 1
    quarantine.remove_from_blacklist()
    df = pd.read_csv(blacklist_file)
    assert df.empty

class MockTimer:
    def __init__(self, interval, function):
        self.interval = interval
        self.function = function
        self.args = ()
        self.kwargs = {}
        self.is_started = False

    def start(self):
        self.is_started = True
        self.function(*self.args, **self.kwargs)

@patch("quarantine.threading.Timer", side_effect=MockTimer)
@patch("quarantine.logger")
def test_start_quarantine(mock_logger, mock_timer, quarantine_env):
    temp_dir, blacklist_file = quarantine_env
    quarantine = Quarantine("00:11:22:33:44:55", "192.168.1.1", 2, MagicMock(), set(), blacklist_dir=temp_dir)
    quarantine.add_to_blacklist = MagicMock()
    quarantine.end_quarantine = MagicMock()

    quarantine.start_quarantine()

    # Verify add_to_blacklist is called
    quarantine.add_to_blacklist.assert_called_once()

    # Verify the Timer was set correctly
    assert mock_timer.call_args[0][0] == 2  # quarantine_period
    assert mock_timer.call_args[0][1] == quarantine.end_quarantine

    # Simulate the timer elapsing
    mock_timer.return_value.start()

    # Verify end_quarantine is called
    quarantine.end_quarantine.assert_called_once()

@patch("quarantine.logger")
def test_end_quarantine(mock_logger, quarantine_env):
    temp_dir, blacklist_file = quarantine_env
    blacklist_lock = MagicMock()
    blocked_ips = {"192.168.1.1", "192.168.1.2"}
    quarantine = Quarantine("00:11:22:33:44:55", "192.168.1.1", 3600, blacklist_lock, blocked_ips, blacklist_dir=temp_dir)

    # Mocking the remove_from_blacklist method
    quarantine.remove_from_blacklist = MagicMock()

    # Call end_quarantine and check if the node is removed from blocked_ips
    quarantine.end_quarantine()

    assert "192.168.1.1" not in quarantine.blocked_ips
    quarantine.remove_from_blacklist.assert_called_once()

# Remove log directory during teardown
import shutil
import os
def remove_logs_directory():
    logs_directory = 'logs'
    if os.path.exists(logs_directory) and os.path.isdir(logs_directory):
        shutil.rmtree(logs_directory)

def teardown_module():
    remove_logs_directory()