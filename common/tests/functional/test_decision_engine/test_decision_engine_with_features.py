import pytest
from unittest.mock import MagicMock, patch

import add_syspath
from decision_engine import DecisionEngine
from observable_module import ObservableModule

import threading

import time

# Mock feature classes
class MockPHYCRA(ObservableModule):
    def __init__(self, decision_engine):
        super().__init__(decision_engine)
        self.stop_event = threading.Event()

    def check(self):
        # Simulated PHYCRA detection to notify decision engine
        ip = "192.168.1.1"
        mac = "00:00:00:00:00:11"
        self.notify({"feature": "PHY", "module": "sp_cra", "result": "fail", "ip": ip, "mac": mac})

class MockRSS_Auth(ObservableModule):
    def __init__(self, decision_engine):
        super().__init__(decision_engine)
        self.stop_event = threading.Event()
    def check(self):
        # Simulated RSS_Auth detection to notify decision engine
        ip = "192.168.1.2"
        mac = "00:00:00:00:00:22"
        self.notify({"feature": "RSS", "module": "rss_auth", "result": "fail", "ip": ip, "mac": mac})


class MockIDS(ObservableModule):
    def __init__(self, decision_engine):
        super().__init__(decision_engine)
        self.stop_event = threading.Event()

    def check_dpi(self):
        # Simulated DPI detection to notify decision engine
        ip = "192.168.1.3"
        mac = "00:00:00:00:00:33"
        self.notify({"feature": "IDS", "module": "DPI", "malicious": True, "ip": ip, "mac": mac})

    def check_mpi(self):
        # Simulated MPI detection to notify decision engine
        ip = "192.168.1.4"
        mac = "00:00:00:00:00:44"
        self.notify({"feature": "IDS", "module": "MPI", "malicious": True, "ip": ip, "mac": mac})


# Test setup
@pytest.fixture
def setup_decision_engine():
    with patch('decision_engine.MBA') as mock_mba, \
         patch('decision_engine.Quarantine') as mock_quarantine:
        decision_engine = DecisionEngine(mba_multicast_group='ff02::1', mba_port=12345, mba_interface="interface", quarantine_period=10, my_cert_dir="/path/to/my/certs", peer_cert_dir="/path/to/peer/certs")
        decision_engine.mba = mock_mba
        yield decision_engine, mock_quarantine

def test_phycra_interaction_with_decision_engine(setup_decision_engine):
    decision_engine, mock_quarantine = setup_decision_engine
    mock_phycra = MockPHYCRA(decision_engine)

    # Notify decision_engine from MockPHYCRA
    mock_phycra.check()
    # Verify Decision Engine's response to MockPHYCRA notification
    time.sleep(0.1) # Verify that Decision Engine responds within 100 ms
    assert "192.168.1.1" in decision_engine.blocked_ips # Assert that the notified IP is added to blocked_ips
    mock_quarantine.assert_called_once() # Assert that quarantine was called
    decision_engine.mba.send_mba.assert_called_once_with(mac="00:00:00:00:00:11", ip="192.168.1.1") # Assert that MBA was called
def test_rss_auth_interaction_with_decision_engine(setup_decision_engine):
    decision_engine, mock_quarantine = setup_decision_engine
    mock_rss_auth = MockRSS_Auth(decision_engine)

    # Notify decision_engine from MockRSS_Auth
    mock_rss_auth.check()
    # Verify Decision Engine's response to MockRSSAuth notification
    time.sleep(0.1) # Verify that Decision Engine responds within 100 ms
    assert "192.168.1.2" in decision_engine.blocked_ips # Assert that the notified IP is added to blocked_ips
    mock_quarantine.assert_called_once() # Assert that quarantine was called
    decision_engine.mba.send_mba.assert_called_once_with(mac="00:00:00:00:00:22", ip="192.168.1.2") # Assert that MBA was called

def test_ids_dpi_interaction_with_decision_engine(setup_decision_engine):
    decision_engine, mock_quarantine = setup_decision_engine
    mock_ids = MockIDS(decision_engine)

    # Notify decision_engine from MockIDS DPI module
    mock_ids.check_dpi()
    # Verify Decision Engine's response to MockRSSAuth notification
    time.sleep(0.1)  # Verify that Decision Engine responds within 100 ms
    assert "192.168.1.3" in decision_engine.blocked_ips  # Assert that the notified IP is added to blocked_ips
    mock_quarantine.assert_called_once()  # Assert that quarantine was called
    decision_engine.mba.send_mba.assert_called_once_with(mac="00:00:00:00:00:33", ip="192.168.1.3")  # Assert that MBA was called

def test_ids_mpi_interaction_with_decision_engine(setup_decision_engine):
    decision_engine, mock_quarantine = setup_decision_engine
    mock_ids = MockIDS(decision_engine)

    # Notify decision_engine from MockIDS MPI module
    mock_ids.check_mpi()
    # Verify Decision Engine's response to MockRSSAuth notification
    time.sleep(0.1)  # Verify that Decision Engine responds within 100 ms
    assert "192.168.1.4" in decision_engine.blocked_ips  # Assert that the notified IP is added to blocked_ips
    mock_quarantine.assert_called_once()  # Assert that quarantine was called
    decision_engine.mba.send_mba.assert_called_once_with(mac="00:00:00:00:00:44", ip="192.168.1.4")  # Assert that MBA was called



