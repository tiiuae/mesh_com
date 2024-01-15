import pytest
from unittest.mock import patch, MagicMock
import threading

import add_syspath
from decision_engine import DecisionEngine

@pytest.fixture
def decision_engine_setup():
    with patch('decision_engine.MBA') as mock_mba, \
         patch('decision_engine.Quarantine') as mock_quarantine, \
         patch('decision_engine.logger', new_callable=MagicMock) as mock_logger:
        engine = DecisionEngine(mba_multicast_group="ff02::1", mba_port=12345, mba_interface="eth0",
                                my_cert_dir="/path/to/certs", peer_cert_dir="/path/to/peer/certs", quarantine_period=3600)
        engine.mba = mock_mba
        yield engine, mock_quarantine, mock_logger

def test_init(decision_engine_setup):
    engine, _, _ = decision_engine_setup
    assert engine.quarantine_period == 3600
    assert isinstance(engine.mba, MagicMock)
    assert isinstance(engine.mba_receiver_thread, threading.Thread)

def test_update_with_ids_report(decision_engine_setup):
    engine, mock_quarantine, _ = decision_engine_setup
    report = {'feature': 'IDS', 'malicious': True, 'ip': '192.168.1.1', 'mac': '00:1B:44:11:3A:B7'}
    engine.update(report)
    mock_quarantine.assert_called_once()
    engine.mba.send_mba.assert_called_once()

def test_update_with_phy_report(decision_engine_setup):
    engine, mock_quarantine, _ = decision_engine_setup
    report = {'feature': 'PHY', 'result': 'fail', 'ip': '192.168.1.2', 'mac': '00:1B:44:11:3A:B8'}
    engine.update(report)
    mock_quarantine.assert_called_once()
    engine.mba.send_mba.assert_called_once_with(mac="00:1B:44:11:3A:B8", ip="192.168.1.2")

def test_update_with_rss_report(decision_engine_setup):
    engine, mock_quarantine, _ = decision_engine_setup
    report = {'feature': 'RSS', 'result': 'fail', 'ip': '192.168.1.3', 'mac': '00:1B:44:11:3A:B9'}
    engine.update(report)
    mock_quarantine.assert_called_once()
    engine.mba.send_mba.assert_called_once_with(mac="00:1B:44:11:3A:B9", ip="192.168.1.3")

def test_update_with_mba_report(decision_engine_setup):
    engine, _, _ = decision_engine_setup
    report = {'module': 'MBA', 'ip': '192.168.1.4', 'mac': '00:1B:44:11:3A:BA'}
    engine.update(report)
    # Assertions for MBA report handling

def test_respond_to_malicious_peer_new(decision_engine_setup):
    engine, _, _ = decision_engine_setup
    engine.respond_to_malicious_peer("192.168.1.5", "00:1B:44:11:3A:BB")
    assert "192.168.1.5" in engine.blocked_ips
    engine.mba.send_mba.assert_called_once_with(mac="00:1B:44:11:3A:BB", ip="192.168.1.5")

def test_respond_to_malicious_peer_existing(decision_engine_setup):
    engine, _, _ = decision_engine_setup
    engine.blocked_ips.add("192.168.1.6")
    engine.respond_to_malicious_peer("192.168.1.6", "00:1B:44:11:3A:BC")
    engine.mba.send_mba.assert_not_called()  # Assuming no action for already blocked IP

def test_stop(decision_engine_setup):
    engine, _, _ = decision_engine_setup
    engine.stop()
    assert engine.stop_event.is_set()
