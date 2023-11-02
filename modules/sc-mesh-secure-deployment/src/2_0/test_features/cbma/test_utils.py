import pytest
from unittest.mock import patch, Mock, call

import add_syspath
from tools.utils import *

def test_is_ipv4():
    assert is_ipv4("192.168.1.1")
    assert not is_ipv4("fe80::230:1aff:fe4f:cf3c")

def test_is_ipv6():
    assert is_ipv6("fe80::230:1aff:fe4f:cf3c")
    assert not is_ipv6("192.168.1.1")

def test_mac_to_ipv6():
    assert mac_to_ipv6("00:30:1a:4f:cf:3c") == "fe80::230:1aff:fe4f:cf3c"
    assert mac_to_ipv6("04:f0:21:9e:6b:39") == "fe80::6f0:21ff:fe9e:6b39"

def test_extract_mac_from_ipv6():
    assert extract_mac_from_ipv6("fe80::230:1aff:fe4f:cf3c") == "00:30:1a:4f:cf:3c"
    assert extract_mac_from_ipv6("fe80::6f0:21ff:fe9e:6b39") == "04:f0:21:9e:6b:39"

@patch('tools.utils.subprocess.check_output')
def test_is_wpa_supplicant_running(mock_check_output):
    # Case where wpa_supplicant is running
    mock_check_output.return_value = b'1234 ?        00:00:02 wpa_supplicant'
    assert is_wpa_supplicant_running()

    # Case where wpa_supplicant is not running
    mock_check_output.return_value = b''
    assert not is_wpa_supplicant_running()

@pytest.mark.parametrize('rules_file', ['custom1.nft', 'custom2.nft'])
def test_apply_nft_rules(rules_file):
    # Mock subprocess.run to simulate successful execution
    with patch('tools.utils.subprocess.run', autospec=True) as mock_run:
        # Mock logger.info
        with patch('tools.utils.logger.info', autospec=True) as mock_logger_info:
            apply_nft_rules(rules_file)
            mock_run.assert_called_once_with(['nft', '-f', rules_file], check=True)
def test_batman_exec():
    with patch('tools.utils.run_batman', autospec=True) as mock_run_batman:
        batman_exec("bat0", "batman-adv")
        # assert that run_batman was called once with the correct interface
        mock_run_batman.assert_called_once_with("bat0")

def test_run_batman():
    # Mocking subprocess.run to always return success
    with patch('tools.utils.subprocess.run', autospec=True) as mock_run:
        # Mocking get_mac_addr to return a dummy MAC address
        with patch('tools.utils.get_mac_addr', return_value='00:11:22:33:44:55', autospec=True) as mock_get_mac_addr:
            # Mock logger.info
            with patch('tools.utils.logger.info', autospec=True) as mock_logger_info:
                run_batman("bat0")

            # Verifying get_mac_addr was called with correct interface
            mock_get_mac_addr.assert_called_once_with("wlp1s0")

            # Verifying subprocess.run was called with the expected commands
            mock_run.assert_any_call(["ip", "link", "set", "dev", "bat0", "address", "00:11:22:33:44:55"], check=True)
            mock_run.assert_any_call(["ifconfig", "bat0", "up"], check=True)
            mock_run.assert_any_call(["ifconfig", "bat0", "mtu", "1460"], check=True)
            mock_run.assert_any_call(["ifconfig", "bat0"], check=True)

def test_set_ipv6():
    # Dummy result to mimic the subprocess.run successful result
    mock_result = Mock()
    mock_result.stdout = "Command executed successfully"

    # Mocking subprocess.run to return the dummy result
    with patch('tools.utils.subprocess.run', autospec=True) as mock_run:
        # Mock logger.info
        with patch('tools.utils.logger.info', autospec=True) as mock_logger_info:
            set_ipv6("bat0", "2001:db8::1")

        # Verifying subprocess.run was called with the expected command
        mock_run.assert_called_once_with(["ip", "-6", "addr", "add", "2001:db8::1/64", "dev", "bat0"], capture_output=True, text=True, check=True)

def test_is_interface_pingable_ipv4_success():
    with patch('tools.utils.subprocess.check_output', return_value="1 packets transmitted, 1 received", autospec=True) as mock_check_output:
        assert is_interface_pingable("bat0", "192.168.1.1")
        mock_check_output.assert_called_once_with(['ping', '-c', '1', '-w', '1', "192.168.1.1"], stderr=subprocess.STDOUT, universal_newlines=True)

def test_is_interface_pingable_ipv6_success():
    with patch('tools.utils.subprocess.check_output', return_value="1 packets transmitted, 1 received", autospec=True) as mock_check_output:
        assert is_interface_pingable("bat0", "fe80::230:1aff:fe4f:cf3c")
        mock_check_output.assert_called_once_with(['ping', '-c', '1', '-w', '1', "fe80::230:1aff:fe4f:cf3c%bat0"], stderr=subprocess.STDOUT, universal_newlines=True)

def test_is_interface_pingable_failure():
    with patch('tools.utils.subprocess.check_output', return_value="", autospec=True) as mock_check_output:
        assert not is_interface_pingable("bat0", "fe80::230:1aff:fe4f:cf3c")

def test_is_interface_pingable_invalid_ip():
    with pytest.raises(ValueError, match="Invalid IP address"):
        is_interface_pingable("eth0", "invalid_ip")
        
def test_wait_for_interface_pingable_success_after_tries():
    mock_pingable_side_effect = [False, False, True]  # 2 failures followed by a success
    with patch('tools.utils.is_interface_pingable', side_effect=mock_pingable_side_effect, autospec=True) as mock_is_pingable, \
         patch('tools.utils.logger.info', autospec=True) as mock_logger_info, \
         patch('tools.utils.time.sleep', autospec=True) as mock_sleep:

        wait_for_interface_to_be_pingable("bat0", "fe80::230:1aff:fe4f:cf3c")

        # Verify is_interface_pingable was called 3 times
        assert mock_is_pingable.call_count == 3
        # Verify sleep was called twice
        mock_sleep.assert_has_calls([call(1), call(1)])

def test_interface_up():
    with patch('subprocess.check_output', return_value=b'bat0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1460\ninet6 fe80::230:1aff:fe4f:cf3c  prefixlen 64  scopeid 0x20<link>\nether 00:30:1a:4f:cf:3c  txqueuelen 1000  (Ethernet)', autospec=True) as mock_subprocess:
        assert is_interface_up('bat0')
        mock_subprocess.assert_called_once_with(['ifconfig', 'bat0'], stderr=subprocess.DEVNULL)

def test_interface_down():
    with patch('subprocess.check_output', return_value=b'bat0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1460\nether 00:30:1a:4f:cf:3c  txqueuelen 1000  (Ethernet)', autospec=True) as mock_subprocess:
        assert not is_interface_up('bat0')
        mock_subprocess.assert_called_once_with(['ifconfig', 'bat0'], stderr=subprocess.DEVNULL)

def test_interface_not_present():
    with patch('subprocess.check_output', side_effect=subprocess.CalledProcessError(1, 'ifconfig bat0'), autospec=True) as mock_subprocess:
        assert not is_interface_up('bat0')
        mock_subprocess.assert_called_once_with(['ifconfig', 'bat0'], stderr=subprocess.DEVNULL)

def test_xor_bytes_without_padding():
    result = xor_bytes(b'\x01\x02\x03', b'\x11\x01\x01', 3)
    assert result == b'\x10\x03\x02'

def test_xor_bytes_with_padding():
    result = xor_bytes(b'\x01', b'\x02', 3)
    assert result == b'\x00\x00\x03'

    result = xor_bytes(b'\x01\x02\x03', b'\x01\x01\x01', 3)
    assert result == b'\x00\x03\x02'

    result = xor_bytes(b'\x01\x02', b'\x01\x01\x01', 3)
    assert result == b'\x01\x00\x03'

def test_xor_bytes_with_oversized_input():
    # Test with bytes that are longer than byte_size
    result = xor_bytes(b'\x01\x02\x03\x04', b'\x01\x01\x01\x01', 3)
    assert result == b'\x00\x03\x02'

    result = xor_bytes(b'\x01\x02\x03\x04', b'\x01\x01\x01', 3)
    assert result == b'\x00\x03\x02'

    result = xor_bytes(b'\x01\x02\x03', b'\x01\x01\x01\x01', 3)
    assert result == b'\x00\x03\x02'

def test_add_interface_to_batman_successful():
    with patch("subprocess.run") as mock_run, \
         patch('tools.utils.logger.info', autospec=True) as mock_logger_info:
        add_interface_to_batman("wlan0", "bat0")
        mock_run.assert_called_once_with(["batctl", "meshif", "bat0", "if", "add", "wlan0"], check=True)

def test_add_interface_to_batman_failed():
    with patch("subprocess.run", side_effect=Exception("Dummy error")) as mock_run, \
         patch("tools.utils.logger.error") as mock_logger_error:

        add_interface_to_batman("wlan0", "bat0")
        mock_run.assert_called_once_with(["batctl", "meshif", "bat0", "if", "add", "wlan0"], check=True)
        mock_logger_error.assert_called_once_with("Error adding interface wlan0 to bat0: Dummy error")

def test_add_interface_to_bridge_successful():
    with patch("subprocess.run") as mock_run, \
         patch('tools.utils.logger.info', autospec=True) as mock_logger_info:
        add_interface_to_bridge("eth0", "br0")
        mock_run.assert_called_once_with(["brctl", "addif", "br0", "eth0"], check=True)

def test_add_interface_to_bridge_failed():
    with patch("subprocess.run", side_effect=Exception("Dummy error")) as mock_run, \
         patch("tools.utils.logger.error") as mock_logger_error:

        add_interface_to_bridge("eth0", "br0")
        mock_run.assert_called_once_with(["brctl", "addif", "br0", "eth0"], check=True)
        mock_logger_error.assert_called_once_with("Error adding interface eth0 to br0: Dummy error")