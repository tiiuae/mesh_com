import pytest
from unittest.mock import patch, call

import add_syspath
from macsec.macsec import Macsec


@pytest.fixture
def macsec_instance():
    return Macsec()

def test_set_macsec_tx(macsec_instance):
    client_mac = "aa:bb:cc:dd:ee:ff"
    my_macsec_key = "mykey"
    my_port = 12345

    with patch("subprocess.run") as mock_run:
        macsec_instance.set_macsec_tx(client_mac, my_macsec_key, my_port)

    expected_calls = [
        call(["ip", "link", "add", "link", macsec_instance.interface, "lmsaabbccddeeff", "type", "macsec", "port", str(my_port), "encrypt", macsec_instance.macsec_encryption, "cipher", "gcm-aes-256"], check=True),
        call(["ip", "macsec", "add", "lmsaabbccddeeff", "tx", "sa", "0", "pn", "1", "on", "key", "01", my_macsec_key], check=True),
        call(["ip", "link", "set", "lmsaabbccddeeff", "up"], check=True),
        call(["ip", "macsec", "show"], check=True)
    ]

    assert mock_run.call_args_list == expected_calls

def test_set_macsec_rx(macsec_instance):
    client_mac = "aa:bb:cc:dd:ee:ff"
    client_macsec_key = "clientkey"
    client_port = 12345

    with patch("subprocess.run") as mock_run:
        macsec_instance.set_macsec_rx(client_mac, client_macsec_key, client_port)

    expected_calls = [
        call(["ip", "macsec", "add", "lmsaabbccddeeff", "rx", "port", str(client_port), "address", client_mac], check=True),
        call(["ip", "macsec", "add", "lmsaabbccddeeff", "rx", "port", str(client_port), "address", client_mac, "sa", "0", "pn", "1", "on", "key", client_mac.replace(":", ""), client_macsec_key], check=True),
        call(["ip", "macsec", "show"], check=True)
    ]

    assert mock_run.call_args_list == expected_calls

def test_macsec_interface_name(macsec_instance):
    assert macsec_instance.get_macsec_interface_name("aa:bb:cc:dd:ee:ff") == "lmsaabbccddeeff"
    macsec_instance.level = "upper"
    assert macsec_instance.get_macsec_interface_name("aa:bb:cc:dd:ee:ff") == "umsaabbccddeeff"


def test_assign_unique_port(macsec_instance):
    client_mac = "aa:bb:cc:dd:ee:ff"
    assigned_port = macsec_instance.assign_unique_port(client_mac)

    assert assigned_port in range(1, 2 ** 16) # Assert assigned port is in range
    assert client_mac in macsec_instance.used_ports # Assert that record for client_mac has been added in used_ports
    assert assigned_port == macsec_instance.used_ports[client_mac] # Assert that assigned_port has been recorded correctly for client_mac in used ports
    assert assigned_port not in macsec_instance.available_ports # Assert that assigned port is not in available ports


def test_release_port(macsec_instance):
    macsec_instance = Macsec()
    client_mac = "aa:bb:cc:dd:ee:ff"
    assigned_port = macsec_instance.assign_unique_port(client_mac)
    macsec_instance.release_port(client_mac)

    assert client_mac not in macsec_instance.used_ports # assert that client has been removed from used ports
    assert assigned_port in macsec_instance.available_ports # assert that the assigned port has been released back as available port


def test_release_port_error(macsec_instance):
    macsec_instance = Macsec()
    with pytest.raises(ValueError, match=r"Client .* is not in the list of used ports."):
        macsec_instance.release_port("00:11:22:33:44:55")


# Remove log directory during teardown
import shutil
import os
def remove_logs_directory():
    logs_directory = 'logs'
    if os.path.exists(logs_directory) and os.path.isdir(logs_directory):
        shutil.rmtree(logs_directory)

def teardown_module():
    remove_logs_directory()
