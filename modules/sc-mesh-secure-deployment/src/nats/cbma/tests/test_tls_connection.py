import os
import sys
import threading

from pathlib import Path
from typing import Callable
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).parents[1]))

from secure_socket.verification import TLSVerification
from utils.networking import get_interface_link_local_ipv6_address
from secure_socket.client import FileBasedSecureSocketClient
from secure_socket.server import FileBasedSecureSocketServer
from models.certificates import CBMACertificates


CURRENT_FILE_PATH = Path(__file__).resolve()
TEST_DATA_FOLDER_PATH = CURRENT_FILE_PATH.parent.parent / "unittests/test_data/certificates"


def thread_start(func: Callable) -> threading.Thread:
    thread = threading.Thread(target=func, args=(), daemon=True)
    thread.start()

    return thread


def test_tls_connection_fails_bogus_cert() -> None:
    certfile = os.path.join(TEST_DATA_FOLDER_PATH, "bogus.crt")
    keyfile = os.path.join(TEST_DATA_FOLDER_PATH, "bogus.key")
    chain_path = os.path.join(TEST_DATA_FOLDER_PATH, "chain.crt")

    certificates = CBMACertificates(certfile, keyfile, [chain_path])

    custom_cb_mock = Mock()

    interface = "wlp0s20f3"
    address = get_interface_link_local_ipv6_address(interface)

    server = FileBasedSecureSocketServer(interface, 4433, certificates, custom_cb_mock)
    server_thread = thread_start(server.listen)

    client = FileBasedSecureSocketClient(interface, address, 4433, certificates, custom_cb_mock)

    if client.connect():
        assert False

    server_thread.join()

    assert custom_cb_mock.call_count == 0


def test_tls_connection_fails_bogus_chain() -> None:
    certfile = os.path.join(TEST_DATA_FOLDER_PATH, "csl1.local.crt")
    keyfile = os.path.join(TEST_DATA_FOLDER_PATH, "csl1.local.key")
    chain_path = os.path.join(TEST_DATA_FOLDER_PATH, "bogus.chain.crt")

    certificates = CBMACertificates(certfile, keyfile, [chain_path])

    custom_cb_mock = Mock()

    interface = "wlp0s20f3"
    address = get_interface_link_local_ipv6_address(interface)

    server = FileBasedSecureSocketServer(interface, 4433, certificates, custom_cb_mock)
    server_thread = thread_start(server.listen)

    client = FileBasedSecureSocketClient(interface, address, 4433, certificates, custom_cb_mock)

    if client.connect():
        assert False

    server_thread.join()

    assert custom_cb_mock.call_count == 0


def test_tls_connection_fails_verification() -> None:
    certfile = os.path.join(TEST_DATA_FOLDER_PATH, "csl1.local.crt")
    keyfile = os.path.join(TEST_DATA_FOLDER_PATH, "csl1.local.key")
    chain_path = os.path.join(TEST_DATA_FOLDER_PATH, "chain.crt")

    certificates = CBMACertificates(certfile, keyfile, [chain_path])

    verify_cb_mock = Mock()
    verify_cb_mock.return_value = False

    TLSVerification.verify = verify_cb_mock

    custom_cb_mock = Mock()

    interface = "wlp0s20f3"
    address = get_interface_link_local_ipv6_address(interface)

    server = FileBasedSecureSocketServer(interface, 4433, certificates, custom_cb_mock)
    server_thread = thread_start(server.listen)

    client = FileBasedSecureSocketClient(interface, address, 4433, certificates, custom_cb_mock)

    if client.connect():
        assert False

    server_thread.join()

    # verify_cb is called once for server, for client it is not called at all
    assert verify_cb_mock.call_count == 1


def test_tls_connection_succeeds_verification() -> None:
    certfile = os.path.join(TEST_DATA_FOLDER_PATH, "csl1.local.crt")
    keyfile = os.path.join(TEST_DATA_FOLDER_PATH, "csl1.local.key")
    chain_path = os.path.join(TEST_DATA_FOLDER_PATH, "chain.crt")

    certificates = CBMACertificates(certfile, keyfile, [chain_path])

    verify_cb_mock = Mock()
    verify_cb_mock.return_value = True

    TLSVerification.verify = verify_cb_mock

    custom_cb_mock = Mock()

    interface = "wlp0s20f3"
    address = get_interface_link_local_ipv6_address(interface)

    server = FileBasedSecureSocketServer(interface, 4433, certificates, custom_cb_mock)
    server_thread = thread_start(server.listen)

    client = FileBasedSecureSocketClient(interface, address, 4433, certificates, custom_cb_mock)

    if not client.connect():
        assert False

    server_thread.join()

    # verify_cb is called at least once for each certificate in chain
    # this is 3 times Server + 3 times client = 6
    assert verify_cb_mock.call_count == 6


def test_tls_connection_fails_custom_callback() -> None:
    certfile = os.path.join(TEST_DATA_FOLDER_PATH, "csl1.local.crt")
    keyfile = os.path.join(TEST_DATA_FOLDER_PATH, "csl1.local.key")
    chain_path = os.path.join(TEST_DATA_FOLDER_PATH, "chain.crt")

    certificates = CBMACertificates(certfile, keyfile, [chain_path])

    custom_cb_mock = Mock()
    custom_cb_mock.return_value = False

    interface = "wlp0s20f3"
    address = get_interface_link_local_ipv6_address(interface)

    server = FileBasedSecureSocketServer(interface, 4433, certificates, custom_cb_mock)
    server_thread = thread_start(server.listen)

    client = FileBasedSecureSocketClient(interface, address, 4433, certificates, custom_cb_mock)

    if client.connect():
        assert False

    server_thread.join()

    assert custom_cb_mock.call_count == 2


def test_tls_connection_succeeds_custom_verify_callback() -> None:
    certfile = os.path.join(TEST_DATA_FOLDER_PATH, "csl1.local.crt")
    keyfile = os.path.join(TEST_DATA_FOLDER_PATH, "csl1.local.key")
    chain_path = os.path.join(TEST_DATA_FOLDER_PATH, "chain.crt")

    certificates = CBMACertificates(certfile, keyfile, [chain_path])

    custom_cb_mock = Mock()
    custom_cb_mock.return_value = True

    interface = "wlp0s20f3"
    address = get_interface_link_local_ipv6_address(interface)

    server = FileBasedSecureSocketServer(interface, 4433, certificates, custom_cb_mock)
    server_thread = thread_start(server.listen)

    client = FileBasedSecureSocketClient(interface, address, 4433, certificates, custom_cb_mock)

    if not client.connect():
        assert False

    server_thread.join()

    assert custom_cb_mock.call_count == 2
