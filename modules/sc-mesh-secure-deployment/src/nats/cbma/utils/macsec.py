import os
import time
import struct
import random

from typing import Tuple

from secure_socket.secure_connection import SecureConnection

from utils.networking import get_mac_from_ipv6
from . import logging


BYTES_LENGTH = 128
MAX_MACSEC_PORT = 2 ** 16

MAX_RETRIES = 5
MIN_WAIT_TIME = 0.1  # seconds
MAX_WAIT_TIME = 1    # seconds

_macsec_ports: list[int] = []

logger = logging.get_logger()


def generate_random_bytes() -> bytes:
    logger.debug("Generating random bytes")
    return os.urandom(BYTES_LENGTH)


def xor_bytes(local_key: bytes, peer_key: bytes) -> bytes:
    if len(local_key) != BYTES_LENGTH or len(peer_key) != BYTES_LENGTH:
        logger.debug(f"Local key length {len(local_key)}")
        logger.debug(f"Peer key length {len(peer_key)}")
        logger.debug(f"Expected key length {BYTES_LENGTH}")
        raise ValueError("Key lengths do not match the expected length")
    result = bytes(a ^ b for a, b in zip(local_key, peer_key))
    return result


def get_macsec_port() -> int:
    # TODO - Check ports in used with ip macsec
    port = random.randint(1, MAX_MACSEC_PORT)

    _macsec_ports.append(port)
    return port


def free_macsec_port(port: int) -> bool:
    try:
        _macsec_ports.remove(port)
        return True
    except ValueError:
        logger.error(f"MACsec port {port} wasn't reserved")
    return False


def send_macsec_config(conn: SecureConnection, my_config: bytes) -> int:
    for _ in range(MAX_RETRIES):
        try:
            return conn.sendall(my_config)
        except Exception:
            time.sleep(random.uniform(MIN_WAIT_TIME, MAX_WAIT_TIME))
    raise ConnectionRefusedError("Connection refused")


def recv_macsec_config(conn: SecureConnection, config_length: int) -> bytes:
    for _ in range(MAX_RETRIES):
        try:
            return conn.recv(config_length)
        except Exception:
            time.sleep(random.uniform(MIN_WAIT_TIME, MAX_WAIT_TIME))
    raise ConnectionRefusedError("Connection refused")


def exchange_macsec_config(conn: SecureConnection, my_config: bytes) -> bytes:
    peer_ipv6 = conn.get_peer_name()[0]
    peer_mac = get_mac_from_ipv6(peer_ipv6)

    logger.debug(f"Exchanging MACsec configuration with {peer_mac} peer")

    my_ipv6 = conn.get_sock_name()[0]
    my_mac = get_mac_from_ipv6(my_ipv6)

    config_length = len(my_config)
    if my_mac.lower() > peer_mac.lower():
        peer_config = recv_macsec_config(conn, config_length)
        send_macsec_config(conn, my_config)
    else:
        send_macsec_config(conn, my_config)
        peer_config = recv_macsec_config(conn, config_length)

    peer_config_length = len(peer_config)
    if peer_config_length != config_length:
        raise ValueError(f"Peer {peer_mac} MACsec config length ({peer_config_length}) doesn't match the expected one: {config_length}")

    logger.debug(f"MACsec configuration successfully exchanged with {peer_mac} peer")

    return peer_config


def get_macsec_config(conn: SecureConnection) -> Tuple[Tuple[bytes, bytes], Tuple[int, int]]:
    peer_ipv6 = conn.get_peer_name()[0]
    peer_mac = get_mac_from_ipv6(peer_ipv6)

    logger.debug(f"Generating MACsec configuration for {peer_mac} peer")

    my_tx_key_bytes = generate_random_bytes()
    my_rx_key_bytes = generate_random_bytes()
    rx_port = get_macsec_port()

    struct_format = '!{}sH'.format(BYTES_LENGTH * 2)

    my_packed_config = struct.pack(struct_format, my_tx_key_bytes + my_rx_key_bytes, rx_port)
    peer_packed_config = exchange_macsec_config(conn, my_packed_config)

    peer_config = struct.unpack(struct_format, peer_packed_config)
    peer_rx_key_bytes = peer_config[0][:BYTES_LENGTH]
    peer_tx_key_bytes = peer_config[0][BYTES_LENGTH:]
    tx_port = peer_config[1]

    tx_key = xor_bytes(my_tx_key_bytes, peer_tx_key_bytes)
    rx_key = xor_bytes(my_rx_key_bytes, peer_rx_key_bytes)

    keys = (tx_key, rx_key)
    ports = (rx_port, tx_port)

    logger.debug(f"MACsec configuration for {peer_mac} peer generated successfully")

    return (keys, ports)
