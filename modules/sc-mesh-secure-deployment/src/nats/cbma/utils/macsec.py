import os
import time
import random

from typing import Tuple
from struct import Struct

from secure_socket.secure_connection import SecureConnection

from . import logging


MACSEC_CONFIG_VERSION = 1

BYTES_LENGTH = 128

MAX_RETRIES = 5
MIN_WAIT_TIME = 0.1  # seconds
MAX_WAIT_TIME = 1    # seconds

macsec_struct_format = '!{}sH'.format(BYTES_LENGTH * 2)
macsec_struct_v1 = Struct(macsec_struct_format)
# TODO - Add future structs here: macsec_struct_v2 = Struct(macsec_struct_format + 's16')

logger = logging.get_logger()


def generate_random_bytes() -> bytes:
    logger.debug('Generating random bytes')
    return os.urandom(BYTES_LENGTH)


def xor_bytes(local_key: bytes, peer_key: bytes) -> bytes:
    if len(local_key) != BYTES_LENGTH or len(peer_key) != BYTES_LENGTH:
        logger.debug(f"Local key length {len(local_key)}")
        logger.debug(f"Peer key length {len(peer_key)}")
        logger.debug(f"Expected key length {BYTES_LENGTH}")
        raise ValueError('Key lengths do not match the expected length')
    result = bytes(a ^ b for a, b in zip(local_key, peer_key))
    return result


def send_macsec_config(conn: SecureConnection, my_config: bytes) -> int:
    peer_ipv6 = conn.get_peer_name()[0]
    logger.debug(f"Sending MACsec configuration to {peer_ipv6}")
    for _ in range(MAX_RETRIES):
        try:
            return conn.sendall(my_config)
        except Exception:
            time.sleep(random.uniform(MIN_WAIT_TIME, MAX_WAIT_TIME))
    raise ConnectionRefusedError('Connection refused')


def recv_macsec_config(conn: SecureConnection, config_length: int) -> bytes:
    peer_ipv6 = conn.get_peer_name()[0]
    logger.debug(f"Receiving MACsec configuration from {peer_ipv6}")
    for _ in range(MAX_RETRIES):
        try:
            return conn.recv(config_length)
        except Exception:
            time.sleep(random.uniform(MIN_WAIT_TIME, MAX_WAIT_TIME))
    raise ConnectionRefusedError('Connection refused')


def exchange_macsec_config(conn: SecureConnection, my_config: bytes) -> bytes:
    my_ipv6 = conn.get_sock_name()[0]
    peer_ipv6 = conn.get_peer_name()[0]

    logger.debug(f"Exchanging MACsec configuration with {peer_ipv6}")

    config_length = len(my_config)
    if my_ipv6 > peer_ipv6:
        peer_config = recv_macsec_config(conn, config_length)
        send_macsec_config(conn, my_config)
    else:
        send_macsec_config(conn, my_config)
        peer_config = recv_macsec_config(conn, config_length)

    peer_config_length = len(peer_config)
    if peer_config_length != config_length:
        raise ValueError(f"Peer {peer_ipv6} MACsec config length ({peer_config_length}) doesn't match the expected one: {config_length}")

    logger.debug(f"MACsec configuration successfully exchanged with {peer_ipv6}")

    return peer_config


def get_macsec_config(conn: SecureConnection) -> Tuple[bytes, bytes]:
    peer_ipv6 = conn.get_peer_name()[0]

    logger.debug(f"Generating MACsec configuration for {peer_ipv6}")

    my_tx_key_bytes = generate_random_bytes()
    my_rx_key_bytes = generate_random_bytes()
    my_version = MACSEC_CONFIG_VERSION

    my_packed_config = macsec_struct_v1.pack(my_tx_key_bytes + my_rx_key_bytes, my_version)
    peer_packed_config = exchange_macsec_config(conn, my_packed_config)

    peer_packed_config_size = len(peer_packed_config)
    if peer_packed_config_size < macsec_struct_v1.size:
        raise ValueError(f"Size of received data from {peer_ipv6} is lower than minimum expected: {peer_packed_config_size} < {macsec_struct_v1.size}")

    peer_config = macsec_struct_v1.unpack(peer_packed_config[:macsec_struct_v1.size])
    peer_rx_key_bytes = peer_config[0][:BYTES_LENGTH]
    peer_tx_key_bytes = peer_config[0][BYTES_LENGTH:]
    peer_version = peer_config[1]

    config_version = min(my_version, max(peer_version, 1))
    if my_version != peer_version:
        # NOTE - Prior to version v1 the version field was a never-used MACsec random port
        # TODO - This is temporarily here to be used in future newer config versions
        logger.debug(f"Ignore -> Version mismatch in MACsec configurations => our: {my_version} - theirs: {peer_version} - using: {config_version}")

    # TODO - Add here extraction of future configs fields, for example:
    # if config_version == 2:
    #     peer_config = macsec_struct_v2.unpack(peer_packed_config[:macsec_struct_v2.size])
    #     peer_cipher = peer_config[2]

    tx_key = xor_bytes(my_tx_key_bytes, peer_tx_key_bytes)
    rx_key = xor_bytes(my_rx_key_bytes, peer_rx_key_bytes)

    logger.debug(f"MACsec configuration for {peer_ipv6} generated successfully")

    return tx_key, rx_key
