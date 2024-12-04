import socket
import struct
import threading
import logging
import json
import time
from netstring import encode, decode
import msgpack


#from config_old import Config, MULTICAST_CONFIG
from config import create_default_config, load_config 
from logging_config import logger
from rmacs_setup import get_mesh_freq, get_ipv6_addr

config_file_path = '/etc/meshshield/rmacs_config.yaml'

def get_multicast_config(interface):
    """
    Retrieve multicast group and port based on the interface.
    """
    config = load_config(config_file_path)
    multicast_config = config.get("MULTICAST_CONFIG",{})
    _config = multicast_config.get(interface)
    if config:
        return _config['group'], _config['port']
    else:
        raise ValueError(f"Unknown interface: {interface}")

def rmacs_comms(interface):
    """
    Create a RMACS Multicast socket for Server and Client communication
    """
    try:
        # Create a socket for IPv6 UDP communication
        # Retrieve the multicast group and port based on the interface
        MULTICAST_GROUP, MULTICAST_PORT = get_multicast_config(interface)
        sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)

        # Allow multiple sockets to use the same port
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)    
        sock.bind(('', MULTICAST_PORT))  # '' means bind to all interfaces

        # Join the multicast group
        # Set the outgoing interface for multicast
        interface_index = socket.if_nametoindex(interface)
        sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_MULTICAST_IF, interface_index)

        # Join the multicast group
        rmacs = struct.pack("16sI", socket.inet_pton(socket.AF_INET6, MULTICAST_GROUP), interface_index)
        sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_JOIN_GROUP, rmacs)
        logger.info(f"Server listening on {MULTICAST_GROUP}:{MULTICAST_PORT}")
        return sock
    except socket.error as sock_err:
            logging.error(f"Socket error occurred: {sock_err}")
            return None
    except ValueError as val_err:
        logging.error(f"Value error occurred: {val_err}")
        return None
    except Exception as ex:
        logging.error(f"An unexpected error occurred: {ex}")
        return None
    
def send_data(socket, data, interface) -> None:

    try:
        MULTICAST_GROUP, MULTICAST_PORT = get_multicast_config(interface)
        serialized_data = msgpack.packb(data)
        netstring_data = encode(serialized_data)
        socket.sendto(netstring_data, (MULTICAST_GROUP, MULTICAST_PORT))  
        logger.info(f"Sent report to Mutlicast")
        return None
    except BrokenPipeError:
        logging.info(f"Broken pipe error")
    except Exception as e:
        logging.info(f"Error sending data to Multicast")
        logger.info(f"Error sending data : {e}")

        