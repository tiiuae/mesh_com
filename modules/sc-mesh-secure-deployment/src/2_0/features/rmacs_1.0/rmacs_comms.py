import socket
import struct
import threading
import logging
import json
import time
from netstring import encode, decode
import msgpack


from config import Config
from logging_config import logger
from rmacs_setup import get_mesh_freq, get_ipv6_addr
# Multicast address and port configuration

def rmacs_comms(interface):
    """
    Create a RMACS Multicast socket for Server and Client communication
    """
    #prepare_multicast(interface)
    #args = Config
    if interface == 'br-lan':
        MULTICAST_GROUP = 'ff02::1'
        MULTICAST_PORT = 12345
        logger.info(f"MG :{MULTICAST_GROUP}, MP : {MULTICAST_PORT}, interface : {interface}")
    elif interface == 'osf0':
        MULTICAST_GROUP = 'ff13::39'
        MULTICAST_PORT = 12345    
        logger.info(f"MG :{MULTICAST_GROUP}, MP : {MULTICAST_PORT}")
    try:
        # Create a socket for IPv6 UDP communication
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
    #prepare_multicast(interface)
    #args = Config
    if interface == 'br-lan':
        MULTICAST_GROUP = 'ff02::1'
        MULTICAST_PORT = 12345
        logger.info(f"MG :{MULTICAST_GROUP}, MP : {MULTICAST_PORT}")
    elif interface == 'osf0':
        MULTICAST_GROUP = 'ff13::39'
        MULTICAST_PORT = 12345    
        logger.info(f"MG :{MULTICAST_GROUP}, MP : {MULTICAST_PORT}")
    try:
        serialized_data = msgpack.packb(data)
        netstring_data = encode(serialized_data)
        #logger.info(f"MG :{MULTICAST_GROUP}, MP : {MULTICAST_PORT} socket : {socket}, interface = {interface}")
        socket.sendto(netstring_data, (MULTICAST_GROUP, MULTICAST_PORT))  
        logger.info(f"Send report to Mutlicast")
        return None
    except BrokenPipeError:
        logging.info(f"Broken pipe error")
    except Exception as e:
        logging.info(f"Error sending data to Multicast")
        logger.info(f"Error sending data : {e}")

def prepare_multicast(interface) -> None:
    
    if interface == 'br-lan':
        MULTICAST_GROUP = 'ff02::1'
        MULTICAST_PORT = 12345
        logger.info(f"MG :{MULTICAST_GROUP}, MP : {MULTICAST_PORT}, interface : {interface}")
    elif interface == 'osf0':
        MULTICAST_GROUP = 'ff13::39'
        MULTICAST_PORT = 12345    
        logger.info(f"MG :{MULTICAST_GROUP}, MP : {MULTICAST_PORT}, interface: {interface} ")
    
        