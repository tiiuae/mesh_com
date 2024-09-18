import socket
import struct
import threading
import logging
import json
import time

# Multicast address and port configuration
MULTICAST_GROUP = 'ff05::1'  # IPv6 multicast address for entire network
MULTICAST_PORT = 5765

def mhra_comms_server():
    """
    UDP server listening for multicast messages.
    Runs in a separate thread.
    """
    # Create a socket for IPv6 UDP communication
    sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

    # Allow multiple sockets to use the same port
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Bind the socket to the multicast port
    sock.bind(('', MULTICAST_PORT))  # '' means bind to all interfaces

    # Join the multicast group
    multicast_group_bin = socket.inet_pton(socket.AF_INET6, MULTICAST_GROUP)
    interface_index = socket.if_nametoindex('br-lan') 

    # Construct the multicast request
    mreq = multicast_group_bin + struct.pack('@I', interface_index)
    sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_JOIN_GROUP, mreq)

    logging.info(f"Server listening on {MULTICAST_GROUP}:{MULTICAST_PORT}")

    while True:
        # Receive data from the socket
        data, address = sock.recvfrom(1024)
        message = data.decode('utf-8')

        # Log the source IPv6 address
        logging.info(f"Received message from {address[0]}")
        logging.info(f"Message: {message}")
        
        # Parse the JSON message
        parsed_message = json.loads(message)
        logging.info(f"Parsed Message: {parsed_message}")


def start_mhra_comms_server():
    """
    Start the MHRA UDP server in a separate thread.
    """
    server_thread = threading.Thread(target=mhra_comms_server)
    server_thread.daemon = True  # Daemonize the thread to exit with the main program
    server_thread.start()
    logging.info("MHRA communication server thread started.")


def mhra_comms_client(message, ttl_value=1):
    """
    Client function to send a message to the multicast group.
    This runs on the main thread when called.
    
    Parameters:
    message (str): The message to send.
    ttl_value (int): The time-to-live value for the multicast message.
    """
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

    # Set the time-to-live for the message (adjustable)
    sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_MULTICAST_HOPS, ttl_value)

    # Get the interface index and set the multicast interface
    sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_MULTICAST_IF, socket.if_nametoindex("br-lan"))

    # Send the message to the multicast group
    sock.sendto(message.encode('utf-8'), (MULTICAST_GROUP, MULTICAST_PORT))

    logging.info(f"Sent message: {message} to {MULTICAST_GROUP}:{MULTICAST_PORT} with TTL={ttl_value}")

def create_json_message(msg_type, target="All", payload=None, status_code=0):
    """
    Create a JSON message based on input parameters without source address.
    
    Parameters:
    msg_type (str): The type of the message (e.g., "COMMAND", "STATUS", "DATA").
    target (str): The target device or entity (default is "All").
    payload (dict): The payload of the message (e.g., command details or data) (optional).

    Returns:
    str: A JSON-formatted message as a string.
    """
    # Create the base message structure
    message = {
        "msg_type": msg_type,
        "target": target  # Default target is "All"
    }

    # Add optional payload if provided
    if payload:
        message["payload"] = payload

    # Convert the message dictionary to a JSON string
    return json.dumps(message, indent=4)

def send_enable_on_demand_radio(target="All"):
    """
    Sends a command to enable the on-demand radio via IPv6 multicast.

    Parameters:
    target (str): The target device (default is "All").
    """
    # Command message to enable on-demand radio
    payload = {
        "command": "ENABLE_ON_DEMAND_RADIO"
    }

    # Create the JSON message
    message = create_json_message(msg_type="COMMAND", target=target, payload=payload)

    # Send message using mhra_comms_client (with TTL set to 10)
    mhra_comms_client(message, ttl_value=10)

