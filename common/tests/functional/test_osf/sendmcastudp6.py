#!/usr/bin/env python3

import time
import socket
import sys
import struct

MYHOP = 1 # Increase to reach other networks

def main(argv):
    multicast_group = argv[1]
    multicast_port = int(argv[2])
    interface = argv[3]
    data = data = "Hello IPV6 world !"
    if len(sys.argv) >= 5:
    	data = argv[4]
    
    # Create ipv6 datagram socket
    sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
    
    # Set hops
    hops_bin = struct.pack('@i', MYHOP)
    sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_MULTICAST_HOPS, hops_bin)
    
    # Bind to interface
    sock.setsockopt(socket.SOL_SOCKET, 25, str(interface + '\0').encode('utf-8'))
    
    # Send message and exit
    try:
    #while True:
     sock.sendto(data.encode('utf-8'), (multicast_group, multicast_port))
     time.sleep(1)   
    except KeyboardInterrupt:
        print("End")
    finally:
        sock.close()

if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("Usage: {0} <group address> <port> <interface> [message]".format(sys.argv[0]))
        sys.exit(1)
    main(sys.argv)

