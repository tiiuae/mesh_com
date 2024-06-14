#!/usr/bin/env python3

import socket
import sys
import struct

def main(argv):
    multicast_group = argv[1]
    multicast_port = int(argv[2])
    interface = argv[3]
   
    # Create socket
    sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
   
    # Use specified multicast interface and port
    ipv6mr_interface = struct.pack('i', socket.if_nametoindex(interface))
    sock.bind(('', multicast_port))
    
    # Compose joint message
    mreq = socket.inet_pton(socket.AF_INET6, multicast_group) + ipv6mr_interface
    sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_JOIN_GROUP, mreq)
    
    print(f"Listen '{multicast_group}' port '{multicast_port}' interface '{interface}' ...")	

    # Receive message(s)	    
    try:
       while True:
        	data, addr = sock.recvfrom(1500)
        	print(f"Received packet from {addr}, {len(data)} bytes, {data}")
        	sys.exit(0)      
    except KeyboardInterrupt:
        print("End")
    finally:
        sock.close()
     
if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("Usage: {0} <group address> <port> <interface>".format(sys.argv[0]))
        sys.exit(1)
    main(sys.argv)

