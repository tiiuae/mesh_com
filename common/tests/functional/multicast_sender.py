import socket

# Define multicast address and port
MULTICAST_ADDRESS = "ff12::1"
PORT = 12345

# Message to send
MESSAGE = "Hello, multicast world!"

# Create a UDP socket
sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)

# Set time-to-live for multicast packet (optional)
sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_MULTICAST_HOPS, 1)

# Send message
sock.sendto(MESSAGE.encode('utf-8'), (MULTICAST_ADDRESS, PORT))

print("Message sent.")

