import socket

# Define multicast address and port
MULTICAST_ADDRESS = "ff12::1"
PORT = 12345

# Create a UDP socket
sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)

# Bind to the port
sock.bind(('', PORT))

# Tell the operating system to add the socket to the multicast group
group_bin = socket.inet_pton(socket.AF_INET6, MULTICAST_ADDRESS)
mreq = group_bin + b'\0' * (len(group_bin) - len(group_bin))
sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_JOIN_GROUP, mreq)

print("Listening for multicast messages...")

# Receive/respond loop
while True:
    data, address = sock.recvfrom(1024)
    print(f"Received message from {address}: {data.decode('utf-8')}")

