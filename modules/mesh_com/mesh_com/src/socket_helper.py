"""
Simple socket helper to handle large data frames and
ensure complete data frames are received.
"""
import struct


def send_msg(sock, msg):
    """
    sock: socket
    msg: byte string

    send data packet with length prefix
    """
    # Prefix each message with a 4-byte length (network byte order)
    msg = struct.pack('>I', len(msg)) + msg
    # sendall is a high-level Python-only method that sends the entire buffer
    sock.sendall(msg)


def recv_msg(sock):
    """
    sock: socket

    receive data packet with length prefix
    :return: bytearray
    """
    # Read message length and unpack it into an integer
    # message lenght is 4 bytes as message prefix
    raw_msglen = recvall(sock, 4)
    if not raw_msglen:
        return None
    msglen = struct.unpack('>I', raw_msglen)[0]
    # Read the message data
    return recvall(sock, msglen)


def recvall(sock, n):
    """
    sock: socket
    n: integer, data amount

    receive n-amount of data from socket

    :return: bytearray
    """
    data = bytearray()
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data.extend(packet)
    return data
