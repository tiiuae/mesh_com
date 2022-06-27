import hashlib
import json
import math
import random
import socket
import sys
import time

from .functions import client_functions


def initiate_client(server_ip, ID):
    c = socket.socket()  # create server socket c with default param ipv4, TCP

    # connect to server socket
    c.connect((server_ip, 9999))

    # name = input("Enter your name:")  # Taking input from client
    c.send(bytes(ID, 'utf-8'))  # Send client name to server

    # receive byte sent by server and print it
    print(c.recv(1024).decode())  # decode byte to string

    # Initialization
    secret = 1234
    server_id = server_ip
    client_id = socket.gethostbyname(socket.gethostname())
    crc_key = '1001'  # CRC key
    start_time = time.time()
    timestamp = start_time  # Gives current timestamp in seconds
    time_flag = 1  # Initialization of time flag

    period = 2  # Period for each authentication in seconds
    total_period = 20  # Total period for the session
    sent_shares = []  # Store shares sent during the session to prevent duplicate shares
    msg_size = 0  # Initialized to compute avg message size

    # Generate k degree polynomial for the secret
    auth_result = "pass"  # initialization
    while True:
        request = c.recv(1024).decode()
        if request == 'Request to authenticate':
            print("Processing authentication request")
            # Generate share and share authenticator
            while True:  # Select random x until unique share is generated
                x = random.randint(1, 9999)  # Generate random x
                u = secret + time_flag + x  # Share = secret + time flag + random number x
                if sent_shares.count(u) == 0:  # Check that new share has not been sent previously
                    sent_shares.append(u)
                    break

            # Share authenticator = hash(share - secret - timeflag) = hash(x)
            sa = hashlib.sha256(bytes(str(x), 'utf-8')).digest()

            print("share u = ", u)
            print("Share Authenticator sa = ", sa)

            msg = f"Continuous Authentication {str(time_flag)}"

            # Generate message with authentication tokens
            msg_to_send = client_functions.message_generator(secret, server_id, client_id, msg, u, time_flag, sa)
            # Add CRC
            msg_with_crc = client_functions.crc_generator(msg_to_send, crc_key)

            # Convert from binary to bytes
            msg_with_crc_bytes = int(msg_with_crc, 2).to_bytes(math.ceil(len(msg_with_crc) / 8), byteorder='big')
            # print("Message sent in bytes = ", msg_with_crc_bytes)
            print("Size of message in bytes = ", sys.getsizeof(msg_with_crc_bytes))
            # msg_size = msg_size + sys.getsizeof(msg_with_crc_bytes)

            c.send(msg_with_crc_bytes)  # Send message to server

            time_flag = time_flag + 1

            while True:
                # Resend message to server if CRC fails
                result = c.recv(1024).decode()
                if result != 'Resend message':
                    break

                print("Resending message")
                c.send(msg_with_crc_bytes)  # Resend message to server
            print("Result = ", result)
            print('*********************************************************************')
            print(' ')
            result = json.loads(result)

            # Do not send data for backoff period if auth fails
            if result["auth_result"] == "fail":
                backoff_start = time.time()
                while time.time() - backoff_start <= result["backoff_period"]:
                    pass
        elif request == 'Closing connection':
            print('Connection closed')
            break
    print("Share storage cost: ", sys.getsizeof(sent_shares))
    return result["auth_result"]
    # print("Average message size: ", msg_size/ math.ceil(total_period/period))
