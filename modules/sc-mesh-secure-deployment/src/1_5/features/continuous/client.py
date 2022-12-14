import hashlib
import json
import math
import random
import socket
import sys
import time
import pandas as pd

from .functions import client_functions

import sys

sys.path.insert(0, '../../')

from features.mutual.mutual import *

def initiate_client(server_ip, ID, return_dict):
    c = socket.socket()  # create server socket c with default param ipv4, TCP
    partial_result = []

    # connect to server socket
    try:
        c.connect((server_ip, 9999))

        # name = input("Enter your name:")  # Taking input from client
        c.send(bytes(str(ID), 'utf-8'))  # Send client name to server

        # receive byte sent by server and print it
        message = c.recv(1024).decode()  # decode byte to string
        print(message)

        mut = Mutual('wlan1')

        if message == 'Connected to server, need to exchange public keys':
            received_cert = c.recv(1024)
            server_cert = received_cert[:-5]
            servID = received_cert[-5:].decode('utf-8')
            # Save server public key certificate to {cliID}.der
            with open(f'{servID}.der', 'wb') as writer:
                writer.write(server_cert)

            print('Sending my public key')
            cert = open(mut.local_cert, 'rb').read()
            message = cert + mut.myID.encode('utf-8')
            c.send(message)

        # Initialization
        filetable = pd.read_csv('auth/dev.csv')
        servID = filetable[filetable['IP'] == server_ip]['ID'].iloc[0] # Get the server ID
        secret_filename = f'secrets/secret_{servID}.der'
        #secret_byte = open(secret_filename, 'rb').read()

        # Derive secret key and store it to secret_{servID}.der
        print('Deriving secret')
        pri.derive_ecdh_secret(servID, servID, mut.local_cert, mut.salt)
        secret_byte = pri.decrypt_file(secret_filename, mut.local_cert, mut.salt)
        secret = int.from_bytes(secret_byte, byteorder=sys.byteorder)
        #secret = 1234
        server_id = server_ip
        client_id = socket.gethostbyname(socket.gethostname())
        crc_key = '1001'  # CRC key
        start_time = time.time()
        timestamp = start_time  # Gives current timestamp in seconds
        time_flag = 1  # Initialization of time flag
        max_count = 3
        flag_ctr = 0

        period = 2  # Period for each authentication in seconds
        total_period = 20  # Total period for the session
        sent_shares = []  # Store shares sent during the session to prevent duplicate shares
        msg_size = 0  # Initialized to compute avg message size

        # Generate k degree polynomial for the secret
        #auth_result = "pass"  # initialization
        auth_result = 1  # initialization
        while flag_ctr <= max_count:
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
                print("IP: ", server_ip)
                print("Result = ", result)
                print('*********************************************************************')
                print(' ')
                try:
                    result = json.loads(result)
                    partial_result.append(result["auth_result"])
                except json.decoder.JSONDecodeError:
                    res = result.split('Closing connection')[0]
                    result = json.loads(res)
                    partial_result.append(result["auth_result"])

                # Do not send data for backoff period if auth fails
                #if result["auth_result"] == "fail":
                # # function for back off
                # if result["auth_result"] == 0:
                #     backoff_start = time.time()
                #     while time.time() - backoff_start <= result["backoff_period"]:
                #         pass
            elif request == 'Closing connection':
                print('Connection closed')
                break
            flag_ctr += 1
        return_dict[auth_result] = partial_result
        print("Share storage cost: ", sys.getsizeof(sent_shares))
        return result["auth_result"]
    except (ConnectionRefusedError, OSError):
        return_dict = 3
        return 3

