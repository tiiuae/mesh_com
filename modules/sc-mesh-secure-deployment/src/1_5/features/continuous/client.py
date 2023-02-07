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
import os
import traceback

sys.path.insert(0, '../../')

from features.mutual.mutual import *
from features.mutual.utils import primitives as pri

def initiate_client(server_ip, ID, logger=None):
    c = socket.socket()  # create server socket c with default param ipv4, TCP
    partial_result = []
    local_cert = "/etc/ssl/certs/mesh_cert.der"

    # connect to server socket
    try:
        c.connect((server_ip, 9999))

        if logger:
            logger.info("Client connected to server (%s, 9999)", server_ip)

        # name = input("Enter your name:")  # Taking input from client
        c.send(bytes(str(ID), 'utf-8'))  # Send client name to server

        # receive byte sent by server and print it
        message = c.recv(1024).decode()  # decode byte to string
        print(message)

        #mut = Mutual('wlan1')

        client_mesh_name = server_ip.replace('.', '_')
        print("client_mesh_name = ", client_mesh_name)

        if message == 'Connected to server, need to exchange public keys and ID':
            received_cert = c.recv(1024)
            server_cert = received_cert[:-8]
            servID = received_cert[-8:].decode('utf-8')
            #server_cert = received_cert

            # Create directory pubKeys to store neighbor node's public key certificates to use for secret derivation
            if not os.path.exists('pubKeys/'):
                os.mkdir('pubKeys/')

            # Save server public key certificate to pubKeys/{client_mesh_name}.der
            with open(f'pubKeys/{client_mesh_name}.der', 'wb') as writer:
                writer.write(server_cert)

            print('Sending my public key and ID')
            cert = open(local_cert, 'rb').read()
            myID = pri.get_labels()
            message = cert + myID.encode('utf-8')
            #message = cert
            c.send(message)

        elif message == 'Connected to server, need to exchange public keys':
            received_cert = c.recv(1024)
            server_cert = received_cert

            # Create directory pubKeys to store neighbor node's public key certificates to use for secret derivation
            if not os.path.exists('pubKeys/'):
                os.mkdir('pubKeys/')

            # Save server public key certificate to pubKeys/{client_mesh_name}.der
            with open(f'pubKeys/{client_mesh_name}.der', 'wb') as writer:
                writer.write(server_cert)

            print('Sending my public key')
            cert = open(local_cert, 'rb').read()
            message = cert
            c.send(message)

        elif message == 'Connected to server, need to exchange ID':
            received_message = c.recv(1024)
            servID = received_message.decode('utf-8')

            print('Sending my ID')
            myID = pri.get_labels()
            message = myID.encode('utf-8')
            c.send(message)

        # Initialization
        filetable = pd.read_csv('auth/dev.csv')
        #servID = filetable[filetable['IP'] == server_ip]['ID'].iloc[0] # Get the server ID
        #secret_filename = f'secrets/secret_{servID}.der'
        #secret_byte = open(secret_filename, 'rb').read()

        # Derive secret key and store it to secret_{servID}.der
        #salt = os.urandom(16) # Random 16 byte salt
        #print('Deriving secret')
        #pri.derive_ecdh_secret('', servID, mut.local_cert, salt)
        #secret_byte = pri.decrypt_file(secret_filename, mut.local_cert, salt)
        try:
            secret_byte = pri.derive_ecdh_secret('', client_mesh_name)
            if logger:
                logger.info("Secret key derived in client for server %s", server_ip)
        except Exception as e:
            print("Secret key derivation failed in client for server ", server_ip, "with exception ", e)
            traceback.print_exc()
            if logger:
                logger.error("Secret key derivation failed in client for server %s with exception %s", server_ip,
                             e)
            #return
        secret = int.from_bytes(secret_byte, byteorder=sys.byteorder)
        print("Secret = ", secret)
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
            print('Checkpoint 1, flag_ctr = ', flag_ctr)
            request = c.recv(1024).decode()
            print('Checkpoint 2, request = ', request)
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
        # Test
        print("Test partial_result: ", partial_result)
        print("Share storage cost: ", sys.getsizeof(sent_shares))
    except (ConnectionRefusedError, OSError):
        return_dict = 3
        if logger:
            logger.error("Connection refused from server (%s, 9999)", server_ip)
        #return 3 # Check if this needs to be returned at all

