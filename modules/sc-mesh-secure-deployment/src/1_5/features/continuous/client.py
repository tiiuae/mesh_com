import hashlib
import json
import math
import random
import socket
import time
import pandas as pd
import sys
import os
import traceback
import subprocess
sys.path.insert(0, '../../')

from features.mutual.mutual import *
from features.mutual.utils import primitives as pri

from .functions import client_functions





def initiate_client(server_ip, ID, logger=None):
    with socket.socket() as cli_sock:  # create server socket c with default param ipv4, TCP
        partial_result = []
        local_cert = "/etc/ssl/certs/mesh_cert.der"

        # connect to server socket
        try:
            cli_sock.settimeout(10)
            cli_sock.connect((server_ip, 9999))

            if logger:
                logger.info("Client connected to server (%s, 9999)", server_ip)

            # name = input("Enter your name:")  # Taking input from client
            cli_sock.send(bytes(str(ID), 'utf-8'))  # Send client name to server

            # receive byte sent by server and print it
            message = cli_sock.recv(1024).decode()  # decode byte to string
            print(message)

            client_mesh_name = server_ip.replace('.', '_')
            print("client_mesh_name = ", client_mesh_name)

            if message == 'Connected to server, need to exchange public keys and ID':
                received_cert = cli_sock.recv(1024)
                server_cert = received_cert[:-8]
                #servID = received_cert[-8:].decode('utf-8')
                #server_cert = received_cert

                # Create directory pubKeys to store neighbor node's public key certificates to use for secret derivation
                if not os.path.exists('pubKeys/'):
                    os.mkdir('pubKeys/')

                # Save server public key certificate to pubKeys/{client_mesh_name}.der
                with open(f'pubKeys/{client_mesh_name}.der', 'wb') as writer:
                    writer.write(server_cert)

                print('Sending my public key and ID')
                with open(local_cert, 'rb') as file:
                    cert = file.read()
                my_ID = pri.get_labels()
                message = cert + my_ID.encode('utf-8')
                #message = cert
                cli_sock.send(message)

            elif message == 'Connected to server, need to exchange public keys':
                received_cert = cli_sock.recv(1024)
                server_cert = received_cert

                # Create directory pubKeys to store neighbor node's public key certificates to use for secret derivation
                if not os.path.exists('pubKeys/'):
                    os.mkdir('pubKeys/')

                # Save server public key certificate to pubKeys/{client_mesh_name}.der
                with open(f'pubKeys/{client_mesh_name}.der', 'wb') as writer:
                    writer.write(server_cert)

                print('Sending my public key')
                with open(local_cert, 'rb') as file:
                    cert = file.read()
                message = cert
                cli_sock.send(message)

            elif message == 'Connected to server, need to exchange ID':
                received_message = cli_sock.recv(1024)
                servID = received_message.decode('utf-8')

                print('Sending my ID')
                my_ID = pri.get_labels()
                message = my_ID.encode('utf-8')
                cli_sock.send(message)

            # Initialization
            #filetable = pd.read_csv('auth/dev.csv')
            try:
                secret_byte = pri.derive_ecdh_secret('', client_mesh_name)
                if logger:
                    logger.info("Secret key derived in client for server %s", server_ip)
            except Exception as exp:
                print("Secret key derivation failed in client for server ", server_ip, "with exception ", exp)
                traceback.print_exc()
                if logger:
                    logger.error("Secret key derivation failed in client for server %s with exception %s", server_ip,
                                 exp)
                #return
            secret = int.from_bytes(secret_byte, byteorder=sys.byteorder)
            print("Secret = ", secret)
            #secret = 1234
            server_id = server_ip
            client_id = socket.gethostbyname(socket.gethostname())
            crc_key = '1001'  # CRC key
            #start_time = time.time()
            #timestamp = start_time  # Gives current timestamp in seconds
            time_flag = 1  # Initialization of time flag
            max_count = 2
            flag_ctr = 0

            #period = 2  # Period for each authentication in seconds
            #total_period = 20  # Total period for the session
            sent_shares = []  # Store shares sent during the session to prevent duplicate shares
            #msg_size = 0  # Initialized to compute avg message size

            # Generate k degree polynomial for the secret
            #auth_result = "pass"  # initialization
            #auth_result = 1  # initialization
            while flag_ctr <= max_count:
                print('Checkpoint 1, flag_ctr = ', flag_ctr)
                request = cli_sock.recv(1024).decode()
                print('Checkpoint 2, request = ', request)
                if request == 'Request to authenticate':
                    print("Processing authentication request")
                    # Generate share and share authenticator
                    while True:  # Select random x until unique share is generated
                        #rand_num = random.randint(1, 9999)  # Generate random x
                        command = ['od', '-An', '-N4', '-i', '/dev/random'] # Read a 4 byte random number from /dev/random
                        out = subprocess.run(command, shell=False, capture_output=True, text=True)
                        rand_num = int(out.stdout)
                        shared_sec = secret + time_flag + rand_num  # Share = secret + time flag + random number x
                        if sent_shares.count(shared_sec) == 0:  # Check that new share has not been sent previously
                            sent_shares.append(shared_sec)
                            break
                    # Share authenticator = hash(share - secret - timeflag) = hash(x)
                    share_aut = hashlib.sha3_256(bytes(str(rand_num), 'utf-8')).digest()

                    print("share u = ", shared_sec)
                    print("Share Authenticator sa = ", share_aut)

                    msg = f"Continuous Authentication {str(time_flag)}"

                    # Generate message with authentication tokens
                    msg_to_send = client_functions.message_generator(secret, server_id, client_id, msg, shared_sec, time_flag, share_aut)
                    # Add CRC
                    msg_with_crc = client_functions.crc_generator(msg_to_send, crc_key)

                    # Convert from binary to bytes
                    msg_with_crc_bytes = int(msg_with_crc, 2).to_bytes(math.ceil(len(msg_with_crc) / 8), byteorder='big')
                    # print("Message sent in bytes = ", msg_with_crc_bytes)
                    print("Size of message in bytes = ", sys.getsizeof(msg_with_crc_bytes))
                    # msg_size = msg_size + sys.getsizeof(msg_with_crc_bytes)

                    cli_sock.send(msg_with_crc_bytes)  # Send message to server

                    time_flag = time_flag + 1

                    while True:
                        # Resend message to server if CRC fails
                        result = cli_sock.recv(1024).decode()
                        if result != 'Resend message':
                            break
                        print("Resending message")
                        cli_sock.send(msg_with_crc_bytes)  # Resend message to server
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
        except (ConnectionRefusedError, OSError, socket.timeout):
            if logger:
                logger.error("Connection refused from server (%s, 9999)", server_ip)
            #return 3 # Check if this needs to be returned at all