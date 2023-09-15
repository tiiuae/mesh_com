import json
import socket
import time
from threading import Thread, Lock

from .functions import crc_functions
from .functions import server_functions
import pandas as pd
import sys
import os
import traceback

sys.path.insert(0, '../../')

from features.mutual.mutual import *
from features.mutual.utils import primitives as pri

def multi_threaded_client(c, addr, lock, return_dict, id_dict, ips_sectable, logger):
    #lock.acquire()
    partial_res = []
    # receive client's name
    name = c.recv(1024).decode()
    print('====================================================================')
    print('Connected with ', addr, name)
    print('====================================================================')


    client_mesh_ip = addr[0]
    print("client_mesh_ip = ", client_mesh_ip)
    client_mesh_name = client_mesh_ip.replace('.', '_')
    print("client_mesh_name = ", client_mesh_name)

    local_cert = "/etc/ssl/certs/mesh_cert.der"

    #mut = Mutual('wlan1')
    if (not os.path.isfile(f'pubKeys/{client_mesh_name}.der')) and (client_mesh_ip not in ips_sectable):
        print('Public key does not exist, notifying client to exchange public keys')
        print('Node not in auth table, notifying client to exchange ID')
        c.send(bytes('Connected to server, need to exchange public keys and ID', 'utf-8'))

        print('Sending my public key and ID')
        with open(local_cert, 'rb') as file:
            cert = file.read()
        my_ID = pri.get_labels()
        message = cert + my_ID.encode('utf-8')
        #message = cert
        c.send(message)

        received_cert = c.recv(1024)
        client_cert = received_cert[:-8]
        cli_ID = received_cert[-8:].decode('utf-8')
        #client_cert = received_cert
        id_dict[addr[0]] = cli_ID # Used to insert row in auth table

        # Create directory pubKeys to store neighbor node's public key certificates to use for secret derivation
        if not os.path.exists('pubKeys/'):
            os.mkdir('pubKeys/')

        # Save client public key certificate to pubKeys/{cli_ID}.der
        with open(f'pubKeys/{client_mesh_name}.der', 'wb') as writer:
            writer.write(client_cert)

    elif not os.path.isfile(f'pubKeys/{client_mesh_name}.der'):
        print('Public key does not exist, notifying client to exchange public keys')
        c.send(bytes('Connected to server, need to exchange public keys', 'utf-8'))

        print('Sending my public key')
        with open(local_cert, 'rb') as file:
            cert = file.read()
        message = cert
        c.send(message)

        received_cert = c.recv(1024)
        client_cert = received_cert

        # Create directory pubKeys to store neighbor node's public key certificates to use for secret derivation
        if not os.path.exists('pubKeys/'):
            os.mkdir('pubKeys/')

        # Save client public key certificate to pubKeys/{cli_ID}.der
        with open(f'pubKeys/{client_mesh_name}.der', 'wb') as writer:
            writer.write(client_cert)

    elif client_mesh_ip not in ips_sectable:
        print('Node not in auth table, notifying client to exchange ID')
        c.send(bytes('Connected to server, need to exchange ID', 'utf-8'))

        print('Sending my ID')
        my_ID = pri.get_labels()
        message = my_ID.encode('utf-8')
        c.send(message)

        received_message = c.recv(1024)
        cli_ID = received_message.decode('utf-8')
        id_dict[addr[0]] = cli_ID  # Used to insert row in auth table
    else:
        c.send(bytes('Connected to server', 'utf-8'))  # Transmit tcp msg as a byte with encoding format str to client

    # Derive secret key and store it to secret_{cli_ID}.der
    #salt = os.urandom(16)  # Random 16 byte salt
    #print('Deriving secret')
    #pri.derive_ecdh_secret('', cli_ID, mut.local_cert, salt)

    # Session Initializations Parameters
    #secret_byte = open(secret_filename, 'rb').read()
    #secret_byte = pri.decrypt_file(secret_filename, mut.local_cert, salt)
    secret_byte = None
    try:
        secret_byte = pri.derive_ecdh_secret('', client_mesh_name)
        if logger:
            logger.info("Secret key derived in server for client %s", client_mesh_ip)
    except Exception as e:
        print("Secret key derivation failed in server for client ", client_mesh_ip, "with exception ", e)
        traceback.print_exc()
        if logger:
            logger.error("Secret key derivation failed in server for client %s with exception %s", client_mesh_ip, e)
        #lock.release()
        #return

    secret = int.from_bytes(secret_byte, byteorder=sys.byteorder)
    print("Secret = ", secret)
    period = 1  # Period for continuous authentication
    time_margin = 0.4 * period  # Time margin for freshness = 40 % of period
    crc_key = '1001'  # CRC key
    start_time = time.time()  # session start time
    start_timestamp = start_time
    time_flag = 0
    max_count = 2

    received_shares = []  # Store old shares received during the session to check freshness of new share

    num_of_fails = 0  # Number of times authentication has failed

    # Setting socket timeout = time margin to raise exception when client does not respond to authentication request
    # within time margin
    c.settimeout(time_margin)

    #while time.time() - start_time <= total_period:
    while time_flag <= max_count:
        #print('Checkpoint server inside while, client = ', client_mesh_ip, ', server time elapsed = ', time.time() - start_timestamp)
        #if (time_flag == 1) or (time.time() - start_timestamp >= period):
        if (time_flag == 0) or (time.time() - start_timestamp >= period):
            print('Checkpoint server inside while if, client = ', client_mesh_ip, ', time_flag = ', time_flag)
            auth_result = "pass"  # initialized as pass
            c.send(bytes('Request to authenticate', 'utf-8'))  # Request to authenticate client
            print('Requested client for authentication')
            start_timestamp = time.time()
            # receive message until CRC is pass
            num_of_crc_fails = 0  # Initializing num of crc fails to 0
            while True:
                try:
                    received_byte = c.recv(1000000)  # received authentication message byte
                except socket.timeout:
                    # Authentication fail when client does not respond to authentication request within time margin
                    print("Authentication message not received from client")
                    auth_result = "fail"
                    break

                #print('Checkpoint received_byte: ', received_byte)

                # convert byte into binary bits
                msg_received = '0' + bin(int.from_bytes(received_byte, byteorder='big'))[2:].zfill(8)
                # print("Message received = ", msg_received)

                # CRC check
                rem = crc_functions.decodeData(msg_received, crc_key)  # Remainder
                if int(rem) == 0:
                    print("CRC check pass")
                    auth_result = "pass"
                    break
                else:
                    print("CRC check fail")
                    num_of_crc_fails = num_of_crc_fails + 1
                    # Ask client to resend up to 5 fails, then block the client
                    if num_of_crc_fails <= 5:
                        print("Asking client to resend message")
                        c.send(bytes('Resend message', 'utf-8'))  # Ask client to resend message if CRC fails
                        start_timestamp = time.time()  # restart timer
                    else:
                        num_of_fails = num_of_fails + 1
                        backoff_period = max(period, 2) ** num_of_fails
                        result_dict = {
                            "auth_result": "fail",
                            "backoff_period": backoff_period
                        }
                        result = json.dumps(result_dict)
                        print("Result Sent = ", result)
                        c.send(bytes(result, 'utf-8'))
                        backoff_start = time.time()
                        # Do not receive data for backoff period
                        while time.time() - backoff_start <= backoff_period:
                            pass
                        c.send(bytes('Request to authenticate', 'utf-8'))  # Request to authenticate client
                        print('Requested client for authentication')
                        start_timestamp = time.time()
                        print('*********************************************************************')
                    print(' ')

            # Avoid following block of code when auth failed in earlier steps
            if auth_result == "pass" or auth_result == 1:
                r_bin_data = msg_received[:-(len(crc_key) - 1)]
                msg_received_jsn = ''.join(chr(int(r_bin_data[i:i + 8], 2)) for i in range(0, len(r_bin_data), 8))
                msg_received = json.loads(msg_received_jsn)
                print('Message received = ', msg_received)

                auth_result = server_functions.authenticator(secret, crc_key, received_shares, msg_received,
                                                             time_margin, start_timestamp)
                print("Authentication ", auth_result)

            if auth_result == "pass" or auth_result == 1:
                num_of_fails = 0  # reset no of failures to 0
                backoff_period = 0  # reset backoff time to 0
                result_dict = {
                    "auth_result": auth_result,
                    "backoff_period": backoff_period
                }
                result = json.dumps(result_dict)
                print("Result Sent = ", result)
                c.send(bytes(result, 'utf-8'))
            else:
                num_of_fails = num_of_fails + 1
                if auth_result == "fail":
                    auth_result = 0
                # exponential backoff = auth period ^ num of failures
                # max(period, 2) to avoid diminishing exponential when period < 1
                backoff_period = max(period, 2) ** num_of_fails
                result_dict = {
                    "auth_result": auth_result,
                    "backoff_period": backoff_period
                }
                result = json.dumps(result_dict)
                print("Result Sent = ", result)
                c.send(bytes(result, 'utf-8'))
                # backoff_start = time.time()
                # # Do not receive data for backoff period
                # while time.time() - backoff_start <= backoff_period:
                #     pass
            print('*********************************************************************')
            print(' ')
            partial_res.append(auth_result)
            """
            if time_flag == max_count:
                c.send(bytes('Closing connection', 'utf-8'))
                c.close()  # close client socket
                print('Connection closed')
                return_dict[addr[0]] = partial_res
                print("Test return_dict: ", return_dict)
                lock.release()
                break
            """
            time_flag = time_flag + 1

    c.send(bytes('Closing connection', 'utf-8'))
    c.close()  # close client socket
    print('Connection closed')
    return_dict[addr[0]] = partial_res
    print("Test return_dict: ", return_dict)

    #print("Test partial_res server: ", partial_res)
    #print('Checkpoint, server outside while loop time elapsed = ', time.time() - start_time)
    print('================================== Checkpoint, end of server for client', addr[0])
    #ock.release()

def initiate_server(ip, return_dict, id_dict, num_neighbors, ips_sectable, logger=None):

    with socket.socket() as s:  # create server socket s with default param ipv4, TCP
        try:
            s.settimeout(15)  # Setting timeout to prevent infinite blocking at s.accept() when the client node is not on
            # to accept connections from clients, bind IP of server, a port number to the server socket
            s.bind((ip, 9999))
            print('Socket Created')

            # wait for clients to connect (tcp listener)
            s.listen(3)  # buffer for only 3 connections
            print('Waiting for connections')

            if logger:
                logger.info("Server socket created at (%s, 9999)", ip)
        except socket.error:
            print("Server socket creation failed")
            if logger:
                logger.error("Server socket creation failed")
            return

        threads = []

        #while True:
        for i in range(0, num_neighbors):
            print("Server inside for loop, i = ", i)
            lock = Lock()
            # accept connection from client
            c, addr = s.accept()
            print('Connected to client', c)
            print('Client address:', addr)
            print(' ')
            return_dict[addr[0]] = []
            # start thread to handle client
            thread = Thread(target=multi_threaded_client, args=(c, addr, lock, return_dict, id_dict, ips_sectable, logger), daemon=True)
            threads.append(thread)
            thread.start()

        for thread in threads:
            thread.join()
        #c.send(bytes('Closing connection', 'utf-8'))
        #c.close()  # close client socket
        print('Connection closed')

