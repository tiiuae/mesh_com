import socket
import server_functions
import time
import json
import crc_functions
from _thread import *


def multi_threaded_client(c, addr):
    # receive client's name
    name = c.recv(1024).decode()
    print('====================================================================')
    print('Connected with ', addr, name)
    print('====================================================================')

    c.send(bytes('Connected to server', 'utf-8'))  # Transmit tcp msg as a byte with encoding format str to client

    # Session Initializations Parameters
    secret = 1234 #this should be stored on HSM
    total_period = 20  # Total period for the session
    period = 2  # Period for continuous authentication
    time_margin = 0.2 * period  # Time margin for freshness = 20 % of period
    crc_key = '1001'  # CRC key
    start_time = time.time()  # session start time
    start_timestamp = start_time
    time_flag = 1

    received_shares = []  # Store old shares received during the session to check freshness of new share

    num_of_fails = 0  # Number of times authentication has failed
    backoff_period = 0  # back off period initialized to 0

    # Setting socket timeout = time margin to raise exception when client does not respond to authentication request
    # within time margin
    c.settimeout(time_margin)

    while time.time() - start_time <= total_period:
        if (time_flag == 1) or (time.time() - start_timestamp >= period):
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
                        # exponential backoff = auth period ^ num of failures
                        # max(period, 2) to avoid diminishing exponential when period < 1
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
            if auth_result == "pass":
                r_bin_data = msg_received[:-(len(crc_key) - 1)]
                # print('Received binary data = ', r_bin_data)

                # Convert binary to string
                msg_received_jsn = ''.join(chr(int(r_bin_data[i:i + 8], 2)) for i in range(0, len(r_bin_data), 8))

                msg_received = json.loads(msg_received_jsn)
                print('Message received = ', msg_received)

                auth_result = server_functions.authenticator(secret, crc_key, received_shares, msg_received,
                                                             time_margin, start_timestamp)
                print("Authentication ", auth_result)

            if auth_result == "pass":
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
                # exponential backoff = auth period ^ num of failures
                # max(period, 2) to avoid diminishing exponential when period < 1
                backoff_period = max(period,2) ** num_of_fails
                result_dict = {
                    "auth_result": auth_result,
                    "backoff_period": backoff_period
                }
                result = json.dumps(result_dict)
                print("Result Sent = ", result)
                c.send(bytes(result, 'utf-8'))
                backoff_start = time.time()
                # Do not receive data for backoff period
                while time.time() - backoff_start <= backoff_period:
                    pass
            print('*********************************************************************')
            print(' ')
            time_flag = time_flag + 1

    c.send(bytes('Closing connection', 'utf-8'))
    c.close()  # close client socket
    print('Connection closed')

def initiate_server(ip):
    s = socket.socket()  # create server socket s with default param ipv4, TCP
    print('Socket Created')

    # to accept connections from clients, bind IP of server, a port number to the server socket
    server_ip = socket.gethostbyname(socket.gethostname())
    # s.bind(('localhost', 9999))  # (IP, host num) #use a non busy port num
    # s.bind((server_ip, 9999)) # (IP, host num) #use a non busy port num
    # s.bind(('192.168.137.215', 9999))
    s.bind((ip, 9999))
	
    # wait for clients to connect (tcp listener)
    s.listen(3)  # buffer for only 3 connections
    print('Waiting for connections')

    while True:
    	# accept tcp connection from client
        c, addr = s.accept()  # returns client socket and IP addr
        # new_client_thread(c, addr).start()
        start_new_thread(multi_threaded_client, (c, addr))
