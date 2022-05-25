import hashlib
import hmac
import random
import numpy as np
import json
import ntplib
import time
import crc_functions


def message_generator(secret, server_id, client_id, msg, u, time_flag, sa):
    # msg_to_mac = {server id,client id,message,share ui, time_flag}
    # msg_to_mac = str(client_id) + ',' + str(server_id) + ',' + msg + ',' + str(u) + ',' + ',' + str(time_flag)

    msg_to_mac_dict = {
        "client_id": client_id,
        "server_id": server_id,
        "msg": msg,
        "u": int(u),
        "time_flag": time_flag
    }

    # convert dict to json
    msg_to_mac = json.dumps(msg_to_mac_dict)
    print("Message to MAC = ", msg_to_mac)
    # mac = MAC with secret as key (server id,client id,message,share u, time_flag)
    mac = hmac.new(bytes(str(secret), 'utf-8'), msg_to_mac.encode('utf-8'), hashlib.sha256).digest()
    print("MAC =", mac)

    # message to send = {server id,client id,message,share u, timestamp, sa, MAC(server id,client id,message,share u)secret}
    # msg_to_send = msg_to_mac + ',' + str(sa) + ',' + str(mac)
    msg_to_send_dict = msg_to_mac_dict.copy()
    msg_to_send_dict["sa"] = str(sa)
    msg_to_send_dict["mac"] = str(mac)
    msg_to_send = json.dumps(msg_to_send_dict)
    print("Message to send = ", msg_to_send)
    return msg_to_send


def crc_generator(msg_to_send, crc_key):
    bin_data = ''.join(format(ord(i), '08b') for i in msg_to_send)  # Convert json to binary
    print('Binary data = ', bin_data)
    data_with_crc = crc_functions.encodeData(bin_data, crc_key)
    print('Data with crc = ', data_with_crc)
    return data_with_crc
