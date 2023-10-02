import socket
import threading
import random
import pickle
#import scipy.io
from mat4py import loadmat
import time
import logging
import numpy as np
import os
from getmac import get_mac_address  # <-- Import the get_mac_address function

class PHYCRA:
    def __init__(self):
        logging.basicConfig(level=logging.INFO)
        logging.getLogger().setLevel(logging.INFO)

        # Server setup
        DataServer = loadmat('ACF_Table.mat')
        self.acf = np.array(DataServer['y_C']).transpose()
        #self.acf = DataServer['y_C'].transpose()
        self.SERVER = self.get_server_ip()
        self.BROADCAST_PORT = 5051
        self.PORT = 5050
        self.ADDR = (self.SERVER, self.PORT)
        self.FORMAT = 'utf-8'

        # Client setup
        DataClient = loadmat('ACF_Table.mat')
        self.acf_client = np.array(DataClient['y_C']).transpose() 
        #self.acf_client = DataClient['y_C'].transpose()
        
        # Clear the server log at the start of each session
        if os.path.exists("server_log.txt"):
            os.remove("server_log.txt")

        # Start server and client functionalities
        server_thread = threading.Thread(target=self.server_start)
        server_thread.start()
        time.sleep(2)
        listen_thread = threading.Thread(target=self.listen_for_broadcast)
        listen_thread.start()

    # SERVER FUNCTIONS

    def log_authentication(self, node_ip, mac_address, result):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        log_entry = f"{timestamp}\t{node_ip}\t{mac_address}\t{result}\n"
        with open("server_log.txt", "a") as log_file:
            log_file.write(log_entry)

    def display_table(self):
        print("+---------------------+---------------+-------------------+---------------------------+")
        print("|        Time         |     Node IP   |    MAC Address    |   Authentication Result   |")
        print("+---------------------+---------------+-------------------+---------------------------+")
        with open("server_log.txt", "r") as log_file:
            for line in log_file:
                timestamp, node_ip, mac_address, result = line.strip().split("\t")
                formatted_line = f"| {timestamp:<19} | {node_ip:<12} | {mac_address:<15} | {result:<23} |"
                print(formatted_line)
        print("+---------------------+---------------+-------------------+---------------------------+")

    def handle_client(self, conn, addr):
        print(f"Connection request received from {addr}")
        index = random.randint(0, len(self.acf) - 1)
        acf_tx = pickle.dumps(self.acf[index])
        conn.send(acf_tx)
        
        # Receive and convert rx_index to int
        rx_index_length = int.from_bytes(conn.recv(2), 'big')
        rx_index = int(conn.recv(rx_index_length).decode(self.FORMAT))
        
        # Then receive MAC address
        mac_address_length = int.from_bytes(conn.recv(2), 'big')
        mac_address = conn.recv(mac_address_length).decode(self.FORMAT)
        
        if rx_index == index:
            print("Node is authenticated")
            self.log_authentication(addr[0], mac_address, "Success")
        else:
            print('Access denied')
            self.log_authentication(addr[0], mac_address, "Access denied")
        print("\nUpdated Table:")
        self.display_table()
        conn.close()

    def server_start(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(self.ADDR)
        server.listen()
        broadcast_thread = threading.Thread(target=self.broadcast_status)
        broadcast_thread.start()
        while True:
            conn, addr = server.accept()
            thread = threading.Thread(target=self.handle_client, args=(conn, addr))
            thread.start()

    def broadcast_status(self):
        broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        while True:
            msg = "SERVER_AVAILABLE"
            broadcast_sock.sendto(msg.encode(), ('<broadcast>', self.BROADCAST_PORT))
            time.sleep(60)

    # CLIENT FUNCTIONS

    def get_mac_address(self):
        return get_mac_address()

    def connect_to_server(self, server_ip):
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.connect((server_ip, self.PORT))
        acf_rx = pickle.loads(client.recv(4096))
        index = str(np.where((self.acf_client == acf_rx).all(axis=1))[0][0])
        client.send(len(index).to_bytes(2, 'big'))  # Send length of rx_index first
        client.send(index.encode(self.FORMAT))  # Send rx_index
        mac_address = self.get_mac_address()
        client.send(len(mac_address).to_bytes(2, 'big'))  # Send length of MAC address
        client.send(mac_address.encode(self.FORMAT))  # Then send MAC address
        client.close()

    def listen_for_broadcast(self):
        listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        listen_sock.bind(('', self.BROADCAST_PORT))
        while True:
            data, addr = listen_sock.recvfrom(1024)
            if data.decode() == "SERVER_AVAILABLE" and addr[0] != self.SERVER:
                self.connect_to_server(addr[0])

    # Common Functions
    def get_server_ip(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(('10.254.254.254', 1))
            IP = s.getsockname()[0]
        except:
            IP = '127.0.0.1'
        finally:
            s.close()
        return IP

if __name__ == "__main__":
    phycra = PHYCRA()
