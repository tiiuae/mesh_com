import socket
import threading
import random
import pickle
import time
import logging
import numpy as np
import os
from getmac import get_mac_address
from mat4py import loadmat

class PHYCRA:
    def __init__(self):
        log_filename = '/tmp/server_log.txt'  # Set the log file location to /tmp/

        # Clear the server log at the start of each session
        if os.path.exists(log_filename):
            os.remove(log_filename)

        logging.basicConfig(level=logging.INFO, filename=log_filename, filemode='w',
                            format='%(asctime)s - %(levelname)s - %(message)s')
        logging.info("Server initialized")
        # Server setup
        try:
            DataServer = loadmat('ACF_Table.mat')
            self.acf = np.array(DataServer['y_C']).transpose()
            self.SERVER = self.get_server_ip()
            self.BROADCAST_PORT = 5051
            self.PORT = 5050
            self.ADDR = (self.SERVER, self.PORT)
            self.FORMAT = 'utf-8'
            logging.info("Server setup completed successfully")
        except Exception as e:
            logging.error("Error during server setup: %s", e)

        # Client setup
        try:
            DataClient = loadmat('ACF_Table.mat')
            self.acf_client = np.array(DataClient['y_C']).transpose()
            logging.info("Client setup completed successfully")
        except Exception as e:
            logging.error("Error during client setup: %s", e)
        # Create an event to signal when 5 minutes have passed
        self.stop_event = threading.Event()

        # Start server and client functionalities
        try:
            self.server_thread = threading.Thread(target=self.server_start)
            self.server_thread.start()
            logging.info("Server thread started")
            time.sleep(2)
            self.listen_thread = threading.Thread(target=self.listen_for_broadcast)
            self.listen_thread.start()
            logging.info("Broadcast listening thread started")
        except Exception as e:
            logging.error("Error during server/client threads initialization: %s", e)

        # Start a timer to stop the server and broadcasting after 5 minutes
        timer = threading.Timer(60, self.stop_event.set)
        timer.start()
        logging.info("Server shutdown timer started")
        
    # SERVER FUNCTIONS

    def log_authentication(self, node_ip, mac_address, result):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        log_entry = f"{timestamp}\t{node_ip}\t{mac_address}\t{result}\n"
        with open("server_log.txt", "a") as log_file:
            log_file.write(log_entry)
            

    def display_table(self):
        logging.info("Displaying authentication table")
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
        """
           Handles the client connection, sends a challenge (ACF value),
           and verifies the response from the client to authenticate.
           It also logs the authentication result.
        """
        try:
            # Sending a random ACF value as a challenge to the client
            index = random.randint(0, len(self.acf) - 1)
            acf_tx = pickle.dumps(self.acf[index])
            conn.sendall(acf_tx)
            # Receive and verify the index from the client
            rx_index_length_bytes = conn.recv(2)
            rx_index_length = int.from_bytes(rx_index_length_bytes, 'big')
            rx_index_bytes = conn.recv(rx_index_length)
            rx_index = int(rx_index_bytes.decode(self.FORMAT))

            # Then receive the MAC address from the client
            mac_address_length_bytes = conn.recv(2)
            mac_address_length = int.from_bytes(mac_address_length_bytes, 'big')
            mac_address_bytes = conn.recv(mac_address_length)
            mac_address = mac_address_bytes.decode(self.FORMAT)
            # Authenticate the client based on the index and MAC address received
            if rx_index == index:
               print("PASS: Authentication successful")
               logging.info("Authentication successful for %s", addr)
               self.log_authentication(addr[0], mac_address, "Success")
            else:
               print('FAIL: Authentication failed')
               logging.warning("Authentication failed for %s", addr)
               self.log_authentication(addr[0], mac_address, "Access denied")

            print("\nUpdated Table:")
            self.display_table()
        except Exception as e:
            logging.error("Error during client handling: %s", e)
        finally:
            conn.close()


    def server_start(self):
        try:
            self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server.bind(self.ADDR)
            self.server.listen()
            self.server.settimeout(1)
            logging.info("Server started and listening")
        except Exception as e:
            logging.error("Error during server binding/listening: %s", e)
            return

        try:
            self.broadcast_thread = threading.Thread(target=self.broadcast_status)
            self.broadcast_thread.start()
            logging.info("Broadcast thread started")
            while not self.stop_event.is_set():  # Check if the stop event is set
                try:
                    conn, addr = self.server.accept()
                except socket.timeout:
                    continue
                thread = threading.Thread(target=self.handle_client, args=(conn, addr))
                thread.start()
        except Exception as e:
            logging.error("Error during server operations: %s", e)
        finally:
            self.server.close()
            logging.info("Server shutdown")


    def broadcast_status(self):
        try:
            broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            while not self.stop_event.is_set():  # Check if the stop event is set
                msg = "SERVER_AVAILABLE"
                broadcast_sock.sendto(msg.encode(), ('<broadcast>', self.BROADCAST_PORT))
                time.sleep(30)
        except Exception as e:
            logging.error("Error during broadcast: %s", e)
        finally:
            broadcast_sock.close()
            logging.info("Broadcast stopped")

    
    # CLIENT FUNCTIONS

    def get_mac_address(self):
        return get_mac_address()
    

    def connect_to_server(self, server_ip):
        """
        Connects to the server, receives a challenge (ACF value),
        calculates the index of the received ACF value in the local acf_client table,
        sends back this index and the MAC address to the server for authentication.
        """
        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect((server_ip, self.PORT))
            # Receive the ACF challenge from the server
            acf_rx = pickle.loads(client.recv(4096))
            # Calculate the index of the received ACF
            index = str(np.where((self.acf_client == acf_rx).all(axis=1))[0][0])
            # Send the calculated index back to the server
            index_bytes = index.encode(self.FORMAT)
            client.sendall(len(index_bytes).to_bytes(2, 'big'))  # Send length of rx_index first
            client.sendall(index_bytes)  # Send rx_index
          # Retrieve the local MAC address and send it to the server
            mac_address = self.get_mac_address()
            mac_address_bytes = mac_address.encode(self.FORMAT)
            client.sendall(len(mac_address_bytes).to_bytes(2, 'big'))  # Send length of MAC address first
            client.sendall(mac_address_bytes)  # Then send MAC address
            logging.info("Successfully sent index and MAC address to the server")
        except Exception as e:
            logging.error("Error during connection to server: %s", e)
        finally:
            client.close()
    
    
    def listen_for_broadcast(self):
        try:
            self.listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.listen_sock.bind(('', self.BROADCAST_PORT))
            self.listen_sock.settimeout(1)
            logging.info("Listening for broadcasts")
            while not self.stop_event.is_set():
                try:
                    data, addr = self.listen_sock.recvfrom(1024)
                except socket.timeout:
                    continue
                if data.decode() == "SERVER_AVAILABLE" and addr[0] != self.SERVER:
                    self.connect_to_server(addr[0])
        except Exception as e:
            logging.error("Error during broadcast reception: %s", e)
        finally:
            self.listen_sock.close()
            logging.info("Stopped listening for broadcasts")


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
    phycra_instance = PHYCRA()
    time.sleep(60)
    phycra_instance.stop_event.set()
    phycra_instance.server_thread.join()
    phycra_instance.listen_thread.join()
    phycra_instance.broadcast_thread.join()
