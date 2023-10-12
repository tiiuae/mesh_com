import json
import logging
import signal
import socket
import sys
import threading
from abc import ABC, abstractmethod
from typing import List, Tuple, Dict, Any

logging.basicConfig(level=logging.INFO)


class FeatureServer(ABC):
    def __init__(self, host: str, port: int):
        """
        Initializes the FeatureServer object.

        :param host: The host address to bind the server to.
        :param port: The port number to bind the server to.
        """
        self.host = host
        self.port = port
        self.clients: List[FeatureClientTwin] = []
        self.serversocket = None

    @abstractmethod
    def start(self) -> None:
        """
        Starts the server and listens for incoming client connections.
        """
        signal.signal(signal.SIGINT, self.signal_handler)

        self.serversocket = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
        self.serversocket.bind((self.host, self.port))
        self.serversocket.listen(5)
        logging.info("server started and listening")

        while True:
            c_socket, c_address = self.serversocket.accept()
            client = FeatureClientTwin(c_socket, c_address, self.clients, self.host)
            print('New connection', client)
            self.clients.append(client)

    @abstractmethod
    def signal_handler(self, sig: signal.Signals, frame) -> None:
        """
        Handles a signal interrupt (SIGINT) and stops the server gracefully.

        :param sig: The signal received by the handler.
        :param frame: The current execution frame.
        """
        print("Attempting to close threads.")
        for client in self.clients:
            print("joining", client.address)
            client.stop()

        print("threads successfully closed")
        sys.exit(0)

    @abstractmethod
    def send_data_clients(self, message: Dict[str, Any]) -> None:
        """
        Sends a message to all connected clients except for the sender.

        :param message: The message to broadcast.
        """
        print("Broadcasting channel switch info to all nodes...")
        for client in self.clients:
            try:
                client.socket.sendall(json.dumps(message).encode())
            except BrokenPipeError:
                print("Broken pipe error, client disconnected:", client.address)
                self.clients.remove(client)

    @abstractmethod
    def run_server_fsm(self) -> None:
        """
        Runs the server finite state machine.
        """
        pass


class FeatureClientTwin(ABC, threading.Thread):
    def __init__(self, socket: socket.socket, address: Tuple[str, int], clients: List["FeatureClientTwin"], host: str):
        """
        Initializes the FeatureClientTwin object.

        :param socket: The connected socket for the client.
        :param address: The address of the client.
        :param clients: A list of all connected clients.
        """
        threading.Thread.__init__(self)
        self.socket = socket
        self.address = address
        self.running = True
        self.clients = clients
        self.host = host
        self.start()

    @abstractmethod
    def run(self):
        # Create threads for server operations
        listen_thread = threading.Thread(target=self.receive_messages)
        listen_thread.start()

    @abstractmethod
    def receive_messages(self) -> None:
        """
        Handles incoming messages from the client.
        """
        while self.running:
            try:
                message = self.socket.recv(1024).decode()
                if not message:
                    break

                # Split the received message into individual JSON objects
                json_objects = message.strip().split('\n')
                for json_object_str in json_objects:
                    try:
                        json_object = json.loads(json_object_str)
                        action = json_object.get("action")
                        client_ip = json_object.get("node_id")

                        # Policy
                        if action == "broadcast":
                            print(f"Broadcast message received...")

                    except json.JSONDecodeError as e:
                        print(f"Failed to decode JSON: {e}")

            except ConnectionResetError:
                logging.warning("Connection forcibly closed by the remote host")
                break

    @abstractmethod
    def stop(self) -> None:
        """
        Stops the Client thread and closes the socket and database connections.
        """
        self.running = False
        self.socket.close()
