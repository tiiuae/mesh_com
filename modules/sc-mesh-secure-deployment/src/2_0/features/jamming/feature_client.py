import json
import socket
import threading
import time
from abc import ABC, abstractmethod


class FeatureClient(ABC, threading.Thread):
    def __init__(self, node_id: str, host: str, port: int) -> None:
        """
        Initialize the FeatureClient object.

        :param node_id: An integer representing the node ID.
        :param host: A string representing the host address.
        :param port: An integer representing the port number.
        """
        super().__init__()
        self.node_id = node_id
        self.host = host
        self.port = port
        self.running = threading.Event()
        self.switching = threading.Event()
        self.socket = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)

    @abstractmethod
    def run(self) -> None:
        self.socket.connect((self.host, self.port))
        self.running.set()
        receive_thread = threading.Thread(target=self.receive_messages)
        receive_thread.start()

    @abstractmethod
    def run_client_fsm(self) -> None:
        pass

    @abstractmethod
    def receive_messages(self) -> None:
        """
        Receive messages from the socket server.
        """
        while True:
            try:
                message = self.socket.recv(1024).decode()
                if not message:
                    print("No message... break")
                    break

                # Split the received message into individual JSON objects
                json_objects = message.strip().split('\n')
                for json_object_str in json_objects:
                    try:
                        print(f"Received message: {message}")
                        json_object = json.loads(json_object_str)
                        action = json_object.get("action")

                        # Jamming Policy
                        if action == "broadcast":
                            print(f"Broadcast message received...")

                    except json.JSONDecodeError as e:
                        print(f"Failed to decode JSON: {e}")

            except ConnectionResetError:
                print("Connection forcibly closed by the remote host")
                break


    @abstractmethod
    def send_messages(self, action) -> None:
        """
        Send message to the server.

        :param action: Action to be taken by client.
        """
        data = [{'action': 'broadcast', 'node_id': self.node_id},
                {'action': 'broadcast', 'node_id': self.node_id}]

        for message in data:
            json_str = json.dumps(message)
            self.socket.send(json_str.encode())
            print("Sent message to server")
            time.sleep(5)

    @abstractmethod
    def stop(self) -> None:
        self.running.clear()
        self.socket.close()
        self.join()
