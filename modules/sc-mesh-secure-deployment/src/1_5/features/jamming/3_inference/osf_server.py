import json
import logging
import signal
import socket
import sys
import threading
from typing import List, Tuple, Dict, Any

logging.basicConfig(level=logging.INFO)


class Client(threading.Thread):
    def __init__(self, socket: socket.socket, address: Tuple[str, int], clients: List["Client"]):
        """
        Initializes the Client object.

        :param socket: The connected socket for the client.
        :param address: The address of the client.
        :param clients: A list of all connected clients.
        """
        threading.Thread.__init__(self)
        self.socket = socket
        self.address = address
        self.running = True
        self.clients = clients
        self.start()

    def run(self) -> None:
        """
        Handles incoming messages from the client.
        """
        while self.running:
            try:
                message = self.socket.recv(1024).decode()
                if not message:
                    break

                json_object = json.loads(message)
                action = json_object.get("action")
                if action == "broadcast":
                    self.broadcast(json_object)
            except ConnectionResetError:
                logging.warning("Connection forcibly closed by the remote host")
                break
            except ValueError:
                logging.warning("Message is not a JSON", exc_info=True)
                break

    def broadcast(self, message: Dict[str, Any]) -> None:
        """
        Sends a message to all connected clients except for the sender.

        :param message: The message to broadcast.
        """
        for client in self.clients[:]:  # create a copy of the list for safe iteration
            if client != self:
                try:
                    client.socket.sendall(json.dumps(message).encode())
                except BrokenPipeError:
                    print("Broken pipe error, client disconnected:", client.address)
                    self.clients.remove(client)

    def stop(self) -> None:
        """
        Stops the Client thread and closes the socket and database connections.
        """
        self.running = False
        self.socket.close()


class Server:
    def __init__(self, host: str, port: int):
        """
        Initializes the Server object.

        :param host: The host address to bind the server to.
        :param port: The port number to bind the server to.
        """
        self.host = host
        self.port = port
        self.clients: List[Client] = []

    def start(self) -> None:
        """
        Starts the server and listens for incoming client connections.
        """
        signal.signal(signal.SIGINT, self.signal_handler)

        serversocket = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
        serversocket.bind((self.host, self.port))
        serversocket.listen(5)
        logging.info("server started and listening")

        while True:
            c_socket, c_address = serversocket.accept()
            client = Client(c_socket, c_address, self.clients)
            print('New connection', client)
            self.clients.append(client)

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


def main():
    host, port = "fd01::1", 8080
    server = Server(host, port)
    server.start()


if __name__ == "__main__":
    main()
