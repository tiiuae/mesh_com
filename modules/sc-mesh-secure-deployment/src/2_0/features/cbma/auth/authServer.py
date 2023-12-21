import socket
import ssl
import threading
from tools.utils import *
import sys

sys.path.insert(0, '../')
from tools.verification_tools import *
from tools.custom_logger import CustomLogger
from tools.utils import wait_for_interface_to_be_pingable
import glob

logger_instance = CustomLogger("Server")
logger = logger_instance.get_logger()


class AuthServer:
    CLIENT_TIMEOUT = 60

    def __init__(self, interface, ip_address, port, cert_path, ca_path, mua):
        threading.Thread.__init__(self)
        self.running = True
        self.ipAddress = ip_address
        self.port = port
        self.CERT_PATH = cert_path
        self.ca = ca_path
        self.interface = interface
        self.mymac = get_mac_addr(self.interface)
        # Create the SSL context here and set it as an instance variable
        self.context = ssl.create_default_context(purpose=ssl.Purpose.CLIENT_AUTH,
                                                  cafile=glob.glob(self.ca)[0])
        self.context.minimum_version = ssl.TLSVersion.TLSv1_3
        self.context.check_hostname = False
        self.context.verify_mode = ssl.CERT_REQUIRED
        self.context.load_cert_chain(
            certfile=glob.glob(f"{self.CERT_PATH}/MAC/{self.mymac}.crt")[0],
            keyfile=glob.glob(f"{self.CERT_PATH}/private.key")[0],
        )
        self.client_auth_results = {}
        self.active_sockets = {}
        self.client_auth_results_lock = threading.Lock()
        self.active_sockets_lock = threading.Lock()
        self.mua = mua

    def handle_client(self, client_connection, client_address):
        client_mac = extract_mac_from_ipv6(client_address[0])  # TODO: check if it is safe to do so
        print("------------------server---------------------")
        if client_mac not in self.mua.connected_peers_status:
            with self.mua.connected_peers_status_lock:
                self.mua.connected_peers_status[client_mac] = ["ongoing",0]  # Update status as ongoing, num of failed attempts = 0
        else:
            with self.mua.connected_peers_status_lock:
                self.mua.connected_peers_status[client_mac][0] = "ongoing"  # Update status as ongoing, num of failed attempts = same as before
        self.authenticate_client(client_connection, client_address, client_mac)

    def authenticate_client(self, client_connection, client_address, client_mac):
        secure_client_socket = self.context.wrap_socket(client_connection, server_side=True)
        secure_client_socket.settimeout(self.CLIENT_TIMEOUT)

        try:
            client_cert = secure_client_socket.getpeercert(binary_form=True)
            if not client_cert:
                logger.error(f"Unable to get the certificate from the client {client_address[0]}", exc_info=True)
                raise CertificateNoPresentError("Unable to get the certificate from the client")

            auth = verify_cert(client_cert, self.ca, client_address[0], self.interface, logger)
            with self.client_auth_results_lock:
                self.client_auth_results[client_address[0]] = auth
            if auth:
                with self.active_sockets_lock:
                    self.active_sockets[client_address[0]] = secure_client_socket
                self.mua.auth_pass(secure_client_socket=secure_client_socket, client_mac=client_mac)
            else:
                # Handle the case when authentication fails, maybe send an error message
                self.mua.auth_fail(client_mac=client_mac)
                secure_client_socket.close()
                # secure_client_socket.sendall(b"Authentication failed.")
        except Exception:
            logger.error(f"An error occurred while handling the client {client_address[0]}.", exc_info=True)
            self.mua.auth_fail(client_mac=client_mac)
            secure_client_socket.close()

    def get_secure_socket(self, client_address):
        with self.active_sockets_lock:
            return self.active_sockets.get(client_address)

    def get_client_auth_result(self, client_address):
        with self.client_auth_results_lock:
            return self.client_auth_results.get(client_address, None)

    def start_server(self):
        if is_ipv4(self.ipAddress):
            self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            self.serverSocket.bind((self.ipAddress, self.port))
        elif is_ipv6(self.ipAddress):
            self.serverSocket = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
            self.serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            scope_id = socket.if_nametoindex(self.interface)
            self.serverSocket.bind((self.ipAddress, int(self.port), 0, scope_id))
        else:
            raise ValueError("Invalid IP address")

        self.serverSocket.listen()
        logger.info("Server listening")

        while self.running and not self.mua.shutdown_event.is_set():
            try:
                client_connection, client_address = self.serverSocket.accept()
                threading.Thread(target=self.handle_client, args=(client_connection, client_address)).start()
            except Exception as e:
                if self.running:
                    logger.error("Unexpected error in server loop.", exc_info=True)

    def stop_server(self):
        self.running = False
        if is_ipv4(self.ipAddress):
            serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            serverSocket.bind((self.ipAddress, self.port))
        elif is_ipv6(self.ipAddress):
            serverSocket = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
            scope_id = socket.if_nametoindex(self.interface)
            serverSocket.bind((self.ipAddress, int(self.port), 0, scope_id))
        if hasattr(self, "serverSocket"):
            self.serverSocket.close()
            for sock in self.active_sockets.values():
                sock.close()
