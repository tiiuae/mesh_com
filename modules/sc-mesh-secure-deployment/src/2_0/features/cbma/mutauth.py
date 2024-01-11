import os
import sys
import time
import threading
import queue
import json

path_to_cbma_dir = os.path.dirname(
    __file__
)  # Path to dir containing this script
sys.path.insert(0, path_to_cbma_dir)

# pylint: disable=wrong-import-position
from auth.authServer import AuthServer  # noqa
from auth.authClient import AuthClient  # noqa

from multicast.multicast import MulticastHandler  # noqa
from tools.utils import (  # noqa
    get_mac_addr,
    mac_to_ipv6,
    setup_bridge,
    add_interface_to_batman,
    is_wpa_supplicant_running,
    run_wpa_supplicant,
    set_ipv6,
    generate_random_bytes,
    xor_bytes,
    add_interface_to_bridge,
)
from tools.custom_logger import CustomLogger  # noqa
from macsec import macsec  # noqa
from secure_channel.secchannel import SecMessageHandler  # noqa

BEACON_TIME = 10
MAX_CONSECUTIVE_NOT_RECEIVED = 2
MULTICAST_ADDRESS = "ff02::1"
TIMEOUT = 3 * BEACON_TIME
logger_instance = CustomLogger("mutAuth")


class mutAuth:
    def __init__(
        self,
        in_queue,
        level,
        meshiface,
        port,
        batman_interface,
        path_to_certificate,
        path_to_ca,
        macsec_encryption,
        shutdown_event,
        lan_bridge_flag=False,
    ):
        self.level = level
        self.meshiface = meshiface
        self.mymac = get_mac_addr(self.meshiface)
        self.ipAddress = mac_to_ipv6(self.mymac)
        self.port = port
        self.CERT_PATH = (
            path_to_certificate  # Absolute path to certificates folder
        )
        self.CA_PATH = path_to_ca  # Absolute path to ca certificate
        self.in_queue = in_queue
        self.logger = logger_instance.get_logger()
        self.multicast_handler = MulticastHandler(
            self.in_queue,
            MULTICAST_ADDRESS,
            self.port,
            self.meshiface,
            shutdown_event,
        )
        self.stop_event = threading.Event()
        self.sender_thread = threading.Thread(
            target=self._periodic_sender, args=()
        )
        self.shutdown_event = (
            shutdown_event  # Add this to handle graceful shutdown
        )
        self.batman_interface = batman_interface
        self.macsec_obj = macsec.Macsec(
            level=self.level,
            interface=self.meshiface,
            macsec_encryption=macsec_encryption
        )  # Initialize macsec object
        if self.level == "upper":
            self.bridge_interface = (
                "br-upper"  # bridge for upper macsec interfaces
            )
            setup_bridge(self.bridge_interface)
            add_interface_to_batman(
                interface_to_add=self.bridge_interface,
                batman_interface=self.batman_interface,
            )
        # key = client mac address,
        # value = [status : ("ongoing", "authenticated", "not connected"),
        #          no of failed attempts]
        self.connected_peers_status = {}
        self.connected_peers_status_lock = threading.Lock()

        # Maximum number of failed attempts for mutual authentication
        self.maximum_num_failed_attempts = 3
        self.lan_bridge_flag = lan_bridge_flag
        self.macsec_setup_event = threading.Event()

    def check_mesh(self):
        if not is_wpa_supplicant_running():
            self.logger.info("wpa_supplicant process is not running.")
            run_wpa_supplicant(self.meshiface)
            set_ipv6(self.meshiface, self.ipAddress)
        else:
            self.logger.info("wpa_supplicant process is running.")

    def _periodic_sender(self):
        while (
            not self.stop_event.is_set() and not self.shutdown_event.is_set()
        ):
            self.multicast_handler.send_multicast_message(self.mymac)
            time.sleep(BEACON_TIME)
        if self.shutdown_event.is_set():
            self.stop()

    def monitor_wpa_multicast(self):
        while not self.shutdown_event.is_set():
            try:
                source, message = self.in_queue.get(block=True, timeout=1.0)
            except queue.Empty:
                # Continue in loop to enable shutdown_event check
                continue
            if source == "WPA":
                self.logger.info("External node_connect event triggered!")
                self.logger.info("Received MAC from WPA event: %s", message)
                handle_peer_connected_thread = threading.Thread(
                    target=self.handle_wpa_multicast_event, args=(message,)
                )
                handle_peer_connected_thread.start()
            elif source == "MULTICAST":
                self.logger.info(
                    "Received MAC on multicast: %s at interface %s",
                    message,
                    self.meshiface,
                )
                handle_peer_connected_thread = threading.Thread(
                    target=self.handle_wpa_multicast_event, args=(message,)
                )
                handle_peer_connected_thread.start()

    def handle_wpa_multicast_event(self, mac):
            if self.mymac > mac:
                # I have higher mac, so I should initiate as client
                with self.connected_peers_status_lock:
                    connected_peers_status = self.connected_peers_status
                    if mac not in connected_peers_status:
                        # There is no ongoing connection with peer yet
                        # Start as client
                        print("------------------client ---------------------")
                        # Update status as ongoing, num of failed attempts = 0
                        self.connected_peers_status[mac] = ["ongoing", 0]
                    elif connected_peers_status[mac][0] not in ["ongoing", "authenticated"]:
                        # If node does not have ongoing authentication or is not already authenticated or has not been blacklisted
                        # Start as client
                        print("------------------client ---------------------")
                        # Update status as ongoing, num of failed attempts = same as before
                        self.connected_peers_status[mac][0] = "ongoing"
                    else:
                        return
                self.start_auth_client(mac)

    def start_auth_server(self):
        auth_server = AuthServer(
            self.meshiface,
            self.ipAddress,
            self.port,
            self.CERT_PATH,
            self.CA_PATH,
            self,
        )
        auth_server_thread = threading.Thread(target=auth_server.start_server)
        auth_server_thread.start()
        return auth_server_thread, auth_server

    def start_auth_client(self, server_mac):
        cli = AuthClient(
            self.meshiface,
            server_mac,
            self.port,
            self.CERT_PATH,
            self.CA_PATH,
            self,
        )
        cli.establish_connection()

    def auth_pass(self, secure_client_socket, client_mac):
        # Steps to execute if auth passes
        with self.connected_peers_status_lock:
            # Update status as authenticated, num of failed attempts
            #  = same as before
            self.connected_peers_status[client_mac][0] = "authenticated"
        self.setup_macsec(
            secure_client_socket=secure_client_socket, client_mac=client_mac
        )

    def auth_fail(self, client_mac):
        # Steps to execute if auth fails
        with self.connected_peers_status_lock:
            self.connected_peers_status[client_mac][1] = (
                self.connected_peers_status[client_mac][1] + 1
            )  # Increment number of failed attempt by 1
            self.connected_peers_status[client_mac][
                0
            ] = "not connected"  # Update status as not connected

    def setup_secchannel(self, secure_client_socket, my_macsec_param):
        # Establish secure channel and exchange macsec key
        secchan = SecMessageHandler(secure_client_socket, self.shutdown_event)
        # queue to store macsec parameters: macsec_key, port
        # from client_secchan.receive_message
        macsec_param_q = queue.Queue()
        receiver_thread = threading.Thread(
            target=secchan.receive_message, args=(macsec_param_q,)
        )
        receiver_thread.start()
        print(
            "Sending my macsec parameters: %s to %s",
            my_macsec_param,
            secure_client_socket.getpeername()[0],
        )
        secchan.send_message(json.dumps(my_macsec_param))
        client_macsec_param = json.loads(macsec_param_q.get())
        return secchan, client_macsec_param

    def setup_macsec(self, secure_client_socket, client_mac):
        # Setup macsec
        # Compute macsec parameters
        bytes_for_my_key = generate_random_bytes()  # bytes for my key
        bytes_for_client_key = generate_random_bytes()  # bytes for client key
        my_port = self.macsec_obj.assign_unique_port(client_mac)

        # Bytes conveted into hex strings so that they can be
        # dumped into json later
        my_macsec_param = {
            "bytes_for_my_key": bytes_for_my_key.hex(),
            "bytes_for_client_key": bytes_for_client_key.hex(),
            "port": my_port,
        }

        # Establish secure channel and exchange bytes for macsec keys and port
        # pylint: disable=unused-variable
        secchan, client_macsec_param = self.setup_secchannel(
            secure_client_socket, my_macsec_param
        )
        # pylint: enable=unused-variable

        # Compute keys by XORing bytes
        my_macsec_key = xor_bytes(
            bytes_for_my_key,
            bytes.fromhex(client_macsec_param["bytes_for_client_key"]),
        ).hex()  # XOR my bytes_for_my_key with client's bytes_for_client_key
        client_macsec_key = xor_bytes(
            bytes_for_client_key,
            bytes.fromhex(client_macsec_param["bytes_for_my_key"]),
        ).hex()  # XOR my bytes_for_client_key with client's bytes_for_my_key

        self.macsec_obj.set_macsec_tx(
            client_mac, my_macsec_key, my_port
        )  # setup macsec tx channel
        self.macsec_obj.set_macsec_rx(
            client_mac, client_macsec_key, client_macsec_param["port"]
        )  # setup macsec rx channel
        self.add_to_batman(client_mac)
        self.macsec_setup_event.set()

    def add_to_batman(self, client_mac):
        # Adds macsec interface to batman
        if self.level == "lower":
            # TODO: change this to add lower macsec interfaces to a bridge
            # before adding to batman (to avoid bridge loops with ebtables)

            # Add lower macsec interface to lower batman interface
            add_interface_to_batman(
                interface_to_add=self.macsec_obj.get_macsec_interface_name(
                    client_mac
                ),
                batman_interface=self.batman_interface,
            )
        elif self.level == "upper":
            add_interface_to_bridge(
                interface_to_add=self.macsec_obj.get_macsec_interface_name(
                    client_mac
                ),
                bridge_interface=self.bridge_interface,
            )
            # setup_ebtables_macsec(
            #     interface=self.macsec_obj.get_macsec_interface_name(client_mac),
            #     mac=client_mac
            # )

    def start(self):
        # ... other starting procedures
        self.sender_thread.start()

    def stop(self):
        # Use this method to stop the periodic sender and other threads
        self.stop_event.set()
        self.sender_thread.join()
