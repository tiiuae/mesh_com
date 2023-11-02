import os
import sys
path_to_cbma_dir = os.path.dirname(__file__) # Path to dir containing this script
sys.path.insert(0, path_to_cbma_dir)

from auth.authServer import AuthServer
from auth.authClient import AuthClient
import threading
from multicast.multicast import MulticastHandler
from tools.monitoring_wpa import *
from tools.utils import *
from tools.custom_logger import CustomLogger
from macsec import macsec
import queue
import random
import json
from secure_channel.secchannel import SecMessageHandler
BEACON_TIME = 10
MAX_CONSECUTIVE_NOT_RECEIVED = 2
MULTICAST_ADDRESS = 'ff02::1'
TIMEOUT = 3 * BEACON_TIME
BRIDGE = False # TODO: During migration to mesh_com, get it from /opt/mesh.conf
logger_instance = CustomLogger("mutAuth")

class mutAuth():
    def __init__(self, in_queue, level, meshiface, port, batman_interface, shutdown_event, batman_setup_event):
        self.level = level
        self.meshiface = meshiface
        self.mymac = get_mac_addr(self.meshiface)
        self.ipAddress = mac_to_ipv6(self.mymac)
        self.port = port
        self.CERT_PATH = f'{path_to_cbma_dir}/cert_generation/certificates'  # Change this to the actual path of your certificates
        self.wpa_supplicant_ctrl_path = f"/var/run/wpa_supplicant/{self.meshiface}"
        self.in_queue = in_queue
        self.logger = logger_instance.get_logger()
        self.multicast_handler = MulticastHandler(self.in_queue, MULTICAST_ADDRESS, self.port, self.meshiface)
        self.stop_event = threading.Event()
        self.sender_thread = threading.Thread(target=self._periodic_sender, args=(self.stop_event,))
        self.shutdown_event = shutdown_event  # Add this to handle graceful shutdown
        if self.level == "lower":
            self.macsec_obj = macsec.Macsec(level=self.level, interface=self.meshiface, macsec_encryption="off")  # Initialize lower macsec object
        elif self.level == "upper":
            self.macsec_obj = macsec.Macsec(level=self.level, interface=self.meshiface, macsec_encryption="on")  # Initialize upper macsec object
        self.connected_peers_status = {} # key = client mac address, value = [status : ("ongoing", "authenticated", "not connected"), no of failed attempts]}
        self.connected_peers_status_lock = threading.Lock()
        self.maximum_num_failed_attempts = 3 # Maximum number of failed attempts for mutual authentication (can be changed)
        self.batman_interface = batman_interface
        self.batman_setup_event = batman_setup_event

    def check_mesh(self):
        if not is_wpa_supplicant_running():
            logger.info("wpa_supplicant process is not running.")
            run_wpa_supplicant(self.meshiface)
            set_ipv6(self.meshiface, self.ipAddress)
        else:
            logger.info("wpa_supplicant process is running.")

    def _periodic_sender(self, stop_event):
        while not stop_event.is_set() and not self.shutdown_event.is_set():
            self.multicast_handler.send_multicast_message(self.mymac)
            time.sleep(BEACON_TIME)

    def monitor_wpa_multicast(self):
        #muthread = threading.Thread(target=self.multicast_handler.receive_multicast)
        #muthread.start()

        while not self.shutdown_event.is_set():
            source, message = self.in_queue.get()
            if source == "WPA":
                self.logger.info("External node_connect event triggered!")
                self.logger.info(f"Received MAC from WPA event: {message}")
                handle_peer_connected_thread = threading.Thread(target=self.handle_wpa_multicast_event, args=(message,))
                handle_peer_connected_thread.start()
            elif source == "MULTICAST":
                self.logger.info(f"Received MAC on multicast: {message} at interface {self.meshiface}")
                handle_peer_connected_thread = threading.Thread(target=self.handle_wpa_multicast_event, args=(message,))
                handle_peer_connected_thread.start()

    def handle_wpa_multicast_event(self, mac):
        if mac not in self.connected_peers_status:
            # There is no ongoing connection with peer yet
            # Wait for random seconds
            random_wait = random.uniform(0.5,3)  # Wait between 0.5 to 3 seconds. Random waiting to avoid race condition
            time.sleep(random_wait)
            if mac not in self.connected_peers_status:
                # Start as client
                print("------------------client ---------------------")
                with self.connected_peers_status_lock:
                    self.connected_peers_status[mac] = ["ongoing", 0] # Update status as ongoing, num of failed attempts = 0
                self.start_auth_client(mac)
        elif self.connected_peers_status[mac][0] not in ["ongoing", "authenticated"]:
            # If node does not have ongoing authentication or is not already authenticated or has not been blacklisted
            # Wait for random seconds
            random_wait = random.uniform(0.5,3)  # Wait between 0.5 to 3 seconds. Random waiting to avoid race condition
            time.sleep(random_wait)
            if self.connected_peers_status[mac][0] not in ["ongoing", "authenticated"]:
                # Start as client
                print("------------------client ---------------------")
                with self.connected_peers_status_lock:
                    self.connected_peers_status[mac][0] = "ongoing"  # Update status as ongoing, num of failed attempts = same as before
                self.start_auth_client(mac)

    def start_auth_server(self):
        auth_server = AuthServer(self.meshiface, self.ipAddress, self.port, self.CERT_PATH, self)
        auth_server_thread = threading.Thread(target=auth_server.start_server)
        auth_server_thread.start()
        return auth_server_thread, auth_server

    def start_auth_client(self, server_mac):
        cli = AuthClient(self.meshiface, server_mac, self.port, self.CERT_PATH, self)
        cli.establish_connection()

    def auth_pass(self, secure_client_socket, client_mac):
        # Steps to execute if auth passes
        with self.connected_peers_status_lock:
            self.connected_peers_status[client_mac][0] = "authenticated"  # Update status as authenticated, num of failed attempts = same as before
        self.setup_macsec(secure_client_socket=secure_client_socket, client_mac=client_mac)

    def auth_fail(self, client_mac):
        # Steps to execute if auth fails
        with self.connected_peers_status_lock:
            self.connected_peers_status[client_mac][1] = self.connected_peers_status[client_mac][1] + 1  # Increment number of failed attempt by 1
            self.connected_peers_status[client_mac][0] = "not connected"  # Update status as not connected
    def batman(self, batman_interface):
        try:
            batman_exec(batman_interface,"batman-adv")
        except Exception as e:
            logger.error(f'Error setting up bat0: {e}')
            sys.exit(1)

    @staticmethod
    def setup_secchannel(secure_client_socket, my_macsec_param):
        # Establish secure channel and exchange macsec key
        secchan = SecMessageHandler(secure_client_socket)
        macsec_param_q = queue.Queue()  # queue to store macsec parameters: macsec_key, port from client_secchan.receive_message
        receiver_thread = threading.Thread(target=secchan.receive_message, args=(macsec_param_q,))
        receiver_thread.start()
        print(f"Sending my macsec parameters: {my_macsec_param} to {secure_client_socket.getpeername()[0]}")
        secchan.send_message(json.dumps(my_macsec_param))
        client_macsec_param = json.loads(macsec_param_q.get())
        return secchan, client_macsec_param

    def setup_macsec(self, secure_client_socket, client_mac):
        # Setup macsec
        # Compute macsec parameters
        bytes_for_my_key = generate_random_bytes() # bytes for my key
        bytes_for_client_key = generate_random_bytes() # bytes for client key
        my_port = self.macsec_obj.assign_unique_port(client_mac)
        my_macsec_param = {'bytes_for_my_key': bytes_for_my_key.hex(), 'bytes_for_client_key': bytes_for_client_key.hex(), 'port': my_port} # Bytes conveted into hex strings so that they can be dumped into json later

        # Establish secure channel and exchange bytes for macsec keys and port
        secchan, client_macsec_param = self.setup_secchannel(secure_client_socket, my_macsec_param)

        # Compute keys by XORing bytes
        my_macsec_key = xor_bytes(bytes_for_my_key, bytes.fromhex(client_macsec_param['bytes_for_client_key'])).hex() # XOR my bytes_for_my_key with client's bytes_for_client_key
        client_macsec_key = xor_bytes(bytes_for_client_key, bytes.fromhex(client_macsec_param['bytes_for_my_key'])).hex() # XOR my bytes_for_client_key with client's bytes_for_my_key

        self.macsec_obj.set_macsec_tx(client_mac, my_macsec_key, my_port) # setup macsec tx channel
        self.macsec_obj.set_macsec_rx(client_mac, client_macsec_key, client_macsec_param['port'])  # setup macsec rx channel
        self.setup_batman(client_mac)

    def setup_batman(self, client_mac):
        if self.level == "lower":
            # Add lower macsec interface to lower batman interface
            add_interface_to_batman(interface_to_add=self.macsec_obj.get_macsec_interface_name(client_mac), batman_interface=self.batman_interface)
        elif self.level == "upper":
            bridge_interface = "br-upper"
            if not is_interface_up(bridge_interface):
                subprocess.run(["brctl", "addbr", bridge_interface], check=True)
                add_interface_to_bridge(interface_to_add=self.macsec_obj.get_macsec_interface_name(client_mac), bridge_interface=bridge_interface)
                subprocess.run(["ip", "link", "set", bridge_interface, "up"], check=True)
            else:
                add_interface_to_bridge(interface_to_add=self.macsec_obj.get_macsec_interface_name(client_mac), bridge_interface=bridge_interface)
            add_interface_to_batman(interface_to_add=bridge_interface, batman_interface=self.batman_interface)
        if not is_interface_up(self.batman_interface): # Turn batman interface up if not up already
            self.batman(self.batman_interface)
            if self.level == "upper" and BRIDGE:
                # Add bat1 and ethernet interface to br-lan to connect external devices
                bridge_interface = "br-lan"
                if not is_interface_up(bridge_interface):
                    self.setup_bridge_over_batman(bridge_interface)
        if not self.batman_setup_event.is_set():
            # Set batman setup event to signal that batman has been setup
            # Used to trigger mtls for upper macsec
            self.batman_setup_event.set()

    def setup_bridge_over_batman(self, bridge_interface):
        # TODO: Need to make it compatible with bridge_settings in mesh_com (This is just for quick test)
        subprocess.run(["brctl", "addbr", bridge_interface], check=True)
        add_interface_to_bridge(interface_to_add=self.batman_interface, bridge_interface=bridge_interface) # Add bat1 to br-lan
        add_interface_to_bridge(interface_to_add="eth1", bridge_interface=bridge_interface)  # Add eth1 to br-lan
        logger.info(f"Setting mac address of {bridge_interface} to be same as {self.batman_interface}..")
        subprocess.run(["ip", "link", "set", "dev", bridge_interface, "address", get_mac_addr(self.batman_interface)], check=True)
        subprocess.run(["ip", "link", "set", bridge_interface, "up"], check=True)
        subprocess.run(["ifconfig", bridge_interface], check=True)



    def start(self):
        # ... other starting procedures
        self.sender_thread.start()

    def stop(self):
        # Use this method to stop the periodic sender and other threads
        self.stop_event.set()
        self.sender_thread.join()