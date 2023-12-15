import sys
import os
import threading

path_to_decision_engine = os.path.dirname(__file__) # Path to dir containing this script
sys.path.insert(0, path_to_decision_engine)
sys.path.append(f'{path_to_decision_engine}/..')

from quarantine import Quarantine
from mba import MBA
from cbma.tools.custom_logger import CustomLogger
logger = CustomLogger("decision_engine").get_logger()

blacklist_lock = threading.Lock()
class DecisionEngine:
    """
    A class representing the Decision Engine responsible for handling reports and responding to notifications from security features.

    Attributes:
    blocked_ips (set): A set of IP addresses that are currently blocked.
    quarantine_period (int): Duration for which an IP is quarantined.
    quarantine_timer_threads (list): A list of threads running quarantine timers.
    stop_event (threading.Event): An event to signal the stopping of the engine.
    mba (MBA): An instance of MBA for sending and receiving malicious behavior announcements.
    mba_receiver_thread (threading.Thread): Thread for receiving MBA messages.

    Methods:
    update(report): Processes incoming reports and takes appropriate actions.
    respond_to_malicious_peer(ip, mac): Responds to a report of a malicious peer.
    stop(): Stops the decision engine and all its components.
    """
    def __init__(self, mba_multicast_group, mba_port, mba_interface, my_cert_dir, peer_cert_dir, quarantine_period):
        """
        Initializes the Decision Engine with the necessary configuration for multicast communication and quarantine.

        Parameters:
        mba_multicast_group (str): The multicast group address for MBA communication.
        mba_port (int): The port number for MBA communication.
        mba_interface (str): The network interface used for MBA communication.
        my_cert_dir (str): Directory containing the machine's own certificates.
        peer_cert_dir (str): Directory containing peer certificates.
        quarantine_period (int): The duration in seconds for quarantining malicious peers.
        """
        self.blocked_ips = set()
        self.quarantine_period = quarantine_period
        self.quarantine_timer_threads = []
        self.stop_event = threading.Event()
        self.mba = MBA(decision_engine=self, multicast_group=mba_multicast_group, port=mba_port, interface=mba_interface, my_cert_dir=my_cert_dir, peer_cert_dir=peer_cert_dir,  stop_event=self.stop_event)
        self.mba_receiver_thread = threading.Thread(target=self.mba.receive_mba, daemon=True)
        self.mba_receiver_thread.start()

    def update(self, report):
        """
        Processes an incoming report and takes appropriate action based on the report content.

        Parameters:
        report (dict): A dictionary containing the report data with keys like 'feature', 'malicious', 'result', etc.
        """
        logger.info(f"DecisionEngine received data: {report}")

        if (report.get("feature") == "IDS" and report.get("malicious")) or (report.get("feature") == "PHY" and report.get("result") == "fail") or (report.get("feature") == "RSS" and report.get("result") == "fail"):
            ip = report.get("ip")
            mac = report.get("mac")
            self.respond_to_malicious_peer(ip, mac)

        elif report.get("module") == "MBA":
            ip = report.get("ip")
            mac = report.get("mac")
            # Placeholder to respond to MBA

    def respond_to_malicious_peer(self, ip, mac):
        """
        Initiates a response to a malicious peer by starting their quarantine and sending a malicious behavior announcement.

        Calls start_quarantine() method of Quarantine class that starts a timer for quaranntine_period and automatically calls end_quarantine at the end of the timer

        Parameters:
        ip (str): The IP address of the malicious peer.
        mac (str): The MAC address of the malicious peer.
        """
        if ip not in self.blocked_ips:
            self.blocked_ips.add(ip)
            mal_peer = Quarantine(mal_mac=mac, mal_ip=ip, quarantine_period=self.quarantine_period,
                                  blacklist_lock=blacklist_lock, blacklist_dir='./blacklist',
                                  blacklist_filename='blacklist.csv', blocked_ips=self.blocked_ips)
            # Start quarantine
            quarantine_timer_thread = mal_peer.start_quarantine()
            self.quarantine_timer_threads.append(quarantine_timer_thread)

            # Send MBA
            logger.info(f"Sending mba for IP: {ip}, MAC: {mac}")
            self.mba.send_mba(mac=mac, ip=ip)

        else:
            logger.info(f"IP {ip} is already in quarantine.")


    def stop(self):
        """
        Stops the decision engine by setting the stop event and joining all quarantine timer threads.
        """
        self.stop_event.set()
        for thread in self.quarantine_timer_threads:
            thread.join()
        self.mba_receiver_thread.join()
