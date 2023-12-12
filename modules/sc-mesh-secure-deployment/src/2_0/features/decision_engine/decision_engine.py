import sys
import os

path_to_decision_engine = os.path.dirname(__file__) # Path to dir containing this script
sys.path.insert(0, path_to_decision_engine)
sys.path.append(f'{path_to_decision_engine}/..')

from .quarantine import Quarantine
import threading
import pandas as pd
from .mba import MBA
from cbma.tools.custom_logger import CustomLogger
logger = CustomLogger("decision_engine").get_logger()

blacklist_lock = threading.Lock()
class DecisionEngine:
    def __init__(self, mba_multicast_group, mba_port, mba_interface, my_cert_dir, peer_cert_dir, quarantine_period):
        self.blocked_ips = set()
        self.quarantine_period = quarantine_period
        self.quarantine_timer_threads = []
        self.stop_event = threading.Event()
        self.mba = MBA(decision_engine=self, multicast_group=mba_multicast_group, port=mba_port, interface=mba_interface, my_cert_dir=my_cert_dir, peer_cert_dir=peer_cert_dir,  stop_event=self.stop_event)
        self.mba_receiver_thread = threading.Thread(target=self.mba.receive_mba, daemon=True)
        self.mba_receiver_thread.start()

    def update(self, report):
        logger.info(f"DecisionEngine received data: {report}")

        if (report.get("feature") == "IDS" and report.get("malicious")) or (report.get("feature") == "PHY" and report.get("result") == "fail") or (report.get("feature") == "RSS" and report.get("result") == "fail"):
            ip = report.get("ip")
            mac = report.get("ip")
            #self.block_ip(ip)
            self.respond_to_malicious_peer(ip)

        elif report.get("module") == "MBA":
            mac = report.get("mac")
            ip = report.get("ip")
            # Placeholder to respond to MBA

        elif report.get("feature") == "JammingDetection" and report.get("jamming_detected"):
            frequency = report.get("frequency")
            quality = report.get("quality")
            self.respond_to_jamming(frequency, quality)

    def respond_to_malicious_peer(self, ip):
        """
        Block ip and send malicious behaviour announcements
        """
        if ip not in self.blocked_ips:
            self.blocked_ips.add(ip)
            mal_peer = Quarantine(mal_id="dummy", mal_ip=ip, quarantine_period=self.quarantine_period,
                                  blacklist_lock=blacklist_lock, blacklist_dir='./blacklist',
                                  blacklist_filename='blacklist.csv', blocked_ips=self.blocked_ips)
            # Block ip
            quarantine_timer_thread = mal_peer.start_quarantine()
            self.quarantine_timer_threads.append(quarantine_timer_thread)

            # Send MBA
            logger.info(f"Sending mba for IP: {ip}")
            self.mba.send_mba(mac="Test mac", ip=ip)

        else:
            logger.info(f"IP {ip} is already blocked.")


    def stop(self):
        self.stop_event.set()
        for thread in self.quarantine_timer_threads:
            thread.join()

    def respond_to_jamming(self, frequency, quality):
        logger.info(f"Responding to jamming at {frequency} GHz with {quality} quality.")
