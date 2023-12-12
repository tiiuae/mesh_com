import random
import threading
import time
from ..decision_engine.observable_module import ObservableModule

class IDS(ObservableModule):
    """
    Sample class for IDS
    """
    def __init__(self, decision_engine):
        super().__init__(decision_engine)
        self.stop_event = threading.Event()

    def start(self):
        self.monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
        self.monitor_thread.start()

    def stop(self):
        print("stopping IDS")
        self.stop_event.set()
        self.monitor_thread.join()

    def monitor_loop(self):
        while not self.stop_event.is_set():
            time.sleep(5)
            self.check_dpi()
            self.check_mpi()

    def check_dpi(self):
        # Simulated DPI detection logic
        if random.choice([True, False]):
            ip = f"192.168.1.{random.randint(1, 3)}"
            self.notify({"feature": "IDS", "module": "DPI", "malicious": True, "ip": ip})

    def check_mpi(self):
        # Simulated MPI detection logic
        if random.choice([True, False]):
            ip = f"192.168.1.{random.randint(1, 3)}"
            self.notify({"feature": "IDS", "module": "MPI", "malicious": True, "ip": ip})