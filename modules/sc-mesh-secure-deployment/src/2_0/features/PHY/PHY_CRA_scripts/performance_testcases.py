import threading
import time
import psutil
from SP_CRA_v7 import PHYCRA  # SP_CRA_v7 is the main script/implementation

class PerformanceTest:
    def __init__(self, num_clients):
        self.num_clients = num_clients
        self.phycra_instance = PHYCRA()
        self.client_threads = []
        self.start_time = None
        self.end_time = None
        self.cpu_usage = []
        self.memory_usage = []
        self.network_usage_start = psutil.net_io_counters()
        self.network_usage_end = None

    def run_test(self):
        self.start_time = time.time()
        for _ in range(self.num_clients):
            client_thread = threading.Thread(target=self.simulate_client)
            self.client_threads.append(client_thread)
            client_thread.start()

        # Monitor system resources in a separate thread
        monitor_thread = threading.Thread(target=self.monitor_resources)
        monitor_thread.start()

        for thread in self.client_threads:
            thread.join()

        self.network_usage_end = psutil.net_io_counters()
        self.end_time = time.time()
        monitor_thread.join()
        self.phycra_instance.stop_event.set()
        self.phycra_instance.server_thread.join()
        self.phycra_instance.listen_thread.join()
        self.phycra_instance.broadcast_thread.join()
        self.display_results()

    def simulate_client(self):
        self.phycra_instance.connect_to_server(self.phycra_instance.SERVER)

    def monitor_resources(self):
        while any(thread.is_alive() for thread in self.client_threads):
            self.cpu_usage.append(psutil.cpu_percent(interval=1))
            self.memory_usage.append(psutil.virtual_memory().percent)

    def display_results(self):
        total_time = self.end_time - self.start_time
        avg_cpu_usage = sum(self.cpu_usage) / len(self.cpu_usage)
        avg_memory_usage = sum(self.memory_usage) / len(self.memory_usage)
        sent_bytes = self.network_usage_end.bytes_sent - self.network_usage_start.bytes_sent
        recv_bytes = self.network_usage_end.bytes_recv - self.network_usage_start.bytes_recv
        print(f"Total time for {self.num_clients} clients: {total_time:.2f} seconds")
        print(f"Average time per client: {total_time / self.num_clients:.2f} seconds")
        print(f"Average CPU usage during the test: {avg_cpu_usage:.2f}%")
        print(f"Average memory usage during the test: {avg_memory_usage:.2f}%")
        print(f"Total data sent: {sent_bytes} bytes")
        print(f"Total data received: {recv_bytes} bytes")

if __name__ == "__main__":
    num_clients = 10  # Change this to the number of simulated clients to be tested
    performance_test = PerformanceTest(num_clients)
    performance_test.run_test()

