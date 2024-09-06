import os
import subprocess
import time
import logging

class MultiRadioManager:
    def __init__(self):
        self.base_path = "/var/run/wpa_supplicant"

    def execute_command(self, command):
        try:
            result = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            return result.stdout.decode('utf-8').strip()
        except subprocess.CalledProcessError as e:
            logging.error(f"Command failed with error: {e.stderr.decode('utf-8').strip()}")
            return None

    def stop_mesh(self, interface_name, radio_index):
        path = f"{self.base_path}_id{radio_index}"
        command = f"wpa_cli -i {interface_name} -p {path} disable_network 0"
        output = self.execute_command(command)
        if output is not None:
            logging.info(f"Mesh network on {interface_name} stopped successfully.")
        else:
            logging.warning(f"Failed to stop mesh network on {interface_name}.")

    def start_mesh(self, interface_name, radio_index):
        path = f"{self.base_path}_id{radio_index}"
        command = f"wpa_cli -i {interface_name} -p {path} enable_network 0"
        output = self.execute_command(command)
        if output is not None:
            logging.info(f"Mesh network on {interface_name} started successfully.")
        else:
            logging.warning(f"Failed to start mesh network on {interface_name}.")

    def get_mesh_status(self, interface_name, radio_index):
        path = f"{self.base_path}_id{radio_index}"
        command = f"wpa_cli -i {interface_name} -p {path} status"
        output = self.execute_command(command)
        if output is None:
            logging.warning(f"Failed to retrieve mesh status for {interface_name}.")
            return None

        status_dict = {}
        for line in output.splitlines():
            key_value = line.split('=', 1)
            if len(key_value) == 2:
                key, value = key_value
                status_dict[key] = value

        return status_dict

    def is_mesh_mode_enabled(self, interface_name, radio_index):
        status = self.get_mesh_status(interface_name, radio_index)
        if status and status.get("mode") == "mesh":
            return True
        return False

    def is_interface_up(self, interface_name):
        """Check if a network interface is up."""
        if not interface_name:
            return False
        return os.path.exists(f"/sys/class/net/{interface_name}")

    def wait_for_interface(self, interface_name, timeout_ms=120000):
        """Wait for a network interface to be up, with a timeout."""
        while timeout_ms > 0:
            if self.is_interface_up(interface_name):
                return True
            time.sleep(1)
            timeout_ms -= 1000
        return False
