import subprocess
import time

class MultiRadioManager:
    def __init__(self):
        self.base_path = "/var/run/wpa_supplicant"

    def execute_command(self, command):
        """
        Executes a given shell command and returns the output.
        """
        try:
            result = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            return result.stdout.decode('utf-8').strip()
        except subprocess.CalledProcessError as e:
            print(f"Command failed with error: {e.stderr.decode('utf-8').strip()}")
            return None

    def stop_mesh(self, interface_name, radio_index):
        """
        Stops the mesh network on the specified radio interface.
        
        :param interface_name: The name of the radio interface (e.g., "halow1").
        :param radio_index: The index of the radio (e.g., 0).
        """
        path = f"{self.base_path}_id{radio_index}"
        command = f"wpa_cli -i {interface_name} -p {path} disable_network 0"
        output = self.execute_command(command)
        if output is not None:
            print(f"Mesh network on {interface_name} stopped successfully.")
        else:
            print(f"Failed to stop mesh network on {interface_name}.")

    def start_mesh(self, interface_name, radio_index):
        """
        Starts the mesh network on the specified radio interface.
        
        :param interface_name: The name of the radio interface (e.g., "halow1").
        :param radio_index: The index of the radio (e.g., 0).
        """
        path = f"{self.base_path}_id{radio_index}"
        command = f"wpa_cli -i {interface_name} -p {path} enable_network 0"
        output = self.execute_command(command)
        if output is not None:
            print(f"Mesh network on {interface_name} started successfully.")
        else:
            print(f"Failed to start mesh network on {interface_name}.")

    def get_mesh_status(self, interface_name, radio_index):
        """
        Retrieves the status of the mesh network on the specified radio interface.
        
        :param interface_name: The name of the radio interface (e.g., "halow1").
        :param radio_index: The index of the radio (e.g., 0).
        :return: A dictionary containing the mesh status information or None if the command fails.
        """
        path = f"{self.base_path}_id{radio_index}"
        command = f"wpa_cli -i {interface_name} -p {path} status"
        output = self.execute_command(command)
        if output is None:
            print(f"Failed to retrieve mesh status for {interface_name}.")
            return None

        # Parse the output into a dictionary
        status_dict = {}
        for line in output.splitlines():
            key_value = line.split('=', 1)
            if len(key_value) == 2:
                key, value = key_value
                status_dict[key] = value

        return status_dict

    def is_mesh_mode_enabled(self, interface_name, radio_index):
        """
        Checks if the mesh mode is enabled on the specified radio interface.
        
        :param interface_name: The name of the radio interface (e.g., "halow1").
        :param radio_index: The index of the radio (e.g., 0).
        :return: True if mesh mode is enabled, False otherwise.
        """
        status = self.get_mesh_status(interface_name, radio_index)
        if status and status.get("mode") == "mesh":
            return True
        return False

# Example usage
mr_manager = MultiRadioManager()
IFACE="halow1"
MESH_INDEX=0
mesh_enabled = mr_manager.is_mesh_mode_enabled(IFACE, MESH_INDEX)
if mesh_enabled:
    print("Mesh enabled already. Getting the status of the mesh")
    status = mr_manager.get_mesh_status(IFACE, MESH_INDEX)
    print("Mesh Status:")
    for key, value in status.items():
        print(f"{key}: {value}")
    time.sleep(5)
    print("Stopping the mesh")
    mr_manager.stop_mesh(IFACE, MESH_INDEX)
else:
    print("Mesh disabled. Starting the mesh ")
    mr_manager.start_mesh(IFACE, MESH_INDEX)
    time.sleep(5)
    status = mr_manager.get_mesh_status(IFACE, MESH_INDEX)
    print("Mesh Status:")
    for key, value in status.items():
        print(f"{key}: {value}")
    time.sleep(5)
    print("Stopping the mesh")
    mr_manager.stop_mesh(IFACE, MESH_INDEX)

