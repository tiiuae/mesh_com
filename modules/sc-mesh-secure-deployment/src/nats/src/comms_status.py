"""
Comms status class
"""
import subprocess
import time
import threading
import copy
import hashlib
from .comms_common import STATUS


class CommsStatus:
    """
    Maintains mesh and radio statuses
    """
    class WpaStatus:
        """
        Maintains wpa_supplicant status
        """
        def __init__(self):
            self.interface = ""
            self.bssid = ""
            self.freq = ""
            self.ssid = ""
            self.id = ""
            self.mode = ""
            self.pairwise_cipher = "UNKNOWN"
            self.group_cipher = "UNKNOWN"
            self.key_mgmt = "UNKNOWN"
            self.wpa_state = "UNKNOWN"
            self.address = ""
            self.uuid = ""

        def reset(self):
            self.interface = ""
            self.bssid = ""
            self.freq = ""
            self.ssid = ""
            self.id = ""
            self.mode = ""
            self.pairwise_cipher = "UNKNOWN"
            self.group_cipher = "UNKNOWN"
            self.key_mgmt = "UNKNOWN"
            self.wpa_state = "INTERFACE_DISABLED"
            self.address = ""
            self.uuid = ""

        def __eq__(self, other):
            if isinstance(other, CommsStatus.WpaStatus):
                return self.interface == other.interface and \
                    self.bssid == other.bssid and \
                    self.freq == other.freq and \
                    self.ssid == other.ssid and \
                    self.id == other.id and \
                    self.pairwise_cipher == other.pairwise_cipher and \
                    self.group_cipher == other.group_cipher and \
                    self.key_mgmt == other.key_mgmt and \
                    self.wpa_state == other.wpa_state and\
                    self.address == other.address and \
                    self.uuid == other.uuid
            return False

    def __init__(self, logger):
        self.__lock = threading.Lock()
        self.__thread_running = False
        self.__logger = logger
        self.__wpa_status = self.WpaStatus()
        self.__old_wpa_status = copy.copy(self.__wpa_status)
        self.__mesh_status = STATUS.no_status
        self.__mesh_cfg_status = STATUS.mesh_default
        self.__is_mission_cfg = False  # True when mission cfg has been applied
        self.__is_radio_on = True  # True since mesh is started via initd
        self.__is_visualisation_active = False
        self.__is_hash_file = False
        # Refresh status
        self.__update_status()

    @property
    def mesh_status(self):
        return self.__mesh_status

    @property
    def mesh_cfg_status(self):
        return self.__mesh_cfg_status

    @mesh_cfg_status.setter
    def mesh_cfg_status(self, status: STATUS):
        if status is STATUS.mesh_cfg_stored \
                or status is STATUS.mesh_cfg_not_stored:
            self.__mesh_cfg_status = status

    @property
    def is_mission_cfg(self):
        return self.__is_mission_cfg

    @property
    def is_radio_on(self):
        return self.__is_radio_on

    @is_radio_on.setter
    def is_radio_on(self, value: bool):
        self.__is_radio_on = value

    @property
    def is_visualisation_active(self):
        return self.__is_visualisation_active

    @is_visualisation_active.setter
    def is_visualisation_active(self, value: bool):
        self.__is_visualisation_active = value

    def refresh_status(self):
        """
        comms status

        Returns:
            str: OK
            str: Additional info related to command
        """
        self.__update_status()

        return "OK", "Current status read"

    def __update_status(self):
        """
        Get mesh configuration status and wpa_supplicant
        status and update internal states accordingly.
        """
        # Lock thread
        self.__lock.acquire()

        # Check whether mission config exists
        try:
            self.__get_mission_cfg_status()
        except RuntimeError as error:
            # Do not change mission config status in case of read error
            self.__logger.debug(error)

        # Get wpa_supplicant status
        try:
            if self.__get_wpa_supplicant_pid():
                self.__get_wpa_cli_status()
            else:
                # wpa_supplicant not running => reset
                self.__wpa_status.reset()
        except RuntimeError as error:
            self.__logger.debug(error)
            self.__wpa_status.reset()

        # Update internal statuses
        if self.__wpa_status.wpa_state == "COMPLETED":
            if self.__is_mission_cfg:
                self.__mesh_status = STATUS.mesh_mission_connected
            else:
                if self.__mesh_cfg_status == STATUS.mesh_cfg_stored:
                    # In case has file exists it means that mission
                    # setting has been applied at least once after revoke.
                    if self.__is_hash_file:
                        self.__mesh_status = STATUS.mesh_old_mission_connected
                    else:
                        self.__mesh_status = STATUS.mesh_default
                else:
                    self.__mesh_status = STATUS.mesh_default
            self.__is_radio_on = True
        else:  # SCANNING, DISCONNECTED...
            if self.__is_mission_cfg:
                self.__mesh_status = STATUS.mesh_mission_not_connected
            else:
                self.__mesh_status = STATUS.no_status
            if self.__wpa_status.wpa_state == "INTERFACE_DISABLED":
                self.__is_radio_on = False
            else:
                self.__is_radio_on = True
        # Unlock thread
        self.__lock.release()

    def __get_wpa_supplicant_pid(self) -> str:
        """
        Get wpa_supplicant process ID.
        """
        # Run commands in pieces
        ps_command = ["ps", "ax"]
        grep_command = ["grep", "-E", r"wpa_supplicant\-11s"]
        awk_command = ["awk", '{print $1}']

        ps_process = subprocess.Popen(ps_command, stdout=subprocess.PIPE)
        grep_process = subprocess.Popen(grep_command, stdin=ps_process.stdout,
                                        stdout=subprocess.PIPE)
        awk_process = subprocess.Popen(awk_command, stdin=grep_process.stdout,
                                       stdout=subprocess.PIPE)

        # Wait for completion and store the output
        output, error = awk_process.communicate()
        if error:
            raise RuntimeError("Error getting wpa_supplicant PID: {}".format(error))
        return output.decode()

    def __get_wpa_cli_status(self):
        """
        Get wpa_supplicant states via wpa_cli tool.
        """
        wpa_cli_command = ["wpa_cli", "status"]
        proc = subprocess.Popen(wpa_cli_command,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        output, error = proc.communicate()
        if error:
            raise RuntimeError("Error from wpa_cli: {}".format(error))
        output_lines = output.decode().split("\n")
        for line in output_lines:
            if "Selected interface " in line:
                interface = line.split("Selected interface ")[1].strip()
                self.__wpa_status.interface = interface.strip("\'")
            elif "bssid=" in line:
                self.__wpa_status.bssid = line.split("bssid=")[1].strip()
            elif "freq=" in line:
                self.__wpa_status.freq = line.split("freq=")[1].strip()
            elif "ssid=" in line:
                self.__wpa_status.ssid = line.split("ssid=")[1].strip()
            elif "id=" in line and "uuid" not in line:
                self.__wpa_status.id = line.split("id=")[1].strip()
            elif "mode=" in line:
                self.__wpa_status.mode = line.split("mode=")[1].strip()
            elif "pairwise_cipher=" in line:
                self.__wpa_status.pairwise_cipher = line.split("pairwise_cipher=")[1].strip()
            elif "group_cipher=" in line:
                self.__wpa_status.group_cipher = line.split("group_cipher=")[1].strip()
            elif "key_mgmt=" in line:
                self.__wpa_status.key_mgmt = line.split("key_mgmt=")[1].strip()
            elif "wpa_state=" in line:
                self.__wpa_status.wpa_state = line.split("wpa_state=")[1].strip()
            elif "address=" in line:
                self.__wpa_status.address = line.split("address=")[1].strip()
            elif "uuid=" in line:
                self.__wpa_status.uuid = line.split("uuid=")[1].strip()

        if self.__wpa_status != self.__old_wpa_status:
            # Update copy of wpa_status
            self.__old_wpa_status = copy.copy(self.__wpa_status)
            # Debug
            self.__logger.debug("interface=%s", self.__wpa_status.interface)
            self.__logger.debug("bssid=%s", self.__wpa_status.bssid)
            self.__logger.debug("freq=%s", self.__wpa_status.freq)
            self.__logger.debug("ssid=%s", self.__wpa_status.ssid)
            self.__logger.debug("id=%s", self.__wpa_status.id)
            self.__logger.debug("mode=%s", self.__wpa_status.mode)
            self.__logger.debug("pairwise_cipher=%s", self.__wpa_status.pairwise_cipher)
            self.__logger.debug("group_cipher=%s", self.__wpa_status.group_cipher)
            self.__logger.debug("key_mgmt=%s", self.__wpa_status.key_mgmt)
            self.__logger.debug("wpa_state=%s", self.__wpa_status.wpa_state)
            self.__logger.debug("address=%s", self.__wpa_status.address)
            self.__logger.debug("uuid=%s", self.__wpa_status.uuid)

    def __get_mission_cfg_status(self):
        """
        Checks whether mission config file exists and does it
        match with hash file of previously applied settings.
        """
        config_file_path = "/opt/mesh.conf"
        hash_file_path = "/opt/mesh.conf_hash"
        old_mesh_cfg_status = self.__mesh_cfg_status
        old_is_mission_cfg = self.__is_mission_cfg
        try:
            with open(config_file_path, "rb") as f:
                self.__mesh_cfg_status = STATUS.mesh_cfg_stored
                config = f.read()
                hash_obj = hashlib.sha256(config)
                hash_hex = hash_obj.hexdigest()
            try:
                with open(hash_file_path, 'r') as f:
                    data = f.read()
                    self.__is_hash_file = True
                    if hash_hex == data:
                        self.__logger.debug("Config hash matches")
                        self.__is_mission_cfg = True
                    else:
                        self.__logger.debug("Config hash doesn't match")
                        self.__is_mission_cfg = False
            except (FileNotFoundError, ValueError):
                self.__is_mission_cfg = False
                self.__is_hash_file = False
        except (FileNotFoundError, ValueError):
            # No config file or not able to read it
            self.__mesh_cfg_status = STATUS.mesh_cfg_not_stored
            self.__is_mission_cfg = False
        # Debug only changes
        if old_mesh_cfg_status != self.__mesh_cfg_status or \
                old_is_mission_cfg != self.__is_mission_cfg:
            self.__logger.debug("mesh_cfg_status=%s, is_mission_cfg=%s, ",
                                self.__mesh_cfg_status, self.__is_mission_cfg)
