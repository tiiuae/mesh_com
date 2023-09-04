"""
mesh setting nats node
"""
import json
from shlex import quote
import subprocess

try:
    import comms_common as comms
    import validation
    import comms_status as cs
except ImportError:
    import src.validation as validation
    import src.comms_common as comms
    import src.comms_status as cs


class CommsSettings:  # pylint: disable=too-few-public-methods, too-many-instance-attributes
    """
    Comms settings class
    """

    def __init__(self, comms_status: cs.CommsStatus, logger):
        self.logger = logger
        self.api_version = 1
        self.ssid = ""
        self.key = ""
        self.ap_mac = ""
        self.country = ""
        self.frequency = ""
        self.frequency_mcc = ""
        self.ip_address = ""
        self.subnet = ""
        self.tx_power = ""
        self.mode = ""
        self.routing = ""
        self.priority = ""
        self.role = ""
        self.mesh_vif = ""
        self.phy = ""
        self.msversion = ""
        self.delay = ""    # delay for channel change
        self.comms_status = comms_status
        self.csa_state = 0  # 0: not started, 1: stored, 2: triggered
        self.csa_count = 0  # number of CSA triggered
        self.device_amount = "0"

    def validate_mesh_settings(self) -> (str, str):
        """
        Validate mesh settings
        """
        # pylint: disable=too-many-return-statements
        if validation.validate_ssid(self.ssid) is False:
            return "FAIL", "Invalid SSID"

        if validation.validate_wpa3_psk(self.key) is False:
            return "FAIL", "Invalid WPA3 PSK"

        if validation.validate_ip_address(self.ip_address) is False:
            return "FAIL", "Invalid IP address"

        if validation.validate_mode(self.mode) is False:
            return "FAIL", "Invalid mode"

        if validation.validate_frequency(int(self.frequency)) is False:
            return "FAIL", "Invalid frequency"

        if validation.validate_frequency(int(self.frequency_mcc)) is False:
            return "FAIL", "Invalid mcc frequency"

        if validation.validate_country_code(self.country) is False:
            return "FAIL", "Invalid country code"

        if validation.validate_netmask(self.subnet) is False:
            return "FAIL", "Invalid subnet"

        if validation.validate_tx_power(int(self.tx_power)) is False:
            return "FAIL", "Invalid tx power"

        if validation.validate_routing(self.routing) is False:
            return "FAIL", "Invalid routing algo"

        if validation.validate_priority(self.priority) is False:
            return "FAIL", "Invalid priority"

        if validation.validate_role(self.role) is False:
            return "FAIL", "Invalid role"

        return "OK", "Mesh settings OK"

    def handle_mesh_settings_csa(self, msg: str, path="/opt",
                             file="mesh_stored.conf") -> (str, str, str):
        """
        Handle mesh settings
        """
        try:
            parameters = json.loads(msg)
            if "status" in parameters and self.csa_state == 1:
                self.csa_count = self.csa_count + 1
                if self.csa_count >= int(self.device_amount):
                    self.csa_state = 2
                    self.logger.debug(f"Trigger channel switch to {self.frequency}")
                    return "TRIGGER", "Channel switch triggered", self.delay
                return "COUNT", "Channel switch count", self.delay

            self.csa_state = 0
            ret, info = self.__load_settings()
            self.logger.debug("load settings: %s, %s", ret, info)

            #self.api_version = int(parameters["api_version"])
            self.frequency, self.delay, self.device_amount= map(quote,
                                                                (str(parameters["frequency"]),
                                                                 str(parameters["delay"]),
                                                                 str(parameters["amount"])))

            if validation.validate_delay(self.delay) and validation.validate_frequency(int(self.frequency)):
                ret, info  = "OK", "CSA settings OK"
            else:
                ret, info = "FAIL", "Invalid delay or frequency"

            self.logger.debug(" settings validation: %s, %s", ret, info)
            if ret == "FAIL":
                self.comms_status.mesh_cfg_status = \
                    comms.STATUS.mesh_cfg_not_stored
                self.logger.error("save settings failed: %s, %s", ret, info)
            else:
                ret, info = self.__save_settings(path, file)
                self.logger.debug("save settings: %s, %s", ret, info)
                self.csa_state = 1
                self.csa_count = 0

                return ret, info, self.delay

        except (json.decoder.JSONDecodeError, KeyError,
                TypeError, AttributeError) as error:
            self.comms_status.mesh_cfg_status = \
                comms.STATUS.mesh_cfg_not_stored
            ret, info = "FAIL", "JSON format not correct" + str(error)
            self.logger.error("csa settings validation: %s, %s", ret, info)

        return ret, info, self.delay

    def handle_mesh_settings(self, msg: str, path="/opt",
                             file="mesh_stored.conf") -> (str, str):
        """
        Handle mesh settings
        """
        try:
            parameters = json.loads(msg)
            print(parameters)
            self.msversion = "nats"
            self.api_version = int(parameters["api_version"])
            self.ssid = quote(str(parameters["ssid"]))
            self.key = quote(str(parameters["key"]))
            self.ap_mac = quote(str(parameters["ap_mac"]))
            self.country = quote(str(parameters["country"]).lower())
            self.frequency = quote(str(parameters["frequency"]))
            self.frequency_mcc = quote(str(parameters["frequency_mcc"]))
            self.ip_address = quote(str(parameters["ip"]))
            self.subnet = quote(str(parameters["subnet"]))
            self.tx_power = quote(str(parameters["tx_power"]))
            self.mode = quote(str(parameters["mode"]))
            self.routing = quote(str(parameters["routing"]))
            self.priority = quote(str(parameters["priority"]))
            self.role = quote(str(parameters["role"]))
            # not currently in json mesh_settings
            self.mesh_vif = "wlp1s0"
            # not currently in json mesh_settings
            self.phy = "phy0"

            ret, info = self.validate_mesh_settings()
            self.logger.debug("Mesh settings validation: %s, %s", ret, info)
            if ret == "FAIL":
                self.comms_status.mesh_cfg_status = \
                    comms.STATUS.mesh_cfg_not_stored
                self.logger.error("save settings failed: %s, %s", ret, info)
            else:
                ret, info = self.__save_settings(path, file)
                self.logger.debug("save settings: %s, %s", ret, info)

        except (json.decoder.JSONDecodeError, KeyError,
                TypeError, AttributeError) as error:
            self.comms_status.mesh_cfg_status = \
                comms.STATUS.mesh_cfg_not_stored
            ret, status = "FAIL", self.comms_status.mesh_cfg_status
            info = "JSON format not correct" + str(error)
            self.logger.error("Mesh settings validation: %s, %s", ret, info)

        return ret, info

    def __save_settings(self, path: str, file: str) -> (str, str):
        """
        Save mesh settings
        """
        try:
            with open(f"{path}/{file}", "w", encoding="utf-8") as mesh_conf:
                # not currently in json mesh_settings
                mesh_conf.write(f"MSVERSION={quote(self.msversion)}\n")
                mesh_conf.write(f"MODE={quote(self.mode)}\n")
                mesh_conf.write(f"IP={quote(self.ip_address)}\n")
                mesh_conf.write(f"MASK={quote(self.subnet)}\n")
                mesh_conf.write(f"MAC={quote(self.ap_mac)}\n")
                mesh_conf.write(f"KEY={quote(self.key)}\n")
                mesh_conf.write(f"ESSID={quote(self.ssid)}\n")
                mesh_conf.write(f"FREQ={quote(self.frequency)}\n")
                mesh_conf.write(f"FREQ_MCC={quote(self.frequency_mcc)}\n")
                mesh_conf.write(f"TXPOWER={quote(self.tx_power)}\n")
                mesh_conf.write(f"COUNTRY={quote(self.country).upper()}\n")
                mesh_conf.write(f"ROUTING={quote(self.routing)}\n")
                mesh_conf.write(f"ROLE={quote(self.role)}\n")
                mesh_conf.write(f"PRIORITY={quote(self.priority)}\n")
                # not currently in json mesh_settings
                mesh_conf.write(f"MESH_VIF={quote(self.mesh_vif)}\n")
                # not currently in json mesh_settings
                mesh_conf.write(f"PHY={quote(self.phy)}\n")
        except:
            self.comms_status.mesh_cfg_status = \
                comms.STATUS.mesh_cfg_not_stored
            self.logger.error("not able to write new %s", file)
            return "FAIL", "not able to write new mesh.conf"

        self.comms_status.mesh_cfg_status = comms.STATUS.mesh_cfg_stored
        self.logger.debug("%s written", file)
        return "OK", "Mesh configuration stored"

    def __read_configs(self, mesh_conf_lines):
        import re
        pattern = r'(\w+)=(.*?)(?:\s*#.*)?$'
        # Find all key-value pairs in the text
        matches = re.findall(pattern, mesh_conf_lines, re.MULTILINE)
        for match in matches:
            print(f"{match[0]}={match[1]}")
            if match[0] == "MODE":
                self.mode = match[1]
            elif match[0] == "IP":
                self.ip_address = match[1]
            elif match[0] == "MASK":
                self.subnet = match[1]
            elif match[0] == "MAC":
                self.ap_mac = match[1]
            elif match[0] == "KEY":
                self.key = match[1]
            elif match[0] == "ESSID":
                self.ssid = match[1]
            elif match[0] == "FREQ":
                self.frequency = match[1]
            elif match[0] == "FREQ_MCC":
                self.frequency_mcc = match[1]
            elif match[0] == "TXPOWER":
                self.tx_power = match[1]
            elif match[0] == "COUNTRY":
                self.country = match[1]
            elif match[0] == "ROUTING":
                self.routing = match[1]
            elif match[0] == "ROLE":
                self.role = match[1]
            elif match[0] == "PRIORITY":
                self.priority = match[1]
            elif match[0] == "MESH_VIF":
                self.mesh_vif = match[1]
            elif match[0] == "PHY":
                self.phy = match[1]
            elif match[0] == "MSVERSION":
                self.msversion = match[1]
            else:
                self.logger.debug("unknown config parameter: %s", match[0])

    def __load_settings(self) -> (str, str):
        """
        Load mesh settings
        return: OK, FAIL
        """
        config_file_path = "/opt/mesh_default.conf"
        mission_config_file_path = "/opt/mesh.conf"

        try:
            with open(mission_config_file_path, "r", encoding="utf-8") as mesh_conf:
                mesh_conf_lines = mesh_conf.read()
                self.__read_configs(mesh_conf_lines)
        except FileNotFoundError:
            try:
                with open(config_file_path, "r", encoding="utf-8") as mesh_conf:
                    mesh_conf_lines = mesh_conf.read()
                    self.__read_configs(mesh_conf_lines)
            except FileNotFoundError:
                self.logger.error("not able to read mesh config files")
                return "FAIL", "not able to read mesh config files"
        return "OK", "Mesh configuration loaded"
