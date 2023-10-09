"""
mesh setting nats node
"""
import json
from shlex import quote
import subprocess
import logging
import re

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
        self.logger: logging = logger
        self.api_version: int = 1
        self.radio_index = []
        self.ssid = []
        self.key = []
        self.ap_mac = []
        self.country = []
        self.frequency = []
        self.frequency_mcc = []
        self.ip_address = []
        self.subnet = []
        self.tx_power = []
        self.mode = []
        self.routing = []
        self.priority = []
        self.role: str = ""
        self.mesh_vif = []
        self.phy = []
        self.batman_iface = []
        self.bridge = []
        self.msversion: str = ""
        self.delay:str = ""    # delay for channel change
        self.comms_status = comms_status
        self.csa_state: int = 0  # 0: not started, 1: stored, 2: triggered
        self.csa_count: int = 0  # number of CSA triggered
        self.device_amount: str = "0"  # number of devices to trigger CSA from nats message

    def validate_mesh_settings(self, index: int) -> (str, str):
        """
        Validate mesh settings
        """

        self.logger.debug("validate mesh settings")

        # pylint: disable=too-many-return-statements
        if validation.validate_ssid(self.ssid[index]) is False:
            return "FAIL", "Invalid SSID"
        self.logger.debug("validate mesh settings ssid ok")

        if validation.validate_wpa3_psk(self.key[index]) is False:
            return "FAIL", "Invalid WPA3 PSK"
        self.logger.debug("validate mesh settings wpa3 ok")

        if validation.validate_ip_address(self.ip_address[index]) is False:
            return "FAIL", "Invalid IP address"
        self.logger.debug("validate mesh settings ip ok")

        if validation.validate_mode(self.mode[index]) is False:
            return "FAIL", "Invalid mode"
        self.logger.debug("validate mesh settings mode ok")

        if validation.validate_frequency(int(self.frequency[index])) is False:
            return "FAIL", "Invalid frequency"
        self.logger.debug("validate mesh settings freq ok")

        if validation.validate_frequency(int(self.frequency_mcc[index])) is False:
            return "FAIL", "Invalid mcc frequency"
        self.logger.debug("validate mesh settings mcc freq ok")

        if validation.validate_country_code(self.country[index]) is False:
            return "FAIL", "Invalid country code"
        self.logger.debug("validate mesh settings country ok")

        if validation.validate_netmask(self.subnet[index]) is False:
            return "FAIL", "Invalid subnet"
        self.logger.debug("validate mesh settings subnet ok")

        if validation.validate_tx_power(int(self.tx_power[index])) is False:
            return "FAIL", "Invalid tx power"
        self.logger.debug("validate mesh settings tx power ok")

        if validation.validate_routing(self.routing[index]) is False:
            return "FAIL", "Invalid routing algo"
        self.logger.debug("validate mesh settings routing ok")

        if validation.validate_priority(self.priority[index]) is False:
            return "FAIL", "Invalid priority"
        self.logger.debug("validate mesh settings priority ok")

        if validation.validate_role(self.role) is False:
            return "FAIL", "Invalid role"
        self.logger.debug("validate mesh settings role ok")

        if validation.validate_mesh_vif(self.mesh_vif[index]) is False:
            return "FAIL", "Invalid mesh vif"
        self.logger.debug("validate mesh settings mesh vif ok")

        if validation.validate_phy(self.phy[index]) is False:
            return "FAIL", "Invalid phy"
        self.logger.debug("validate mesh settings phy ok")

        if validation.validate_batman_iface(self.batman_iface[index]) is False:
            return "FAIL", "Invalid batman iface"
        self.logger.debug("validate mesh settings batman iface ok")

        return "OK", "Mesh settings OK"

    def handle_mesh_settings_csa(self, msg: str, path="/opt",
                             file="mesh_stored.conf") -> (str, str, str):
        """
        Handle mesh settings
        """
        # TODO: check implementation
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
            parameters_set = json.loads(msg)
            print(parameters_set)
            self.msversion = "nats"
            self.api_version = int(parameters_set["api_version"])
            self.role = quote(str(parameters_set["role"]))

            for parameters in parameters_set["radios"]:
                self.radio_index.append(int(parameters["radio_index"]))
                self.ssid.append(quote(str(parameters["ssid"])))
                self.key.append(quote(str(parameters["key"])))
                self.ap_mac.append(quote(str(parameters["ap_mac"])))
                self.country.append(quote(str(parameters["country"]).lower()))
                self.frequency.append(quote(str(parameters["frequency"])))
                self.frequency_mcc.append(quote(str(parameters["frequency_mcc"])))
                self.ip_address.append(quote(str(parameters["ip"])))
                self.subnet.append(quote(str(parameters["subnet"])))
                self.tx_power.append(quote(str(parameters["tx_power"])))
                self.mode.append(quote(str(parameters["mode"])))
                self.routing.append(quote(str(parameters["routing"])))
                self.priority.append(quote(str(parameters["priority"])))
                self.mesh_vif.append(quote(str(parameters["mesh_vif"])))
                self.phy.append(quote(str(parameters["phy"])))
                self.batman_iface.append(quote(str(parameters["batman_iface"])))

            bridges = parameters_set["bridge"]
            for bridge in bridges:
                for key, values in bridge.items():
                    self.bridge.append(f"{quote(str(key))} {quote(str(values))}")

            ret, info = "FAIL", "Before validation"

            for index in self.radio_index:
                self.logger.debug("Mesh settings validation index: %s", str(index))
                ret, info = self.validate_mesh_settings(index)
                if ret == "FAIL":
                    break

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
                mesh_conf.write(f"ROLE={quote(self.role)}\n")
                mesh_conf.write(f"MSVERSION={quote(self.msversion)}\n")
                for index in self.radio_index:
                    mesh_conf.write(f"id{index}_MODE={quote(self.mode[index])}\n")
                    mesh_conf.write(f"id{index}_IP={quote(self.ip_address[index])}\n")
                    mesh_conf.write(f"id{index}_MASK={quote(self.subnet[index])}\n")
                    mesh_conf.write(f"id{index}_MAC={quote(self.ap_mac[index])}\n")
                    mesh_conf.write(f"id{index}_KEY={quote(self.key[index])}\n")
                    mesh_conf.write(f"id{index}_ESSID={quote(self.ssid[index])}\n")
                    mesh_conf.write(f"id{index}_FREQ={quote(self.frequency[index])}\n")
                    mesh_conf.write(f"id{index}_FREQ_MCC={quote(self.frequency_mcc[index])}\n")
                    mesh_conf.write(f"id{index}_TXPOWER={quote(self.tx_power[index])}\n")
                    mesh_conf.write(f"id{index}_COUNTRY={quote(self.country[index]).upper()}\n")
                    mesh_conf.write(f"id{index}_ROUTING={quote(self.routing[index])}\n")
                    mesh_conf.write(f"id{index}_PRIORITY={quote(self.priority[index])}\n")
                    mesh_conf.write(f"id{index}_MESH_VIF={quote(self.mesh_vif[index])}\n")
                    mesh_conf.write(f"id{index}_PHY={quote(self.phy[index])}\n")
                    mesh_conf.write(f"id{index}_BATMAN_IFACE={quote(self.batman_iface[index])}\n")

                mesh_conf.write("BRIDGE=")
                for name in self.bridge:
                    mesh_conf.write(f"{name},")
                mesh_conf.write("\n")
        except:
            self.comms_status.mesh_cfg_status = \
                comms.STATUS.mesh_cfg_not_stored
            self.logger.error("not able to write new %s", file)
            return "FAIL", "not able to write new mesh.conf"

        self.comms_status.mesh_cfg_status = comms.STATUS.mesh_cfg_stored
        self.logger.debug("%s written", file)
        return "OK", "Mesh configuration stored"

    def __read_configs(self, mesh_conf_lines):
        pattern: str = r'(\w+)=(.*?)(?:\s*#.*)?$'
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
        config_file_path: str = "/opt/mesh_default.conf"
        mission_config_file_path: str = "/opt/mesh.conf"

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
