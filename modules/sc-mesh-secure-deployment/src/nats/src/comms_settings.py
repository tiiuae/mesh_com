"""
mesh setting nats node
"""
import json
from shlex import quote
import logging
import re
import os

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

    def __init__(self, comms_status: [cs.CommsStatus, ...], logger):
        self.logger: logging = logger
        self.api_version: int = 1
        self.radio_index: [int, ...] = []
        self.ssid: [str, ...] = []
        self.key: [str, ...] = []
        self.country: [str, ...] = []
        self.frequency: [str, ...] = []
        self.frequency_mcc: [str, ...] = []
        self.tx_power: [str, ...] = []
        self.mode: [str, ...] = []
        self.priority: [str, ...] = []
        self.role: str = ""
        self.mesh_vif = []
        self.mptcp = []
        self.slaac = []
        self.msversion: str = ""
        self.delay: str = ""  # delay for channel change
        self.comms_status = comms_status
        # TODO: check can we do this
        ret, info = self.__load_settings()
        self.logger.debug("load settings: %s, %s", ret, info)

    def validate_mesh_settings(self, index: int) -> (str, str):
        """
        Validate mesh settings
        """

        self.logger.debug("validate mesh settings")

        # pylint: disable=too-many-return-statements
        if index > len(self.ssid) or \
                validation.validate_ssid(self.ssid[index]) is False:
            return "FAIL", "Invalid SSID"
        self.logger.debug("validate mesh settings ssid ok")

        if index > len(self.key) or \
                validation.validate_wpa3_psk(self.key[index]) is False:
            return "FAIL", "Invalid WPA3 PSK"
        self.logger.debug("validate mesh settings wpa3 ok")

        if index > len(self.mode) or \
                validation.validate_mode(self.mode[index]) is False:
            return "FAIL", "Invalid mode"
        self.logger.debug("validate mesh settings mode ok")

        if index > len(self.frequency) or \
                validation.validate_frequency(int(self.frequency[index])) is False:
            return "FAIL", "Invalid frequency"
        self.logger.debug("validate mesh settings freq ok")

        if index > len(self.frequency_mcc) or \
                validation.validate_frequency(int(self.frequency_mcc[index])) is False:
            return "FAIL", "Invalid mcc frequency"
        self.logger.debug("validate mesh settings mcc freq ok")

        if index > len(self.country) or \
                validation.validate_country_code(self.country[index]) is False:
            return "FAIL", "Invalid country code"
        self.logger.debug("validate mesh settings country ok")

        if index > len(self.tx_power) or \
                validation.validate_tx_power(int(self.tx_power[index])) is False:
            return "FAIL", "Invalid tx power"
        self.logger.debug("validate mesh settings tx power ok")

        if index > len(self.priority) or \
                validation.validate_priority(self.priority[index]) is False:
            return "FAIL", "Invalid priority"
        self.logger.debug("validate mesh settings priority ok")

        if index > len(self.role) or \
                validation.validate_role(self.role) is False:
            return "FAIL", "Invalid role"
        self.logger.debug("validate mesh settings role ok")

        if index > len(self.mptcp) or \
                validation.validate_mptcp(self.mptcp[index]) is False:
            return "FAIL", "Invalid mptcp value"
        self.logger.debug("validate mesh settings mptcp ok")

        if index > len(self.slaac) or \
                validation.validate_slaac(self.slaac[index]) is False:
            return "FAIL", "Invalid slaac ifaces"
        self.logger.debug("validate mesh settings slaac ifaces ok")

        return "OK", "Mesh settings OK"

    def __clean_all_settings(self) -> None:
        """
        Clean all settings
        """
        self.radio_index: [int, ...] = []
        self.ssid: [str, ...] = []
        self.key: [str, ...] = []
        self.country: [str, ...] = []
        self.frequency: [str, ...] = []
        self.frequency_mcc: [str, ...] = []
        self.tx_power: [str, ...] = []
        self.mode: [str, ...] = []
        self.priority: [str, ...] = []
        self.mesh_vif: [str, ...] = []
        self.mptcp: [str, ...] = []
        self.slaac: [str, ...] = []

    def handle_mesh_settings_channel_change(
        self, msg: str, path="/opt", file="mesh_stored.conf"
    ) -> (str, str, str):
        """
        Handle mesh settings
        """
        try:
            parameters = json.loads(msg)

            ret, info = self.__load_settings()
            self.logger.debug("load settings: %s, %s", ret, info)

            freq, radio_index = map(
                quote, (str(parameters["frequency"]), str(parameters["radio_index"]))
            )

            if validation.validate_radio_index(
                radio_index
            ) and validation.validate_frequency(int(freq)) and int(radio_index) < len(
                self.frequency):
                self.frequency[int(radio_index)] = freq
                ret, info = "TRIGGER", "Channel change settings OK"
            else:
                ret, info = (
                    "FAIL",
                    f"Invalid radio_index or frequency index {radio_index}",
                )

            self.logger.debug(" settings validation: %s, %s", ret, info)

            if ret == "FAIL":
                self.comms_status[
                    int(radio_index)
                ].mesh_cfg_status = comms.STATUS.mesh_cfg_not_stored
                self.logger.error("save settings failed: %s, %s", ret, info)
            else:
                ret, info = self.__save_settings(path, file, int(radio_index))
                if ret == "OK":
                    ret = "TRIGGER"
                self.logger.debug("save settings for channel change: %s, %s", ret, info)
                return ret, info, str(radio_index)

        except (
            json.decoder.JSONDecodeError,
            KeyError,
            TypeError,
            AttributeError,
        ) as error:
            try:
                radio_index
            except NameError:
                radio_index = 0
            self.comms_status[
                radio_index
            ].mesh_cfg_status = comms.STATUS.mesh_cfg_not_stored
            ret, info = "FAIL", "JSON format not correct" + str(error)
            self.logger.error("csa settings validation: %s, %s", ret, info)

        return ret, info, "-1"

    def handle_mesh_settings(
        self, msg: str, path="/opt", file="mesh_stored.conf"
    ) -> (str, str):
        """
        Handle mesh settings
        """
        try:
            parameters_set = json.loads(msg)

            self.msversion = "nats"
            self.api_version = int(parameters_set["api_version"])
            self.role = quote(str(parameters_set["role"]))

            self.__clean_all_settings()

            # sort radios by index
            parameters_set["radios"] = sorted(
                parameters_set["radios"], key=lambda k: k.get("radio_index", 0)
            )

            for parameters in parameters_set["radios"]:
                self.radio_index.append(int(parameters["radio_index"]))
                self.ssid.append(quote(str(parameters["ssid"])))
                self.key.append(quote(str(parameters["key"])))
                self.country.append(quote(str(parameters["country"]).lower()))
                self.frequency.append(quote(str(parameters["frequency"])))
                self.frequency_mcc.append(quote(str(parameters["frequency_mcc"])))
                self.tx_power.append(quote(str(parameters["tx_power"])))
                self.mode.append(quote(str(parameters["mode"])))
                self.priority.append(quote(str(parameters["priority"])))
                self.mesh_vif.append(quote(str(parameters["mesh_vif"])))
                self.mptcp.append(quote(str(parameters["mptcp"])))
                self.slaac.append(str(parameters["slaac"]))

            for index in self.radio_index:
                self.logger.debug("Mesh settings validation index: %s", str(index))
                ret, info = self.validate_mesh_settings(index)

                self.logger.debug(
                    "Mesh settings validation id %s: %s, %s", str(index), ret, info
                )
                if ret == "FAIL":
                    self.comms_status[
                        index
                    ].mesh_cfg_status = comms.STATUS.mesh_cfg_not_stored
                    self.logger.error(
                        "Settings validation failed: %s, %s, id %s",
                        ret,
                        info,
                        str(index),
                    )
                    _, _ = self.__load_settings()  # to restore cleaned settings
                    return ret, info + ", id " + str(index)

            # separate loop to avoid saving if validation fails
            for index in self.radio_index:
                ret, info = self.__save_settings(path, file, index)
                self.logger.debug(
                    "save settings index %s: %s, %s", str(index), ret, info
                )
                if ret == "FAIL":
                    self.comms_status[
                        index
                    ].mesh_cfg_status = comms.STATUS.mesh_cfg_not_stored
                    self.logger.error(
                        "save settings failed: %s, %s, id %s", ret, info, str(index)
                    )
                    _, _ = self.__load_settings()  # to restore cleaned settings
                    return ret, info + ", id " + str(index)

        except (
            json.decoder.JSONDecodeError,
            KeyError,
            TypeError,
            AttributeError,
        ) as error:
            try:
                index
            except NameError:
                index = 0
            self.comms_status[index].mesh_cfg_status = comms.STATUS.mesh_cfg_not_stored
            ret, _ = "FAIL", self.comms_status[index].mesh_cfg_status
            info = "JSON format not correct " + str(error)
            self.logger.error("Mesh settings validation: %s, %s", ret, info)

        return ret, info

    def __save_settings(self, path: str, file: str, index: int) -> (str, str):
        """
        Save mesh settings
        """
        try:
            with open(
                f"{path}/{str(index)}_{file}", "w", encoding="utf-8"
            ) as mesh_conf:
                # not currently in json mesh_settings
                mesh_conf.write(f"ROLE={quote(self.role)}\n")
                mesh_conf.write(f"MSVERSION={quote(self.msversion)}\n")
                mesh_conf.write(f"id{str(index)}_MODE={quote(self.mode[index])}\n")
                mesh_conf.write(f"id{str(index)}_KEY={quote(self.key[index])}\n")
                mesh_conf.write(f"id{str(index)}_ESSID={quote(self.ssid[index])}\n")
                mesh_conf.write(f"id{str(index)}_FREQ={quote(self.frequency[index])}\n")
                mesh_conf.write(
                    f"id{str(index)}_FREQ_MCC={quote(self.frequency_mcc[index])}\n"
                )
                mesh_conf.write(
                    f"id{str(index)}_TXPOWER={quote(self.tx_power[index])}\n"
                )
                mesh_conf.write(
                    f"id{str(index)}_COUNTRY={quote(self.country[index]).upper()}\n"
                )
                mesh_conf.write(
                    f"id{str(index)}_PRIORITY={quote(self.priority[index])}\n"
                )
                mesh_conf.write(
                    f"id{str(index)}_MESH_VIF={quote(self.mesh_vif[index])}\n"
                )
                mesh_conf.write(f"id{str(index)}_MPTCP={quote(self.mptcp[index])}\n")

                self.slaac[index] = self.slaac[index].replace('"', "")
                self.slaac[index] = self.slaac[index].replace("'", "")

                mesh_conf.write(f'id{str(index)}_SLAAC="{self.slaac[index]}"\n')

        except Exception as error:
            self.comms_status[index].mesh_cfg_status = comms.STATUS.mesh_cfg_not_stored
            self.logger.error("not able to write new %s", f"{str(index)}_{file}, {error}")
            return "FAIL", "not able to write new mesh.conf"

        self.comms_status[index].mesh_cfg_status = comms.STATUS.mesh_cfg_stored
        self.logger.debug("%s written", f"{str(index)}_{file}")
        return "OK", "Mesh configuration stored"

    def __read_configs(self, mesh_conf_lines) -> None:
        pattern: str = r"(\w+)=(.*?)(?:\s*#.*)?$"
        # Find all key-value pairs in the text
        matches = re.findall(pattern, mesh_conf_lines, re.MULTILINE)
        for match in matches:
            if "id" in match[0]:
                name_parts = match[0].split("_")[1:]
                name = "_".join(name_parts)
                if name == "MODE":
                    self.mode.append(match[1])
                    index = int(match[0].split("_")[0].replace("id", ""))
                    if index not in self.radio_index:
                        self.radio_index.append(index)
                elif name == "KEY":
                    self.key.append(match[1])
                elif name == "ESSID":
                    self.ssid.append(match[1])
                elif name == "FREQ":
                    self.frequency.append(match[1])
                elif name == "FREQ_MCC":
                    self.frequency_mcc.append(match[1])
                elif name == "TXPOWER":
                    self.tx_power.append(match[1])
                elif name == "COUNTRY":
                    self.country.append(match[1])
                elif name == "PRIORITY":
                    self.priority.append(match[1])
                elif name == "MESH_VIF":
                    self.mesh_vif.append(match[1])
                elif name == "MPTCP":
                    self.mptcp.append(match[1])
                elif name == "SLAAC":
                    self.slaac.append(str(match[1]))
                else:
                    self.logger.error("unknown config parameter: %s", name)
            else:  # global config without index
                if match[0] == "MSVERSION":
                    self.msversion = match[1]
                elif match[0] == "ROLE":
                    self.role = match[1]
                else:
                    pass  # self.logger.error("unknown config parameter: %s", match[0])

    def __load_settings(self) -> (str, str):
        """
        Load mesh settings
        return: OK, FAIL
        """
        config_file_path: str = "/opt/mesh_default.conf"

        self.__clean_all_settings()

        try:
            for index in range(0, len(self.comms_status)):
                file = f"/opt/{str(index)}_mesh.conf"
                if os.path.exists(file):
                    self.logger.debug("mesh config file %s found", file)
                    with open(file, "r", encoding="utf-8") as mesh_conf:
                        mesh_conf_lines = mesh_conf.read()
                        self.__read_configs(mesh_conf_lines)
                else:
                    if index == 0:
                        self.logger.debug(
                            "mesh config file %s not found, loading default", file
                        )
                        with open(config_file_path, "r", encoding="utf-8") as mesh_conf:
                            mesh_conf_lines = mesh_conf.read()
                            self.__read_configs(mesh_conf_lines)
                    else:
                        self.logger.debug("index not supported for default: %s", index)
        except:
            self.logger.error("not able to read mesh config files")
            return "FAIL", "not able to read mesh config files"
        return "OK", "Mesh configuration loaded"
