"""
mesh setting nats node
"""
import json
from shlex import quote

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

    def __init__(self, comms_status: cs.CommsStatus):
        self.api_version = 1
        self.ssid = ""
        self.key = ""
        self.ap_mac = ""
        self.country = ""
        self.frequency = ""
        self.ip_address = ""
        self.subnet = ""
        self.tx_power = ""
        self.mode = ""
        self.comms_status = comms_status

    def validate_mesh_settings(self) -> (str, str, str):
        """
        Validate mesh settings
        """
        if validation.validate_ssid(self.ssid) is False:
            return "FAIL", comms.STATUS.mesh_fail, "Invalid SSID"

        if validation.validate_wpa3_psk(self.key) is False:
            return "FAIL", comms.STATUS.mesh_fail, "Invalid WPA3 PSK"

        if validation.validate_ip_address(self.ip_address) is False:
            return "FAIL", comms.STATUS.mesh_fail, "Invalid IP address"

        if validation.validate_mode(self.mode) is False:
            return "FAIL", comms.STATUS.mesh_fail, "Invalid mode"

        if validation.validate_frequency(int(self.frequency)) is False:
            return "FAIL", comms.STATUS.mesh_fail, "Invalid frequency"

        if validation.validate_country_code(self.country) is False:
            return "FAIL", comms.STATUS.mesh_fail, "Invalid country code"

        if validation.validate_netmask(self.subnet) is False:
            return "FAIL", comms.STATUS.mesh_fail, "Invalid subnet"

        if validation.validate_tx_power(int(self.tx_power)) is False:
            return "FAIL", comms.STATUS.mesh_fail, "Invalid tx power"

        return "OK", "", "Mesh settings OK"

    def handle_mesh_settings(self, msg: str, path="/opt", file="mesh.conf") \
            -> (str, str, comms.STATUS):
        """
        Handle mesh settings
        """
        try:
            parameters = json.loads(msg)
            print(parameters)
            self.api_version = int(parameters["api_version"])
            self.ssid = quote(str(parameters["ssid"]))
            self.key = quote(str(parameters["key"]))
            self.ap_mac = quote(str(parameters["ap_mac"]))
            self.country = quote(str(parameters["country"]).lower())
            self.frequency = quote(str(parameters["frequency"]))
            self.ip_address = quote(str(parameters["ip"]))
            self.subnet = quote(str(parameters["subnet"]))
            self.tx_power = quote(str(parameters["tx_power"]))
            self.mode = quote(str(parameters["mode"]))

            ret, status, info = self.validate_mesh_settings()
            if ret == "FAIL":
                self.comms_status.mesh_cfg_status = \
                    comms.STATUS.mesh_configuration_not_stored
            else:
                ret, info, status = self.__save_settings(path, file)

        except (json.decoder.JSONDecodeError, KeyError,
                TypeError, AttributeError) as error:
            self.comms_status.mesh_cfg_status = \
                comms.STATUS.mesh_configuration_not_stored
            ret, status = "FAIL", self.comms_status.mesh_cfg_status
            info = "JSON format not correct" + str(error)

        return ret, info, status

    def __save_settings(self, path: str, file: str) -> (str, str, comms.STATUS):
        """
        Save mesh settings
        """
        try:
            with open(f"{path}/{file}", "w", encoding="utf-8") as mesh_conf:
                mesh_conf.write(f"MODE={quote(self.mode)}\n")
                mesh_conf.write("IP=10.20.15.3\n")
                mesh_conf.write("MASK=255.255.255.0\n")
                mesh_conf.write(f"MAC={quote(self.ap_mac)}\n")
                mesh_conf.write(f"KEY={quote(self.key)}\n")
                mesh_conf.write(f"ESSID={quote(self.ssid)}\n")
                mesh_conf.write(f"FREQ={quote(self.frequency)}\n")
                mesh_conf.write(f"TXPOWER={quote(self.tx_power)}\n")
                mesh_conf.write(f"COUNTRY={quote(self.country).upper()}\n")
                mesh_conf.write("MESH_VIF=wlp1s0\n")
                mesh_conf.write("PHY=phy0\n")
                mesh_conf.write("#CONCURRENCY configuration\n")
                mesh_conf.write("#CONCURRENCY=ap+mesh\n")
                mesh_conf.write("#MCC_CHANNEL=2412\n")
        except:
            self.comms_status.mesh_cfg_status = \
                comms.STATUS.mesh_configuration_not_stored
            return "FAIL", "not able to write new mesh.conf", \
                self.comms_status.mesh_cfg_status

        self.comms_status.mesh_cfg_status = \
            comms.STATUS.mesh_configuration_stored

        print('Settings saved')
        return "OK", "Mesh configuration stored", \
            self.comms_status.mesh_cfg_status
