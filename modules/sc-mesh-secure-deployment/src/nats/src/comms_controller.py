import subprocess
import logging
from typing import List
from . import comms_command
from . import comms_settings
from . import comms_status
from . import comms_mesh_telemetry


class CommsController:  # pylint: disable=too-few-public-methods
    """
    Mesh network
    """

    def __init__(self, interval: int = 1000):
        self.interval: int = interval
        self.debug_mode_enabled: bool = False

        # base logger for comms and which is used by all other modules
        self.main_logger: logging.Logger = logging.getLogger("comms")
        self.main_logger.setLevel(logging.DEBUG)
        log_formatter: logging.Formatter = logging.Formatter(
            fmt="%(asctime)s :: %(name)-18s :: %(levelname)-8s :: %(message)s"
        )
        console_handler: logging.StreamHandler = logging.StreamHandler()
        console_handler.setFormatter(log_formatter)
        self.main_logger.addHandler(console_handler)

        self.c_status = []

        self.radio_amount: int = self.__check_radio_amount()
        if self.radio_amount < 0:
            self.main_logger.debug("Radio amount Error")

        for i in range(0, self.radio_amount):
            self.c_status.append(
                comms_status.CommsStatus(
                    self.main_logger.getChild(f"status {str(i)}"), i
                )
            )

        self.settings = comms_settings.CommsSettings(
            self.c_status, self.main_logger.getChild("settings")
        )

        for cstat in self.c_status:
            if cstat.index < len(self.settings.mesh_vif):
                cstat.wifi_interface = self.settings.mesh_vif[cstat.index]

        self.command = comms_command.Command(
            self.c_status, self.main_logger.getChild("command")
        )
        self.telemetry = comms_mesh_telemetry.MeshTelemetry(
            self.interval, self.main_logger.getChild("telemetry")
        )

        # logger for this module and derived from main logger
        self.logger = self.main_logger.getChild("controller")

    def __check_radio_amount(self) -> int:
        """
        Check how many radios are available

        $ awk -F= '$2=="wlan"{getline;print $2}' /sys/class/net/*/uevent
        # wlan1
        # wlp1s0

        arguments:
            None
        returns:
            int: number of radios

        """
        try:
            cmd = "awk -F= '$2==\"wlan\"{getline;print $2}' /sys/class/net/*/uevent"
            cmd_result = subprocess.run(
                cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True
            )
        except Exception as e:
            # main_logger is used here as self.logger is not yet initialized
            self.main_logger.exception(e)
            return -1
        return len(cmd_result.stdout.decode().splitlines())