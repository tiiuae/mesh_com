"""
Radio control module
"""
import json
import logging
from comms_controller import CommsController


class RadioAdaptation:
    """
    This class is used to control the radio
    """
    def __init__(self, comms_ctrl: CommsController, logger: logging.Logger):
        self.__comms_ctrl = comms_ctrl
        self.__logger = logger
        self.__settings = comms_ctrl.settings

    def apply(self, index: str) -> bool:
        """
        Apply the radio configuration
        :param index: The index of the radio
        :return: True if successful, False otherwise
        """
        return self.__send_cmd("APPLY", index)

    def start(self, index: str = "*") -> bool:
        """
        Start the radio interfaces
        :param index: The index of the radio, index or * for all
        :return: True if successful, False otherwise
        """
        if index != "":
            return self.__send_cmd("UP", index)
        return False

    def stop(self, index: str ="*") -> bool:
        """
        Stop the radio interfaces
        :param index: The index of the radio, index or * for all
        :return: True if successful, False otherwise
        """
        if index != "":
            return self.__send_cmd("DOWN", index)
        return False

    def __send_cmd(self, cmd, index) -> bool:
        """
        Send a command to the radio
        :param cmd: The command to send
        :param index: The index of the radio
        :return: True if successful, False otherwise
        """
        cmd = json.dumps(
            {
                "api_version": 1,
                "cmd": cmd,
                "radio_index": index,
            }
        )

        ret, info, _ = self.__comms_ctrl.command.handle_command(cmd, self.__comms_ctrl)

        if ret != "OK":
            self.__logger.error(f"Error: Unable to bring down the radio interfaces! {info}")
            return False
        return True

    def get_radio_index(self, radio_name: str) -> str:
        """
        Get the index of the radio
        :param radio_name: The name of the radio
        :return: The index of the radio
        """
        for index, mesh_vif in enumerate(self.__settings.mesh_vif):
            if mesh_vif == radio_name:
                return str(index)
        return ""
