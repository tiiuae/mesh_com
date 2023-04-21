"""
Comms status class
"""
from .comms_common import STATUS


class CommsStatus:
    """
    Maintains mesh and radio statuses
    """

    def __init__(self):
        # Todo: Initial status needs to be read from system
        # Consider e.g. use case where mission configurations
        # has been applied and then device is switched off.
        # Mission settings are then applied automatically during
        # nex boot thus this kind of dummy init is not proper.
        self.__mesh_status = STATUS.no_status
        self.__mesh_cfg_status = STATUS.mesh_default
        self.__is_mission_cfg = False
        self.__is_radio_on = True  # True since mesh is started via initd
        self.__is_visualisation_active = False
        # self.__device_status()

    @property
    def mesh_status(self):
        return self.__mesh_status

    @mesh_status.setter
    def mesh_status(self, status: STATUS):
        if status.name in STATUS.__members__:
            self.__mesh_status = status
            # self.__device_status()
    @property
    def mesh_cfg_status(self):
        return self.__mesh_cfg_status

    @mesh_cfg_status.setter
    def mesh_cfg_status(self, status: STATUS):
        if status is STATUS.mesh_configuration_stored \
                or status is STATUS.mesh_configuration_not_stored:
            self.__mesh_cfg_status = status

    @property
    def is_mission_cfg(self):
        return self.__is_mission_cfg

    @is_mission_cfg.setter
    def is_mission_cfg(self, value: bool):
        self.__is_mission_cfg = value

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

    # def __device_status(self):
    #    """
    #    Get mesh status from wpa_supplicant status and
    #    update internal states accordingly.
    #    """
