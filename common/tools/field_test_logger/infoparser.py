"""
InfoParser class is used to parse the information from the system and GPSD.
"""
import glob
import gpsd


def read_value(file: str) -> str:
    """
    Read a single value from a file.
    """
    try:
        with open(file, 'r') as file_handle:
            value = file_handle.readline()

            return value.strip()

    except:
        return "NaN"


def get_hwmon_path_from_options(paths: [str]) -> str:
    """
    Get the path to the hwmon directory.
    """
    for path in paths:
        path_to_validate = get_hwmon_path(path)
        if path_to_validate != "NaN":
            return path_to_validate
    return "NaN"


def get_hwmon_path(path: str) -> str:
    """
    Get the path to the hwmon directory.
    """
    # hwmon directory should contain only one entry which is of format hwmon*
    try:
        return glob.glob(path)[0]
    except:
        return "NaN"

#pylint: disable=too-many-instance-attributes, too-many-public-methods
class InfoParser:
    """
    InfoParser class is used to parse the information from the system and GPSD.
    """

    def __init__(self, pcb_version: str):
        """
        Initialize the InfoParser class.
        """
        self.__pcb_version = pcb_version
        #
        self.__gpsd_connected = False
        self.__max17260_connected = True
        #
        self.__altitude = 0
        self.__latitude = -999999
        self.__longitude = -999999
        self.__gps_time = "NaN"
        self.__pdop = 0
        self.__speed = 0
        self.__climb = 0
        self.__track = 0
        #
        self.__cpu_temp = "NaN"
        self.__bat_temp = "NaN"
        self.__wifi_temp = "NaN"
        self.__tmp100 = "NaN"
        #
        self.__battery_voltage = "NaN"
        self.__battery_current = "NaN"
        #
        self.__current_nrf = "NaN"
        self.__voltage_nrf = "NaN"
        self.__current_3v3 = "NaN"
        self.__voltage_3v3 = "NaN"
        self.__current_dc = "NaN"
        self.__voltage_dc = "NaN"
        #
        self.__current_mpcie3 = "NaN"
        self.__voltage_mpcie3 = "NaN"
        self.__current_mpcie5 = "NaN"
        self.__voltage_mpcie5 = "NaN"
        self.__current_mpcie7 = "NaN"
        self.__voltage_mpcie7 = "NaN"
        #
        self.__humidity = "NaN"
        self.__pressure = "NaN"
        self.__temperature = "NaN"

    # ----------------------------------------

    def get_altitude(self) -> str:
        """
        Get altitude
        """
        return str(self.__altitude)

    def get_latitude(self) -> str:
        """
        Get latitude
        """
        return str(self.__latitude)

    def get_longitude(self) -> str:
        """
        Get longitude
        """
        return str(self.__longitude)

    def get_gps_time(self) -> str:
        """
        Get GPS time
        """
        return self.__gps_time

    def get_pdop(self) -> str:
        """
        Get pdop
        """
        return str(self.__pdop)

    def get_speed(self) -> str:
        """
        Get speed
        """
        return str(self.__speed)

    def get_climb(self) -> str:
        """
        Get climb
        """
        return str(self.__climb)

    def get_track(self) -> str:
        """
        Get track
        """
        return str(self.__track)

    def get_cpu_temp(self):
        """
        Get CPU temperature
        """
        return self.__cpu_temp

    def get_bat_temp(self):
        """
        Get battery temperature
        """
        return self.__bat_temp

    def get_tmp100(self):
        """
        Get TMP100 temperature
        """
        return self.__tmp100

    def get_wifi_temp(self):
        """
        Get wifi temperature
        """
        return self.__wifi_temp

    def get_battery_voltage(self):
        """
        Get battery voltage
        """
        return self.__battery_voltage

    def get_battery_current(self):
        """
        Get battery current
        """
        return self.__battery_current

    def get_nrf_current(self):
        """
        Get NRF current
        """
        return self.__current_nrf

    def get_nrf_voltage(self):
        """
        Get NRF voltage
        """
        return self.__voltage_nrf

    def get_3v3_current(self):
        """
        Get 3V3 current
        """
        return self.__current_3v3

    def get_3v3_voltage(self):
        """
        Get 3V3 voltage
        """
        return self.__voltage_3v3

    def get_dc_current(self):
        """
        Get DC current
        """
        return self.__current_dc

    def get_dc_voltage(self):
        """
        Get DC voltage
        """
        return self.__voltage_dc

    def get_mpcie3_current(self):
        """
        Get mPCIe3 current
        """
        return self.__current_mpcie3

    def get_mpcie3_voltage(self):
        """
        Get mPCIe3 voltage
        """
        return self.__voltage_mpcie3

    def get_mpcie5_current(self):
        """
        Get mPCIe5 current
        """
        return self.__current_mpcie5

    def get_mpcie5_voltage(self):
        """
        Get mPCIe5 voltage
        """
        return self.__voltage_mpcie5

    def get_mpcie7_current(self):
        """
        Get mPCIe7 current
        """
        return self.__current_mpcie7

    def get_mpcie7_voltage(self):
        """
        Get mPCIe7 voltage
        """
        return self.__voltage_mpcie7

    def get_humidity(self):
        """
        Get humidity
        """
        return self.__humidity

    def get_pressure(self):
        """
        Get pressure
        """
        return self.__pressure

    def get_temperature(self):
        """
        Get temperature
        """
        return self.__temperature

    # ----------------------------------------

    def __update_battery_status(self):
        """
        Update battery status
        """
        if self.__max17260_connected:
            # check if max17260 is connected and if not, set all values to NaN and
            # don't try to check anymore ( causing a lot of errors in the dmesg log )
            if read_value("/sys/class/power_supply/max1726x_battery/uevent") == "":
                self.__battery_voltage = "NaN"
                self.__battery_current = "NaN"
                self.__max17260_connected = False
                return

            self.__battery_voltage = read_value(
                get_hwmon_path("/sys/class/power_supply/max1726x_battery/voltage_now"))
            self.__battery_current = read_value(
                get_hwmon_path("/sys/class/power_supply/max1726x_battery/current_now"))

    # @staticmethod
    # def __get_cpu_load() -> str:
    #     """
    #     Get CPU load
    #     """
    #     load = read_value("/proc/loadavg")
    #     return load.split()[0]

    def __update_bme280_status(self):
        """
        Update BME280 status
        """
        self.__humidity = read_value(
            get_hwmon_path("/sys/bus/iio/devices/iio:device*/in_humidityrelative_input"))
        self.__pressure = read_value(
            get_hwmon_path("/sys/bus/iio/devices/iio:device*/in_pressure_input"))
        self.__temperature = read_value(
            get_hwmon_path("/sys/bus/iio/devices/iio:device*/in_temp_input"))

    def __update_ina231_status(self):
        """
        Update INA231 status
        """
        if self.__pcb_version == "1":
            self.__current_mpcie3 = read_value(
                get_hwmon_path("/sys/class/i2c-adapter/i2c-1/1-0046/hwmon/hwmon*/curr1_input"))
            self.__voltage_mpcie3 = read_value(
                get_hwmon_path("/sys/class/i2c-adapter/i2c-1/1-0046/hwmon/hwmon*/in1_input"))
            self.__current_mpcie5 = read_value(
                get_hwmon_path("/sys/class/i2c-adapter/i2c-1/1-0045/hwmon/hwmon*/curr1_input"))
            self.__voltage_mpcie5 = read_value(
                get_hwmon_path("/sys/class/i2c-adapter/i2c-1/1-0045/hwmon/hwmon*/in1_input"))
            self.__current_mpcie7 = read_value(
                get_hwmon_path("/sys/class/i2c-adapter/i2c-1/1-0040/hwmon/hwmon*/curr1_input"))
            self.__voltage_mpcie7 = read_value(
                get_hwmon_path("/sys/class/i2c-adapter/i2c-1/1-0040/hwmon/hwmon*/in1_input"))
            self.__current_nrf = read_value(
                get_hwmon_path("/sys/class/i2c-adapter/i2c-1/1-0044/hwmon/hwmon*/curr1_input"))
            self.__voltage_nrf = read_value(
                get_hwmon_path("/sys/class/i2c-adapter/i2c-1/1-0044/hwmon/hwmon*/in1_input"))

    def __update_ina2xx_status(self):
        """
        Update INA2xx status
        """
        if self.__pcb_version != "1":
            self.__current_nrf = read_value(
                get_hwmon_path("/sys/class/i2c-adapter/i2c-1/1-0040/hwmon/hwmon*/curr1_input"))
            self.__voltage_nrf = read_value(
                get_hwmon_path("/sys/class/i2c-adapter/i2c-1/1-0040/hwmon/hwmon*/in1_input"))

            self.__current_3v3 = read_value(
                get_hwmon_path("/sys/class/i2c-adapter/i2c-1/1-0045/hwmon/hwmon*/curr1_input"))
            self.__voltage_3v3 = read_value(
                get_hwmon_path("/sys/class/i2c-adapter/i2c-1/1-0045/hwmon/hwmon*/in1_input"))

        self.__current_dc = read_value(
            get_hwmon_path_from_options(
                ["/sys/class/i2c-adapter/i2c-0/0-0041/hwmon/hwmon*/curr1_input",
                "/sys/class/i2c-adapter/i2c-1/1-0041/hwmon/hwmon*/curr1_input"]))
        self.__voltage_dc = read_value(
            get_hwmon_path_from_options(
                ["/sys/class/i2c-adapter/i2c-0/0-0041/hwmon/hwmon*/in1_input",
                "/sys/class/i2c-adapter/i2c-1/1-0041/hwmon/hwmon*/in1_input"]))

    def __update_temperatures(self):
        """
        Update temperatures
        """
        self.__cpu_temp = read_value("/sys/class/thermal/thermal_zone0/temp")
        self.__bat_temp = read_value("/sys/class/thermal/thermal_zone1/temp")
        self.__tmp100 = read_value(
            get_hwmon_path_from_options(
                ["/sys/class/i2c-adapter/i2c-1/1-0049/hwmon/hwmon*/temp1_input",
                 "/sys/class/i2c-adapter/i2c-10/10-0049/hwmon/hwmon*/temp1_input"]))
        self.__wifi_temp = read_value(
            get_hwmon_path("/sys/class/ieee80211/phy0/device/hwmon/hwmon*/temp1_input"))

    def __update_gpsd_data(self):
        """
        Update GPSD data
        """
        if not self.__gpsd_connected:
            gpsd.connect()
            self.__gpsd_connected = True

        try:
            gps_response = gpsd.get_current()
            self.__latitude = gps_response.lat
            self.__longitude = gps_response.lon
            self.__altitude = gps_response.alt
            self.__gps_time = gps_response.time
            self.__pdop = gps_response.pdop
            self.__speed = gps_response.hspeed
            self.__climb = gps_response.climb
            self.__track = gps_response.track
        except:
            self.__latitude = -999999
            self.__longitude = -999999
            self.__altitude = 0
            self.__gps_time = "NaN"
            self.__pdop = 0
            self.__speed = 0
            self.__climb = 0
            self.__track = 0

    # ----------------------------------------

    def update(self):
        """
        Update variables with the latest info
        """
        self.__update_gpsd_data()
        self.__update_temperatures()
        self.__update_battery_status()
        self.__update_ina2xx_status()
        self.__update_ina231_status()
        self.__update_bme280_status()
        # print(f"lat: {self.latitude}")
        # print(f"lon: {self.longitude}")
        # print(f"alt: {self.altitude}")
