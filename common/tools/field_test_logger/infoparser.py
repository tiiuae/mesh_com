import glob
import gpsd


def read_value(file: str) -> str:
    try:
        with open(file, 'r') as f:
            value = f.readline()

            return value.strip()

    except:
        return "NaN"


def get_hwmon_path_from_options(paths: [str]) -> str:
    for path in paths:
        p = get_hwmon_path(path)
        if p is not "NaN":
            return p
    return "NaN"


def get_hwmon_path(path: str) -> str:
    # hwmon directory should contain only one entry which is of format hwmon*
    try:
        return glob.glob(path)[0]
    except:
        return "NaN"


# ----------------------------------------


class InfoParser:

    def __init__(self):
        self.__gpsdConnected = False
        #
        self.__altitude = 0
        self.__latitude = -999999
        self.__longitude = -999999
        self.__gps_time = "NaN"
        self.__pdop = 0
        #
        self.__cpu_temp = "NaN"
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

    # ----------------------------------------

    def get_altitude(self) -> str:
        return str(self.__altitude)

    def get_latitude(self) -> str:
        return str(self.__latitude)

    def get_longitude(self) -> str:
        return str(self.__longitude)

    def get_gps_time(self) -> str:
        return self.__gps_time

    def get_pdop(self) -> str:
        return str(self.__pdop)

    def get_cpu_temp(self):
        return self.__cpu_temp

    def get_bat_temp(self):
        return self.__bat_temp

    def get_tmp100(self):
        return self.__tmp100

    def get_wifi_temp(self):
        return self.__wifi_temp

    def get_battery_voltage(self):
        return self.__battery_voltage

    def get_battery_current(self):
        return self.__battery_current

    def get_nrf_current(self):
        return self.__current_nrf

    def get_nrf_voltage(self):
        return self.__voltage_nrf

    def get_3v3_current(self):
        return self.__current_3v3

    def get_3v3_voltage(self):
        return self.__voltage_3v3

    def get_dc_current(self):
        return self.__current_dc

    def get_dc_voltage(self):
        return self.__voltage_dc

    # ----------------------------------------

    def __update_battery_status(self):
        self.__battery_voltage = read_value("/sys/class/power_supply/max1726x_battery/voltage_now")
        self.__battery_current = read_value("/sys/class/power_supply/max1726x_battery/current_now")

    def __get_cpu_load(self) -> str:
        load = read_value("/proc/loadavg")
        return load.split()[0]

    def __update_ina2xx_status(self):
        self.__current_nrf = read_value(
            get_hwmon_path_from_options(
                ["/sys/class/i2c-adapter/i2c-1/1-0040/hwmon/hwmon*/curr1_input",
                 "/sys/class/i2c-adapter/i2c-10/10-0040/hwmon/hwmon*/curr1_input"]))
        self.__voltage_nrf = read_value(
            get_hwmon_path_from_options(
                ["/sys/class/i2c-adapter/i2c-1/1-0040/hwmon/hwmon*/in1_input",
                 "/sys/class/i2c-adapter/i2c-10/10-0040/hwmon/hwmon*/in1_input"]))

        self.__current_3v3 = read_value(
            get_hwmon_path_from_options(
                ["/sys/class/i2c-adapter/i2c-1/1-0045/hwmon/hwmon*/curr1_input",
                 "/sys/class/i2c-adapter/i2c-10/10-0045/hwmon/hwmon*/curr1_input"]))
        self.__voltage_3v3 = read_value(
            get_hwmon_path_from_options(
                ["/sys/class/i2c-adapter/i2c-1/1-0045/hwmon/hwmon*/in1_input",
                 "/sys/class/i2c-adapter/i2c-10/10-0045/hwmon/hwmon*/in1_input"]))

        self.__current_dc = read_value(
            get_hwmon_path("/sys/class/i2c-adapter/i2c-0/0-0041/hwmon/hwmon*/curr1_input"))

        self.__voltage_dc = read_value(
            get_hwmon_path("/sys/class/i2c-adapter/i2c-0/0-0041/hwmon/hwmon*/in1_input"))

    def __update_temperatures(self):
        self.__cpu_temp = read_value("/sys/class/thermal/thermal_zone0/temp")
        self.__bat_temp = read_value("/sys/class/thermal/thermal_zone1/temp")
        self.__tmp100 = read_value(
            get_hwmon_path_from_options(
                ["/sys/class/i2c-adapter/i2c-1/1-0049/hwmon/hwmon*/temp1_input",
                 "/sys/class/i2c-adapter/i2c-10/10-0049/hwmon/hwmon*/temp1_input"]))
        self.__wifi_temp = read_value(
            get_hwmon_path("/sys/class/ieee80211/phy0/device/hwmon/hwmon*/temp1_input"))

    def __update_gpsd_data(self):
        if not self.__gpsdConnected:
            gpsd.connect()
            self.__gpsdConnected = True

        try:
            gps_response = gpsd.get_current()
            self.__latitude = gps_response.lat
            self.__longitude = gps_response.lon
            self.__altitude = gps_response.alt
            self.__gps_time = gps_response.time
            self.__pdop = gps_response.pdop
        except:
            self.__latitude = -999999
            self.__longitude = -999999
            self.__altitude = 0
            self.__gps_time = "NaN"
            self.__pdop = 0

    # ----------------------------------------

    def update(self):
        """
        Update variables with latest info
        """
        self.__update_gpsd_data()
        self.__update_temperatures()
        self.__update_battery_status()
        self.__update_ina2xx_status()
        # print(f"lat: {self.latitude}")
        # print(f"lon: {self.longitude}")
        # print(f"alt: {self.altitude}")
