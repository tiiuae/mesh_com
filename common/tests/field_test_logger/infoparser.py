import glob
import gpsd
import subprocess


def read_value(file : str) -> str:
    try:
        with open(file, 'r') as f:
            value = f.readline()

            return value.strip()

    except:
        return "NaN"


def get_hwmon_path(path : str) -> str:
    # hwmon directory should contain only one entry which is of format hwmon*
    return glob.glob(path)[0]


class InfoParser:

    def __init__(self):
        self.__gpsdConnected = False

        #
        self.altitude = 0
        self.latitude = -999999
        self.longitude = -999999
        self.gps_time = "NaN"
        self.pdop = 0
        #
        self.cpu_temp = "NaN"
        self.wifi_temp = "NaN"
        self.tmp100 = "NaN"
        #
        self.battery_voltage = "NaN"
        self.battery_current = "NaN"
        #
        self.nrf_current = "NaN"
        self.nrf_voltage = "NaN"
        self._3v3_current = "NaN"
        self._3v3_voltage = "NaN"

    # ---------------------------------

    def get_altitude(self) -> str:
        return str(self.altitude)

    def get_latitude(self) -> str:
        return str(self.latitude)

    def get_longitude(self) -> str:
        return str(self.longitude)

    def get_gps_time(self) -> str:
        return self.gps_time

    def get_pdop(self) -> str:
        return str(self.pdop)

    def get_cpu_temp(self):
        return self.cpu_temp

    def get_tmp100(self):
        return self.tmp100

    def get_wifi_temp(self):
        return self.wifi_temp

    def get_battery_voltage(self):
        return self.battery_voltage

    def get_battery_current(self):
        return self.battery_current

    def get_nrf_current(self):
        return self.nrf_current

    def get_nrf_voltage(self):
        return self.nrf_voltage

    def get_3v3_current(self):
        return self._3v3_current

    def get_3v3_voltage(self):
        return self._3v3_voltage

    # ---------------------------------

    @staticmethod
    def __get_battery_status() -> tuple[str, str]:
        voltage = read_value("/sys/class/power_supply/max1726x_battery/voltage_now")
        current = read_value("/sys/class/power_supply/max1726x_battery/current_now")

        #print(f"battery voltage: {voltage}")
        #print(f"discharging current: {current}")

        return voltage, current

    @staticmethod
    def __get_ina209_status() -> tuple[str, str, str, str]:
        nrf_current = read_value(get_hwmon_path("/sys/class/i2c-adapter/i2c-10/10-0040/hwmon/hwmon*/curr1_input"))
        nrf_voltage = read_value(get_hwmon_path("/sys/class/i2c-adapter/i2c-10/10-0040/hwmon/hwmon*/in1_input"))

        #print(f"nrf current: {nrf_current}")
        #print(f"nrf voltage: {nrf_voltage}")

        _3v3_current = read_value(get_hwmon_path("/sys/class/i2c-adapter/i2c-10/10-0045/hwmon/hwmon*/curr1_input"))
        _3v3_voltage = read_value(get_hwmon_path("/sys/class/i2c-adapter/i2c-10/10-0045/hwmon/hwmon*/in1_input"))

        #print(f"3v3 current: {_3v3_current}")
        #print(f"3v3 voltage: {_3v3_voltage}")

        return nrf_current, nrf_voltage, _3v3_current, _3v3_voltage

    @staticmethod
    def __get_temperatures() -> tuple[str, str, str]:
        cpu_temp = read_value("/sys/class/thermal/thermal_zone0/temp")
        tmp100 = read_value(get_hwmon_path("/sys/class/i2c-adapter/i2c-10/10-0049/driver/10-0049/hwmon/hwmon*/temp1_input"))
        wifi_temp = read_value(get_hwmon_path("/sys/class/ieee80211/phy0/device/hwmon/hwmon*/temp1_input"))

       # print(f"cpu_temp: {cpu_temp}")
        #print(f"tmp100: {tmp100}")
        #print(f"wifi_temp: {wifi_temp}")

        return cpu_temp, tmp100, wifi_temp

    def __update_gpsd_data(self):
        if not self.__gpsdConnected:
            gpsd.connect()
            self.__gpsdConnected = True

        try:
            gps_response = gpsd.get_current()
            self.latitude = gps_response.lat
            self.longitude = gps_response.lon
            self.altitude = gps_response.alt
            self.gps_time = gps_response.time
            self.pdop = gps_response.pdop
        except:
            self.latitude = -999999
            self.longitude = -999999
            self.altitude = 0
            self.gps_time ="NaN"
            self.pdop = 0


    def update(self):
        self.__update_gpsd_data()
        self.cpu_temp, self.tmp100, self.wifi_temp = self.__get_temperatures()
        self.battery_voltage, self.battery_current = self.__get_battery_status()
        self.nrf_current, self.nrf_voltage, self._3v3_current, self._3v3_voltage = self.__get_ina209_status()
        #print(f"lat: {self.latitude}")
        #print(f"lon: {self.longitude}")
        #print(f"alt: {self.altitude}")
