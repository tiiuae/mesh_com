import csv
import time
from datetime import datetime

import wifi_info
import infoparser

LOG_FOLDER_LOCATION = "logs/"
LOGGING_INTERVAL_SECONDS = 3

class FieldTestLogger:
    def __init__(self):
        self.__logger_functions = {}
        self.__header = []
        self.__logger_output = []
        self.__filename = ""

    def create_csv(self):
        now_str = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
        self.__filename = f"{now_str}.csv"
        self.__construct_csv_header()

        with open(f"{LOG_FOLDER_LOCATION}{self.__filename}", 'w', newline='') as csvfile:
            wr = csv.writer(csvfile, delimiter=',')
            wr.writerow(self.__header)

    def append_csv(self):
        self.__run_logger_functions()

        with open(f"{LOG_FOLDER_LOCATION}{self.__filename}", 'a') as csvfile:
            wr = csv.writer(csvfile, delimiter=',')
            wr.writerow(self.__logger_output)

    def register_logger_function(self, name : str, function):
        """
        params:
            [name] name of the header row element
            [function] callback that returns single string value
        """
        self.__logger_functions[name] = function

    def __run_logger_functions(self):
        self.__logger_output.clear()
        for func in self.__logger_functions.values():
            self.__logger_output.append(func())

    def __construct_csv_header(self):
        for name in self.__logger_functions:
            self.__header.append(name)


def timestamp() -> str:
    now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    return now_str


if __name__ == '__main__':
    ftl = FieldTestLogger()
    wifi_stats = wifi_info.WifiInfo()
    info = infoparser.InfoParser()

    ftl.register_logger_function("Timestamp", timestamp)
    wifi_stats.update()
    info.update()

    ftl.register_logger_function("channel", wifi_stats.get_channel)
    ftl.register_logger_function("rssi", wifi_stats.get_rssi)
    ftl.register_logger_function("txpower", wifi_stats.get_txpower)
    ftl.register_logger_function("noise", wifi_stats.get_noise)
    ftl.register_logger_function("RX MCS", wifi_stats.get_rx_mcs)
    ftl.register_logger_function("TX MCS", wifi_stats.get_tx_mcs)

    ftl.register_logger_function("latitude", info.get_latitude)
    ftl.register_logger_function("longitude", info.get_longitude)
    ftl.register_logger_function("altitude", info.get_altitude)

    ftl.register_logger_function("cpu temp", info.get_cpu_temp)
    ftl.register_logger_function("wifi temp", info.get_wifi_temp)
    ftl.register_logger_function("tmp100", info.get_tmp100)

    ftl.register_logger_function("battery voltage", info.get_battery_voltage)
    ftl.register_logger_function("battery current", info.get_battery_current)
    ftl.register_logger_function("nRF voltage", info.get_nrf_voltage)
    ftl.register_logger_function("nRF current", info.get_nrf_current)
    ftl.register_logger_function("3v3 voltage", info.get_3v3_voltage)
    ftl.register_logger_function("3v3 current", info.get_3v3_current)



    ftl.create_csv()


    wifi_stats.update()
    info.update()
    ftl.append_csv()
    #time.sleep(LOGGING_INTERVAL_SECONDS)

