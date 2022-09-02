import csv
import os
import time
from datetime import datetime

import wifi_info
import infoparser

LOG_FOLDER_LOCATION = r"/root/field_test_logs/"
LOGGING_INTERVAL_SECONDS = 1


def check_log_folder():
    """
    Check if default log folder exists and create it if it doesn't
    """
    if not os.path.exists(LOG_FOLDER_LOCATION):
        os.makedirs(LOG_FOLDER_LOCATION)
        print("Created log directory")


def check_log_file(filename):
    """
    Check if log file exists and return unique name
    if needed.
    """

    count = 0
    prefix = ""

    while True:
        if os.path.isfile(f"{LOG_FOLDER_LOCATION}{prefix}{filename}"):
            count += 1
            prefix = f"{str(count)}_"
        else:
            return f"{prefix}{filename}"


class FieldTestLogger:
    def __init__(self):
        self.__logger_functions = {}
        self.__header = []
        self.__logger_output = []
        self.__filename = ""

    def create_csv(self, suffix : str):
        now_str = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
        self.__filename = f"{now_str}_{suffix}.csv"
        self.__construct_csv_header()

        check_log_folder()
        self.__filename = check_log_file(self.__filename)

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

    # ----------------------------------------

    def __run_logger_functions(self):
        self.__logger_output.clear()
        for func in self.__logger_functions.values():
            self.__logger_output.append(func())

    def __construct_csv_header(self):
        for name in self.__logger_functions:
            self.__header.append(name)


# ----------------------------------------

def timestamp() -> str:
    now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    return now_str


# ----------------------------------------


if __name__ == '__main__':
    ftl = FieldTestLogger()
    wifi_stats = wifi_info.WifiInfo(LOGGING_INTERVAL_SECONDS)
    info = infoparser.InfoParser()

    ftl.register_logger_function("Timestamp", timestamp)
    wifi_stats.update()
    info.update()

    ftl.register_logger_function("GPS time", info.get_gps_time)

    ftl.register_logger_function("channel", wifi_stats.get_channel)
    ftl.register_logger_function("rssi [MAC,dBm;MAC,dBm ...]", wifi_stats.get_rssi)
    ftl.register_logger_function("txpower [dBm]", wifi_stats.get_txpower)
    ftl.register_logger_function("noise [dBm]", wifi_stats.get_noise)
    ftl.register_logger_function("RX MCS [MAC,MCS;MAC,MCS ...]", wifi_stats.get_rx_mcs)
    ftl.register_logger_function("TX MCS [MAC,MCS;MAC,MCS ...]", wifi_stats.get_tx_mcs)
    ftl.register_logger_function("RX throughput [Bits/s]", wifi_stats.get_rx_throughput)
    ftl.register_logger_function("TX throughput [Bits/s]", wifi_stats.get_tx_throughput)
    ftl.register_logger_function("Neighbors", wifi_stats.get_neighbors)
    ftl.register_logger_function("Originators", wifi_stats.get_originators)

    ftl.register_logger_function("latitude", info.get_latitude)
    ftl.register_logger_function("longitude", info.get_longitude)
    ftl.register_logger_function("altitude", info.get_altitude)
    ftl.register_logger_function("PDOP", info.get_pdop)

    ftl.register_logger_function("cpu temp [mC]", info.get_cpu_temp)
    ftl.register_logger_function("battery temp [mC]", info.get_bat_temp)
    ftl.register_logger_function("wifi temp [mC]", info.get_wifi_temp)
    ftl.register_logger_function("tmp100 [mC]", info.get_tmp100)

    ftl.register_logger_function("battery voltage [uV]", info.get_battery_voltage)
    ftl.register_logger_function("battery current [uA]", info.get_battery_current)
    ftl.register_logger_function("nRF voltage [mV]", info.get_nrf_voltage)
    ftl.register_logger_function("nRF current [mA]", info.get_nrf_current)
    ftl.register_logger_function("3v3 voltage [mV]", info.get_3v3_voltage)
    ftl.register_logger_function("3v3 current [mA]", info.get_3v3_current)

    ftl.create_csv(wifi_stats.get_mac_addr())

    while True:
        wifi_stats.update()
        info.update()
        ftl.append_csv()
        time.sleep(LOGGING_INTERVAL_SECONDS)
