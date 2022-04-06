import csv
import time
from datetime import datetime

import wifi_info


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

        with open(f"logs/{self.__filename}", 'w', newline='') as csvfile:
            wr = csv.writer(csvfile, delimiter=',')
            wr.writerow(self.__header)

    def append_csv(self):
        self.__run_logger_functions()

        with open(f"logs/{self.__filename}", 'a') as csvfile:
            wr = csv.writer(csvfile, delimiter=',')
            wr.writerow(self.__logger_output)

    def register_logger_function(self, name : str, function):
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

    ftl.register_logger_function("Timestamp", timestamp)
    wifi_stats.update()

    ftl.register_logger_function("channel", wifi_stats.get_channel)
    ftl.register_logger_function("txpower", wifi_stats.get_txpower)
    ftl.register_logger_function("noise", wifi_stats.get_noise)
    ftl.register_logger_function("RX MCS", wifi_stats.get_rx_mcs)
    ftl.register_logger_function("TX MCS", wifi_stats.get_tx_mcs)

    ftl.create_csv()

    ftl.append_csv()

