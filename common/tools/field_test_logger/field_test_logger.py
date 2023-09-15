import csv
import os
import time
import argparse
from datetime import datetime

import wifi_info
import infoparser

LOG_FOLDER_LOCATION = r"/root/field_test_logs/"
LOGGING_INTERVAL_SECONDS = 1  # LOGGING_INTERVAL_SECONDS needs to be >=0.4s


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
    argparse = argparse.ArgumentParser(description='Field Test Logger',
                                       prog='field_test_logger.py')
    argparse.add_argument('-u', '--unique', help='Add unique suffix to log file name')
    argparse.add_argument('-i', '--interface',
                          help='Specify wifi interface name e.g. wlp1s0 (default: wlp1s0)')
    argparse.add_argument('-b', '--batman',
                          help="Specify batman interface name e.g. bat0 (default: bat0)")

    args = argparse.parse_args()

    if args.batman is None:
        BATMAN_ARG = "bat0"
    else:
        BATMAN_ARG = args.batman

    if args.interface is None:
        INTERFACE_ARG = "wlp1s0"
    else:
        INTERFACE_ARG = args.interface

    if args.unique is None or args.unique == "":
        UNIQUE_ARG = f"_{INTERFACE_ARG}_{BATMAN_ARG}"
    else:
        UNIQUE_ARG = f"_{INTERFACE_ARG}_{BATMAN_ARG}_{args.unique}"

    try:
        with open("/etc/comms_pcb_version", "r") as file_r:
            PCB_VERSION = file_r.read().strip().split("=")[1]
    except FileNotFoundError:
        PCB_VERSION = "unknown"

    ftl = FieldTestLogger()
    wifi_stats = wifi_info.WifiInfo(LOGGING_INTERVAL_SECONDS, INTERFACE_ARG, BATMAN_ARG)
    info = infoparser.InfoParser(PCB_VERSION)

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
    ftl.register_logger_function("speed", info.get_speed)
    ftl.register_logger_function("climb", info.get_climb)
    ftl.register_logger_function("track", info.get_track)

    ftl.register_logger_function("cpu temp [mC]", info.get_cpu_temp)
    ftl.register_logger_function("battery temp [mC]", info.get_bat_temp)
    ftl.register_logger_function("wifi temp [mC]", info.get_wifi_temp)
    ftl.register_logger_function("tmp100 [mC]", info.get_tmp100)

    ftl.register_logger_function("3V3 mpcie3 voltage [mV]", info.get_mpcie3_voltage)
    ftl.register_logger_function("3V3 mpcie3 current [mA]", info.get_mpcie3_current)
    ftl.register_logger_function("3V3 mpcie5 voltage [mV]", info.get_mpcie5_voltage)
    ftl.register_logger_function("3V3 mpcie5 current [mA]", info.get_mpcie5_current)
    ftl.register_logger_function("3V3 mpcie7 voltage [mV]", info.get_mpcie7_voltage)
    ftl.register_logger_function("3V3 mpcie7 current [mA]", info.get_mpcie7_current)

    ftl.register_logger_function("bme280 rel. humidity [m%]", info.get_humidity)
    ftl.register_logger_function("bme280 pressure [kPa]", info.get_pressure)
    ftl.register_logger_function("bme280 temperature [mC]", info.get_temperature)

    ftl.register_logger_function("battery voltage [uV]", info.get_battery_voltage)
    ftl.register_logger_function("battery current [uA]", info.get_battery_current)
    ftl.register_logger_function("nRF voltage [mV]", info.get_nrf_voltage)
    ftl.register_logger_function("nRF current [mA]", info.get_nrf_current)
    ftl.register_logger_function("3v3 voltage [mV]", info.get_3v3_voltage)
    ftl.register_logger_function("3v3 current [mA]", info.get_3v3_current)
    ftl.register_logger_function("DCin (XT30) voltage [mV]", info.get_dc_voltage)
    ftl.register_logger_function("DCin (XT30) current [mA]", info.get_dc_current)

    ftl.create_csv(f"{wifi_stats.get_mac_addr()}{UNIQUE_ARG}")

    while True:
        start = time.time()
        wifi_stats.update()
        info.update()
        ftl.append_csv()
        d = time.time() - start

        # adjust delay for precise logging interval
        if d < LOGGING_INTERVAL_SECONDS:
            time.sleep(LOGGING_INTERVAL_SECONDS - d)
