"""
This module contains CommsInterfaces class which provides
an interface to get a list wireless interfaces and their
properties.
"""
# pylint: disable=too-few-public-methods, too-many-locals

import os
import subprocess
from typing import Tuple, List, Dict


class CommsInterfaceInfo:
    """
    A class to store wireless interface device information.
    """

    def __init__(self):
        self.if_name: str = ""
        self.mac: str = ""
        self.bus_type: str = ""
        self.driver: str = ""
        self.vendor_id: str = ""
        self.device_id: str = ""


class CommsInterfaces:
    """
    A class to gather a list if wireless interfaces and their properties.
    """

    def __init__(self):
        self.interfaces: List[CommsInterfaceInfo] = []

    def __get_wireless_device_info(self):
        """
        Gathers a list of available wlp* and halow* interface devices and their
        properties.

        Arguments:
            None

        Returns:
            None
        """
        net_path = "/sys/class/net"
        device_names = os.listdir(net_path)

        self.interfaces.clear()

        for device_name in device_names:
            if_info = CommsInterfaceInfo()
            device_path = os.path.join(net_path, device_name)

            if "-" in device_name:
                continue

            if device_name.startswith("wlp") or device_name.startswith(
                "halow"
            ):
                if_info.if_name = device_name
                # Read MAC address
                mac_address_file = os.path.join(device_path, "address")
                with open(mac_address_file, "r", encoding="utf-8") as file:
                    if_info.mac = file.read().strip()

                # Determine bus type
                if "pci" in os.readlink(device_path):
                    if_info.bus_type = "PCI"
                elif ".usb" in os.readlink(
                    device_path
                ) and "spi" in os.readlink(device_path):
                    if_info.bus_type = "USB+SPI"
                elif "spi" in os.readlink(device_path):
                    if_info.bus_type = "SPI"

                # Get driver information
                uevent_file = os.path.join(device_path, "device", "uevent")
                with open(uevent_file, "r", encoding="utf-8") as file:
                    lines = file.readlines()
                    for line in lines:
                        if line.startswith("DRIVER="):
                            if_info.driver = line.split("=")[1].strip()

                # Get vendor information if available
                vendor_file = os.path.join(device_path, "device", "vendor")
                try:
                    with open(vendor_file, "r", encoding="utf-8") as file:
                        if_info.vendor_id = file.read().strip()
                except FileNotFoundError:
                    if_info.vendor_id = "N/A"

                # Get device information if available
                device_file = os.path.join(device_path, "device", "device")
                try:
                    with open(device_file, "r", encoding="utf-8") as file:
                        if_info.device_id = file.read().strip()
                except FileNotFoundError:
                    if_info.device_id = "N/A"

                # 2nd approach to get vendor and device id for HaLow device
                if if_info.vendor_id == "N/A" or if_info.device_id == "N/A":
                    if if_info.if_name.startswith("halow"):
                        (
                            if_info.vendor_id,
                            if_info.device_id,
                        ) = self.__get_halow_vendor_dev_info(device_path)

                self.interfaces.append(if_info)
        # end of method

    @staticmethod
    def __get_halow_vendor_dev_info(dev_path: str) -> Tuple[str, str]:
        """
        Checks from HaLow device's link information and in case it is connected
        to USB then vendor_id and device_id information is read from USB device
        properties.

        Arguments:
            dev_path (str) -- Halow device path under /sys/class/net/.

        Returns:
            Tuple[str, str] -- A tuple that contains vendor_id and device_id.
        """
        vendor_id = "N/A"
        device_id = "N/A"
        found_usb_bus = False
        bus_no = "N/A"
        port_no = "N/A"
        sub_port = "N/A"
        dev_no = "N/A"

        try:  # pylint: disable=too-many-nested-blocks
            link_info = os.readlink(dev_path)
            if "halow" in link_info and ".usb" in link_info:
                # Find the index of ".usb"
                usb_index = link_info.find(".usb")
                if usb_index != -1:
                    usb_path = link_info[usb_index + 1:]
                    parts = usb_path.split("/")

                    if len(parts) >= 3:
                        bus_no = parts[1][3:]
                        port_no = parts[2][2:]
                        sub_port = parts[3][4:]

                first_match = f"/:  Bus 0{bus_no}.Port {port_no}"
                second_match = f"|__ Port {sub_port}:"

                lsusb_t_output = subprocess.check_output(
                    ["lsusb", "-t"]
                ).decode("utf-8")
                lsusb_t_lines = lsusb_t_output.split("\n")
                for line in lsusb_t_lines:
                    if first_match in line:
                        found_usb_bus = True
                        continue
                    if found_usb_bus and second_match in line:
                        parts = line.split(": ")
                        for part in parts:
                            if "Dev" in part:
                                sub_parts = part.split(",")
                                dev_parts = sub_parts[0].split(" ")
                                dev_no = dev_parts[1]
                                break
                        # exit loop
                        break

                # Now get the vendor and device id using lsusb
                bus_no = bus_no.zfill(3)
                dev_no = dev_no.zfill(3)
                match_criteria = f"Bus {bus_no} Device {dev_no}"

                lsusb_output = subprocess.check_output(["lsusb"]).decode(
                    "utf-8"
                )
                lsusb_lines = lsusb_output.split("\n")

                for line in lsusb_lines:
                    if match_criteria in line:
                        parts = line.split(" ")
                        vendor_and_dev_id = parts[5].split(":")
                        vendor_id = vendor_and_dev_id[0]
                        device_id = vendor_and_dev_id[1]
                        break
        except Exception:  # pylint: disable = broad-except
            pass
        return vendor_id, device_id

    def get_wireless_device_info(self) -> List[Dict[str, str]]:
        """
        Returns a list of wireless interface device properties in JSON
        compatible format.

        Arguments:
            None

        Returns:
            List[Dict[str, str]]: -- A list of interaface dictionaries.
        """
        # Refresh radio interfaces list each time
        self.__get_wireless_device_info()
        info_list = []

        for device in self.interfaces:
            info = {
                "interface_name": device.if_name,
                "mac_address": device.mac,
                "bus_type": device.bus_type,
                "driver": device.driver,
                "vendor_id": device.vendor_id,
                "device_id": device.device_id,
            }
            info_list.append(info)
        return info_list
