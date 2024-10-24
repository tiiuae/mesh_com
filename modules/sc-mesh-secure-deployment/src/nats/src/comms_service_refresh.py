"""
Refresher class for the comms service registration
"""
import logging
import threading
import time
import subprocess

class CommsServiceRefresh:
    """
    Comms service publisher class
    """
    DNS_SERVICE_EVENT_LOOP_TIMEOUT: int = 5

    def __init__(self, __hostname, __logger: logging.Logger) -> None:

        self.logger: logging.Logger = __logger.getChild("CommsServicePublisher")
        self.logger.setLevel(logging.INFO)
        self.__event: threading.Event = threading.Event()
        self.__hostname: str = __hostname


    def __refresh_hostname(self) -> None:
        """
        Refresh the hostname with avahi-resolve-host-name
        """
        try:
            subprocess.run(["avahi-resolve-host-name", self.__hostname + ".local"],
                           check=True,
                           stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE)
        except subprocess.CalledProcessError:
            self.logger.exception("Error refreshing hostname")

    def dns_service_refresh(self) -> None:
        """
        Register and re-announce service periodically
        """
        self.logger.info("Refresh start")
        while not self.__event.is_set():
            time.sleep(self.DNS_SERVICE_EVENT_LOOP_TIMEOUT)
            self.__refresh_hostname()
        self.logger.info("Refresh stopped")


    def shutdown_service(self) -> None:
        """
        Shutdown the service
        """
        self.__event.set()
