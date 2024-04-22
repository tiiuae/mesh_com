from typing import Optional
from copy import deepcopy
from multiprocessing import Process

from models.certificates import CBMACertificates
from models.controller import ICBMAController
from utils.networking import get_interface_mac_address
from utils.logging import get_logger

from cbma import CBMA


logger = get_logger(log_dir='.')


class CBMAController(ICBMAController):
    PROCESS_START_TIMEOUT: float = 0.2
    PROCESS_STOP_TIMEOUT: float = 10.0

    def __init__(self,
                 port: int,
                 batman: str,
                 certificates: CBMACertificates,
                 is_upper: bool = False,
                 enable_macsec_encryption: bool = False
                 ) -> None:
        self.port = port
        self.batman = batman
        self.certificates = certificates
        self.is_upper = is_upper
        self.enable_macsec_encryption = enable_macsec_encryption

        self.processes: dict[str, Process] = {}


    def __stop_interface_process(self, interface: str, process: Process) -> bool:
        try:
            if process.is_alive():
                logger.info(f"Terminating CBMA in {interface}")
                process.terminate()
                process.join(timeout=self.PROCESS_STOP_TIMEOUT)
        except Exception as e:
            logger.error(f"Terminating CBMA in {interface} error: {e}")

        try:
            if process.is_alive():
                logger.warning(f"Failed to gracefully stop CBMA in {interface}")
                logger.info(f"Killing CBMA in {interface}")
                process.kill()
                process.join(timeout=self.PROCESS_STOP_TIMEOUT)
        except Exception as e:
            logger.error(f"Killing CBMA in {interface} error: {e}")

        if process.is_alive():
            logger.error(f"Unable to stop CBMA in {interface}")
            return False
        return True


    def add_interface(self, interface: str) -> bool:
        logger.debug(f"Adding {interface}")
        if process := self.processes.get(interface):
            if process.is_alive():
                logger.error(f"Unable to add {interface} as CBMA is already running in it")
                return False

            logger.warning(f"Found stopped CBMA instance in {interface} - Cleaning before re-adding it")
            if not self.remove_interface(interface):
                return False

        certificates = deepcopy(self.certificates)
        certificates.cert.rstrip('/')
        certificates.cert += f"/{get_interface_mac_address(interface)}.crt"

        cbma = CBMA(interface=interface,
                    port=self.port,
                    certificates=certificates,
                    batman=self.batman,
                    enable_macsec_encryption=self.enable_macsec_encryption,
                    is_upper=self.is_upper)

        process = Process(target=cbma.start)
        process.start()
        process.join(self.PROCESS_START_TIMEOUT)
        if not process.is_alive():
            logger.error(f"Unable to add {interface}")
            return False

        self.processes[interface] = process
        return True


    def remove_interface(self, interface: str) -> bool:
        logger.debug(f"Removing {interface}")
        if not (process := self.processes.get(interface)):
            logger.error(f"No CBMA instance found in {interface}")
            return False

        success = self.__stop_interface_process(interface, process)

        logger.debug(f"{interface} was {'' if success else 'not '}removed")
        return success


    def is_interface_running(self, interface: str) -> bool:
        if process := self.processes.get(interface, None):
            return process.is_alive()
        return False


    def start_interface(self, interface: str) -> bool:
        logger.info(f"Starting CBMA in {interface}")
        if not (process := self.processes.get(interface, None)):
            logger.error(f"No existing CBMA process found for {interface}")
            return False
        if self.is_interface_running(interface):
            logger.debug(f"CBMA is already running in {interface}")
            return True

        process.start()
        process.join(timeout=self.PROCESS_START_TIMEOUT)
        if not process.is_alive():
            logger.error(f"Unable to start CBMA in {interface}")
            return False
        logger.debug(f"Successfully started CBMA in {interface}")
        return True


    def stop_interface(self, interface: str) -> bool:
        logger.info(f"Stopping CBMA in {interface}")
        if not (process := self.processes.get(interface, None)):
            logger.error(f"No existing CBMA process found for {interface}")
            return False
        if not self.is_interface_running(interface):
            logger.debug(f"CBMA is already stopped in {interface}")
            return True
        if not self.__stop_interface_process(interface, process):
            return False
        logger.debug(f"Successfully stopped CBMA in {interface}")
        return True


    def start(self) -> bool:
        logger.info('Starting all processes')
        success = True
        for process in self.processes.values():
            process.start()
            process.join(timeout=self.PROCESS_START_TIMEOUT)
            if not process.is_alive():
                success = False
        return success


    def stop(self) -> bool:
        logger.info('Stopping all processes')
        success = True
        for interface, process in self.processes.items():
            if not self.__stop_interface_process(interface, process):
                success = False
        return success


    def join(self, timeout: Optional[float] = None) -> bool:
        logger.debug('Joining all processes')
        success = True
        for process in self.processes.values():
            if not process.join(timeout):
                success = False
        return success
