import os
import signal

from multiprocessing import Process
from typing import Tuple

from utils.logging import get_logger, setup_global_file_logging
from utils.networking import get_interface_link_local_ipv6_address
from utils.multicast import bytes_2_multicast_postfix
from models.certificates import CBMACertificates
from models.cbma import ICBMA

from certificates.certificates import OpenSSLCertificate
from multicast.service import MulticastService
from secure_socket.client import FileBasedSecureSocketClient
from secure_socket.server import FileBasedSecureSocketServer
from secure_socket.secure_connection import SecureConnection, MACsecCallbackType
from macsec.macsec import MACsec


SIGNALS = [signal.SIGINT, signal.SIGTERM]


class CBMA(ICBMA):
    SETUP_GLOBAL_FILE_LOGGING: bool = True
    PROCESS_START_TIMEOUT: float = 0.2
    PROCESS_STOP_TIMEOUT: float = 1.0

    # TODO - Give more visibility to 'enable_macsec_encryption' and 'is_upper'
    def __init__(self,
                 interface: str,
                 port: int,
                 certificates: CBMACertificates,
                 batman: str,
                 enable_macsec_encryption: bool = False,
                 is_upper: bool = False
                 ) -> None:
        self.interface = interface
        self.ipv6 = get_interface_link_local_ipv6_address(interface)
        self.port = port
        self.certificates = certificates
        self.batman = batman
        self.enable_macsec_encryption = enable_macsec_encryption
        self.is_upper = is_upper

        self._macsec_processes: dict[int, Tuple[str, Process]] = {}

        self.logger = get_logger(log_dir=interface)

        self.certificate = OpenSSLCertificate
        self.secure_socket_server = FileBasedSecureSocketServer
        self.secure_socket_client = FileBasedSecureSocketClient

        def secure_connection_callback(peer_ipv6: str,
                                       macsec_callback: MACsecCallbackType) -> bool:
            self.logger.info(f"Creating secure connection with {peer_ipv6}")
            try:
                if self.ipv6 > peer_ipv6:
                    secure_socket_server = self.secure_socket_server(self.interface,
                                                                     self.port,
                                                                     self.certificates,
                                                                     macsec_callback)
                    return secure_socket_server.listen()

                secure_socket_client = self.secure_socket_client(self.interface,
                                                                 peer_ipv6,
                                                                 self.port,
                                                                 self.certificates,
                                                                 macsec_callback)
                return secure_socket_client.connect()
            except Exception as e:
                self.logger.error(f"Exception when creating secure connection with {peer_ipv6}: {e}")
            return False

        self.__secure_connection_callback = secure_connection_callback


    def __setup_instance(self) -> None:
        if self.SETUP_GLOBAL_FILE_LOGGING:
            setup_global_file_logging(self.interface)

        for cert_path in self.certificates.chain:
            certificate = self.certificate(cert_path)
            akid = certificate.get_authority_key_identifier()
            if multicast_group_postfix := bytes_2_multicast_postfix(akid):
                break
        else:
            multicast_group_postfix = ":1"
            self.logger.warning(f"Unable to find multicast group in {self.certificates.chain} - Using ::1")

        def multicast_secure_connection_callback(ipv6: str) -> bool:
            return self.__secure_connection_callback(ipv6, self.__macsec_callback)

        self.multicast_service = MulticastService(interface=self.interface,
                                                  port=self.port,
                                                  group_postfix=multicast_group_postfix,
                                                  secure_connection_callback=multicast_secure_connection_callback)
        for s in SIGNALS:
            signal.signal(s, lambda *_: self.__termination_handler())

        signal.signal(signal.SIGCHLD, self.__child_handler)


    def __child_handler(self, signum: int, frame) -> None:
        while True:
            try:
                child_pid, _ = os.waitpid(-1, os.WNOHANG)
            except ChildProcessError:
                break

            if child_pid == 0:
                break

            # NOTE - Needed like this because PyOpenSSL is also creating processes
            if ipv6_process := self._macsec_processes.get(child_pid):
                self.multicast_service.deauthenticate_peer(ipv6_process[0])


    def __termination_handler(self) -> None:
        for s in SIGNALS:
            signal.signal(s, signal.SIG_IGN)

        ret_code = os.EX_OK if self.stop() else os.EX_OSERR
        os._exit(ret_code)


    def __macsec_callback(self, conn: SecureConnection) -> bool:
        ipv6 = conn.get_peer_name()[0]

        def macsec_secure_connection_callback(callback: MACsecCallbackType) -> bool:
            return self.__secure_connection_callback(ipv6, callback)

        macsec = MACsec(is_upper=self.is_upper,
                        interface=self.interface,
                        enable_encryption=self.enable_macsec_encryption,
                        secure_connection_callback=macsec_secure_connection_callback)

        # NOTE - Daemon processes cannot spawn other multiprocessing.Processes
        process = Process(target=macsec.start, args=(conn,))
        process.start()

        process.join(timeout=self.PROCESS_START_TIMEOUT)
        if not process.is_alive() or not process.pid:
            return False

        self._macsec_processes[process.pid] = (ipv6, process)

        return True


    def start(self) -> bool:
        self.__setup_instance()

        self.logger.info(f"Starting CBMA")

        return self.multicast_service.start()


    def stop(self) -> bool:
        self.logger.info(f"Stopping CBMA")

        success = self.multicast_service.stop()

        for _, process in self._macsec_processes.values():
            pid = process.pid
            try:
                if not process.is_alive():
                    continue
                process.terminate()
                process.join(timeout=self.PROCESS_STOP_TIMEOUT)
            except Exception as e:
                self.logger.error(f"Exception when terminating {pid}: {e}")
            try:
                if not process.is_alive():
                    continue
                process.kill()
                process.join(timeout=self.PROCESS_STOP_TIMEOUT)
            except Exception as e:
                self.logger.error(f"Exception when killing {pid}: {e}")
            if process.is_alive():
                self.logger.error(f"Unable to stop {pid}")
                success = False

        self.logger.info(f"CBMA cleanup was {'' if success else 'not '}successful")
        return success
