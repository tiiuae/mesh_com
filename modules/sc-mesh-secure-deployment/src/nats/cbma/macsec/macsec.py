import os
import signal

from secure_socket.secure_connection import SecureConnection, SecureConnectionCallbackType
from utils.common import run_script_bug_workaround_retcode
from utils.networking import get_mac_from_ipv6
from utils.macsec import get_macsec_config
from utils.logging import get_logger


CBMA_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), os.pardir))

SIGNALS = [signal.SIGINT, signal.SIGTERM]

logger = get_logger()


class MACsec(object):
    MAX_RETRIES = 5
    UPPER_KEY_LENGTH = 64
    LOWER_KEY_LENGTH = 32
    KEY_REFRESH_CODE = 255

    def __init__(self,
                 is_upper: bool,
                 interface: str,
                 enable_encryption: bool,
                 secure_connection_callback: SecureConnectionCallbackType
                 ) -> None:
        self.is_upper = is_upper
        self.interface = interface
        self.enable_encryption = enable_encryption
        self.secure_connection_callback = secure_connection_callback

        self.tx_key: str = ''
        self.rx_key: str = ''
        self.tx_port: int = 0
        self.rx_port: int = 0


    def __termination_handler(self) -> None:
        for s in SIGNALS:
            signal.signal(s, signal.SIG_IGN)
        ret_code = os.EX_OK if self.cleanup() else os.EX_OSERR
        os._exit(ret_code)


    def start(self, conn: SecureConnection) -> bool:
        peer_ipv6 = conn.get_peer_name()[0]
        self.peer_mac = get_mac_from_ipv6(peer_ipv6)

        for s in SIGNALS:
            signal.signal(s, lambda *_: self.__termination_handler())

        return self.update_config(conn) and self.connect()


    def update_config(self, conn: SecureConnection) -> bool:
        try:
            keys, ports = get_macsec_config(conn)
            conn.close()
        except Exception as e:
            logger.error(f"Exception when obtaining MACsec configuration: {e}")
            conn.close()
            return False
        tx_key, rx_key = keys
        tx_port, rx_port = ports

        if self.is_upper:
            self.tx_key = tx_key.hex()[:self.UPPER_KEY_LENGTH]
            self.rx_key = rx_key.hex()[:self.UPPER_KEY_LENGTH]
        else:
            self.tx_key = tx_key.hex()[:self.LOWER_KEY_LENGTH]
            self.rx_key = rx_key.hex()[:self.LOWER_KEY_LENGTH]

        self.tx_port = tx_port
        self.rx_port = rx_port

        return True


    def refresh_keys(self) -> bool:
        logger.info(f"Refreshing MACsec keys with {self.peer_mac} peer")

        if not self.secure_connection_callback(self.update_config):
            return False

        l_or_u = 'u' if self.is_upper else 'l'
        update_str = f"bash -x {CBMA_ROOT}/scripts/mess/update_mess.sh {l_or_u} {self.peer_mac} {self.tx_key} {self.rx_key}"

        return not run_script_bug_workaround_retcode(update_str.split())


    def monitor(self) -> bool:
        l_or_u = 'u' if self.is_upper else 'l'
        monitor_str = f"bash -x {CBMA_ROOT}/scripts/mess/monitor_mess.sh {l_or_u} {self.interface} {self.peer_mac}"

        logger.info(f"Monitoring MACsec connection with {self.peer_mac} peer")
        success = False
        while run_script_bug_workaround_retcode(monitor_str.split(),
                                                quiet_on_error=True) == self.KEY_REFRESH_CODE:
            # if not self.refresh_keys():
            #     success = False
            #     break
            success = True

        logger.warning(f"MACsec connection with {self.peer_mac} peer ended")
        return success


    def connect(self) -> bool:
        l_or_u = 'u' if self.is_upper else 'l'
        encryption = 'on' if self.enable_encryption else 'off'

        create_str = f"bash -x {CBMA_ROOT}/scripts/mess/create_mess.sh {l_or_u} {self.interface} {self.peer_mac} {self.tx_key} {self.rx_key} {encryption}"

        retries = 0
        while retries < self.MAX_RETRIES:
            retries += 1
            self.__cleanup()

            logger.info(f"Attempt {retries}/{self.MAX_RETRIES} - Creating MACsec connection with {self.peer_mac} peer")
            if run_script_bug_workaround_retcode(create_str.split()):
                logger.error(f"Attempt {retries}/{self.MAX_RETRIES} - Unable to create MACsec connection with {self.peer_mac} peer")
            elif self.monitor():
                retries = 0

        self.cleanup()
        return False


    def __cleanup(self) -> bool:
        l_or_u = 'u' if self.is_upper else 'l'
        cleanup_str = f"bash -x {CBMA_ROOT}/scripts/mess/cleanup_mess.sh {l_or_u} {self.interface} {self.peer_mac}"

        return not run_script_bug_workaround_retcode(cleanup_str.split())


    def cleanup(self) -> bool:
        logger.info(f"Cleaning up MACsec {self.peer_mac} peer setup")

        if not self.__cleanup():
            logger.error(f"Unable to clean {self.peer_mac} MACsec setup")
            return False

        return True
