import argparse
import sys
import signal
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1]))

from multicast.service import MulticastService
from certificates.certificates import OpenSSLCertificate
from utils.multicast import bytes_2_multicast_postfix


def cbma_impl(client_ipv6: str) -> bool:
    print(f'Mutual Authentication is required for {client_ipv6}')
    return False


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--interface", help="Network interface to run CBMA on",
                        type=str, required=True)
    parser.add_argument("-p", "--port", help="Port to use",
                        type=int, default=15000, required=True)
    parser.add_argument("-c", "--certificate", help="Certificate path",
                        type=str, required=True)
    params = parser.parse_args()

    handler: OpenSSLCertificate = OpenSSLCertificate(cert_path=params.certificate)
    akid = handler.get_authority_key_identifier()

    service: MulticastService = MulticastService(interface=params.interface,
                                                 port=params.port,
                                                 group_postfix=bytes_2_multicast_postfix(akid),
                                                 authentication_callback=cbma_impl)

    signal.signal(signal.SIGINT, lambda *_: service.stop())

    service.start()
