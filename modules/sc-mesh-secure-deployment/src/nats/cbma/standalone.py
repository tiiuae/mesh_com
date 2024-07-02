import os
import sys
import argparse

from glob import glob

from models.certificates import CBMACertificates
from controller import CBMAController
from utils.common import run_command_retcode
from utils.networking import get_interface_mac_address
from utils.logging import get_logger


logger = get_logger(log_dir='.')


BATMAN_ROUTING_ALG = 'BATMAN_V'
CBMA_ROOT = os.path.normpath(os.path.dirname(__file__))

def get_interface_locally_administered_mac(interface: str) -> str:
    mac = get_interface_mac_address(interface)
    mac_bytes = bytearray.fromhex(mac.replace(':', ''))
    mac_bytes[0] ^= 0x2  # Locally administered bit
    return mac_bytes.hex(sep=':', bytes_per_sep=1)

def create_batman(batman: str, mac: str) -> None:
    create_batman_str = f"batctl meshif {batman} interface create routing_algo {BATMAN_ROUTING_ALG}"
    set_batman_mac_str = f"ip link set {batman} address {mac}"
    set_batman_up_str = f"ip link set {batman} up"
    for cmd_str in [create_batman_str, set_batman_mac_str, set_batman_up_str]:
        if run_command_retcode(cmd_str.split()):
            sys.exit(255)

def destroy_batman(batman: str) -> None:
    if glob(f"/sys/class/net/{batman}/lower_*"):
        return
    destroy_batman_str = f"ip link del {batman}"
    run_command_retcode(destroy_batman_str.split())

def get_mtu_from_constants_rc(exclude: list[str] = []) -> int:
    mtu = 0
    constants_rc = f"{CBMA_ROOT}/scripts/mess/constants.rc"
    with open(constants_rc, 'r') as f:
        for line in f.readlines():
            if line.startswith('#') or not '=' in line \
               or not ('OVERHEAD' in line or 'HOPEFULLY' in line):
                continue
            for e in exclude:
                if e in line:
                    break
            else:
                try:
                    mtu += int(line.split('=')[-1].strip())
                except ValueError:
                    logger.warning(f"Ignoring '{line.strip()}' for MTU calculation")
    if not mtu:
        logger.error(f"Unable to get MTU value from {constants_rc}")
        sys.exit(255)

    return mtu

def set_interface_mtu(interface: str, mtu: int) -> bool:
    cmd_str = f"ip link set {interface} mtu {mtu}"
    return not run_command_retcode(cmd_str.split())


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='CBMA standalone parameters')
    parser.add_argument(
        '-i',
        '--interfaces',
        default='all',
        nargs='+',
        help='Interfaces to launch CBMA into - Accepts multiple values',
        required=False
    )
    parser.add_argument(
        '-p',
        '--port',
        default=15001,
        type=int,
        help='Port number to use for both UDP and TCP connections',
        required=False
    )
    parser.add_argument(
        '-b',
        '--batman',
        default='bat0',
        help='Name of the batman interface to use',
        required=False
    )
    parser.add_argument(
        '-u',
        '--upper',
        default=False,
        action=argparse.BooleanOptionalAction,
        help='Launches interfaces in upper-CBMA',
        required=False
    )
    parser.add_argument(
        '-c',
        '--certdir',
        default='./certificates/ecdsa',
        help='Folder where interface certificates are placed',
        required=False
    )
    args = parser.parse_args()

    if args.interfaces == 'all':
        # HACK - Only matching common interfaces such as wlp1s0, eth1, usb0 and halow1
        interfaces = [i.split('/')[-1] for i in glob('/sys/class/net/[!bdls][tals][!a]*')]
    else:
        interfaces = args.interfaces if isinstance(args.interfaces, list) else [args.interfaces]

    if not interfaces:
        logger.error("No interfaces found")
        sys.exit(255)

    if args.batman in interfaces:
        logger.error(f"{args.batman} found in both -i {' '.join(interfaces)} and -b {args.batman} flags")
        sys.exit(255)

    cert_dir = f"{args.certdir}/birth/filebased/MAC/"
    key = f"{args.certdir}/birth/filebased/private.key"
    chain = [f"{args.certdir}/filebased.crt",
             f"{args.certdir}/intermediate.crt",
             f"{args.certdir}/root.crt"]
    ca = f"{args.certdir}/ca"
    certificates = CBMACertificates(cert_dir, key, chain, ca)

    is_upper = args.upper or any('bat' in i and glob(f"/sys/class/net/*/upper_{i}") for i in interfaces)

    enable_macsec_encryption = is_upper
    try:
        controller = CBMAController(args.port,
                                    args.batman,
                                    certificates,
                                    is_upper,
                                    enable_macsec_encryption)
    except Exception as e:
        logger.error(f"Exception when creating the CBMAController: {e}")
        sys.exit(255)

    mtu_base = get_mtu_from_constants_rc(exclude=['OVERHEAD'])
    mtu_overhead = get_mtu_from_constants_rc(exclude=['HOPEFULLY'])
    mtu_batman = mtu_base

    if not is_upper:
        mtu_batman += mtu_overhead
        mtu_overhead *= 2

    mtu = mtu_base + mtu_overhead
    for i in interfaces:
        if not set_interface_mtu(i, mtu):
            sys.exit(255)

    if not (existing_batman := f"/sys/class/net/{args.batman}" in glob("/sys/class/net/*")):
        mac = get_interface_locally_administered_mac(interfaces[0])
        create_batman(args.batman, mac)
        set_interface_mtu(args.batman, mtu_batman)
    try:
        logger.info(f"Adding {interfaces} to the CBMAController")
        for iface in interfaces:
            controller.add_interface(iface)
        controller.join()
    except Exception as e:
        logger.error(e)
    except KeyboardInterrupt:
        logger.info('Interrupting...')
    finally:
        controller.stop()
        if not existing_batman:
            destroy_batman(args.batman)

    logger.info('Exiting')
