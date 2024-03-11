import sys
import argparse

from glob import glob

from models.certificates import CBMACertificates
from controller import CBMAController
from utils.logging import get_logger


logger = get_logger(log_dir='.')


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="CBMA standalone parameters")
    parser.add_argument(
        "-i",
        "--interfaces",
        default="all",
        nargs="+",
        help="Interfaces to launch CBMA into - Accepts multiple values",
        required=False
    )
    parser.add_argument(
        "-p",
        "--port",
        default=1500,
        type=int,
        help="Port number to use for both UDP and TCP connections",
        required=False
    )
    parser.add_argument(
        "-b",
        "--batman",
        default="bat0",
        help="Name of the batman interface to use",
        required=False
    )
    parser.add_argument(
        "-u",
        "--upper",
        default=False,
        action=argparse.BooleanOptionalAction,
        help="Launches interfaces in upper-CBMA",
        required=False
    )
    parser.add_argument(
        "-c",
        "--certdir",
        default="./certificates/ecdsa",
        help="Folder where interface certificates are placed",
        required=False
    )
    args = parser.parse_args()

    if args.interfaces == "all":
        # HACK - Only matching common interfaces such as wlp1s0, eth1, usb0 and halow1
        interfaces = [i.split('/')[-1] for i in glob("/sys/class/net/[!bdls][tals][!a]*")]
    else:
        interfaces = args.interfaces if isinstance(args.interfaces, list) else [args.interfaces]

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

    is_upper = args.upper or any("bat" in i and glob(f"/sys/class/net/*/upper_{i}") for i in interfaces)
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

    try:
        logger.info(f"Adding {interfaces} to the CBMAController")
        for iface in interfaces:
            controller.add_interface(iface)
        controller.join()
    except Exception as e:
        logger.error(e)
    except KeyboardInterrupt:
        logger.info("Interrupting...")
    finally:
        controller.stop()

    logger.info("Exiting")
