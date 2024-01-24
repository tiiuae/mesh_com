import sys
import argparse
import subprocess
import logging

from typing import List
from glob import glob

from setup_cbma import cbma, cbma_processes
from cbma.tools.utils import batman


logger: logging.Logger = logging.getLogger("main")


def setup_logger() -> None:
    global logger

    logger.setLevel(logging.INFO)
    formatter = logging.Formatter(
        f'[%(asctime)s] [main] %(levelname)s %(message)s'
    )
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO)
    console_handler.setFormatter(formatter)

    logger.addHandler(console_handler)


def shutdown_interface(interface_name):
    subprocess.run(["ip", "link", "set", interface_name, "down"],
                   capture_output=True)


def destroy_interface(interface_name):
    subprocess.run(["ip", "link", "delete", interface_name],
                   capture_output=True)


def delete_ebtables_rules():
    command_ebtables = ["ebtables", "-t", "nat", "-L", "OUTPUT"]
    command_sed = ["sed", "-n", "/OUTPUT/,/^$/{/^--/p}"]
    command_xargs = ["xargs", "ebtables", "-t", "nat", "-D", "OUTPUT"]

    proc_ebtables = subprocess.Popen(command_ebtables, stdout=subprocess.PIPE)
    proc_sed = subprocess.Popen(
        command_sed, stdin=proc_ebtables.stdout, stdout=subprocess.PIPE
    )
    proc_xargs = subprocess.Popen(command_xargs, stdin=proc_sed.stdout,
                                  stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL)
    proc_xargs.wait()

    subprocess.run(["ebtables", "--delete", "FORWARD",
                    "--logical-in", "br-upper", "--jump", "ACCEPT"],
                   capture_output=True)
    subprocess.run(["ebtables", "--delete", "FORWARD",
                    "--logical-out", "br-upper", "--jump", "ACCEPT"],
                   capture_output=True)


def delete_macsec_links():
    cmd = "awk -F/ '/macsec/{$0=FILENAME; system(\"ip link delete \" $5)}' /sys/class/net/*/uevent"
    subprocess.run(cmd, shell=True, capture_output=True)


def cleanup_cbma(bat_iface: str) -> None:
    shutdown_interface(bat_iface)
    delete_ebtables_rules()
    delete_macsec_links()
    destroy_interface(bat_iface)


def stop_cbma(bat_iface: str):
    try:
        logger.info("Stopping CBMA...")

        for process in cbma_processes:
            try:
                if process.is_alive():
                    process.terminate()
            except Exception as e:
                logger.error(f"Terminating process error: {e}")

        for process in cbma_processes:
            try:
                if process.is_alive():
                    process.join(timeout=1.0)
                    process.kill()
            except Exception as e:
                logger.error(f"Killing process error: {e}")

        logger.debug("CBMA processes terminated, continue cleanup...")
        cleanup_cbma(bat_iface)
        logger.debug("CBMA cleanup finished")

    except Exception as e:
        logger.error(f"Stop CBMA error: {e}")


def main(interfaces: List [str],
         port: int,
         bat_iface: str,
         cert_folder: str,
         cert_chain: str) -> None:

    cleanup_cbma(bat_iface)

    batman(bat_iface)

    for iface in interfaces:
        try:
            with open(f"/sys/class/net/{iface}/phy80211/index") as f:
                index = f.read()
                wpa_supplicant_control_path = f"/var/run/wpa_supplicant_id{index}/{iface}"
        except FileNotFoundError:
            wpa_supplicant_control_path = False

        process = cbma(
            "lower",
            iface,
            port,
            bat_iface,
            cert_folder,
            cert_chain,
            "off",
            wpa_supplicant_control_path
        )
        cbma_processes.append(process)

    for process in cbma_processes:
        if process.is_alive():
            process.join()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="CBMA test setting")
    parser.add_argument(
        "-i",
        "--interfaces",
        default="all",
        nargs="+",
        help="Interfaces to launch CBMA into. Accepts multiple values",
        required=False
    )
    parser.add_argument(
        "-p",
        "--port",
        default=15001,
        type=int,
        help="Name of the batman interface to create",
        required=False
    )
    parser.add_argument(
        "-b",
        "--batman",
        default="bat0",
        help="Name of the batman interface to create",
        required=False
    )
    parser.add_argument(
        "-d",
        "--certdir",
        default="/opt/crypto/ecdsa/birth/filebased",
        help="Folder where birth certs are placed",
        required=False
    )
    parser.add_argument(
        "-c",
        "--certchain",
        default="/opt/mspki/ecdsa/certificate_chain.crt",
        help="Path to certificate chain",
        required=False
    )
    args = parser.parse_args()

    if args.interfaces == "all":
        # TODO - Only matching common interfaces such as wlp1s0, eth1, usb0 and halow1
        interfaces = [i.split('/')[-1] for i in glob("/sys/class/net/[!bdls][tals][!a]*")]
    else:
        interfaces = args.interfaces if isinstance(args.interfaces, List) else [args.interfaces]

    try:
        setup_logger()
        main(interfaces, args.port, args.batman, args.certdir, args.certchain)
    except Exception as e:
        logger.error(e)
    except KeyboardInterrupt:
        logger.info("Interrupting...")
    finally:
        stop_cbma(args.batman)

        logger.info("Exiting")
