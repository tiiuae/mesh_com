import subprocess
import threading

import netifaces
import sys
import time
import numpy as np
import util
from options import Options
from log_config import logger


def is_interface_up(interface) -> bool:
    """
    Check if a network interface is up.

    :param interface: The name of the network interface to check.
    :return: True if the interface is up, False otherwise.
    """
    available_interfaces = netifaces.interfaces()
    return interface in available_interfaces


def switch_frequency(args) -> None:
    """
    Change the mesh frequency to the starting frequency.
    """
    # Set max number of reties to perform the switch frequency method in case it fails
    max_retries: int = 3
    num_switch_freq_retries: int = 0
    switch_successful = False

    # Initialize switch frequency variables
    starting_frequency = str(util.map_channel_to_freq(args.starting_channel))

    while not switch_successful:
        try:
            # Execute switch frequency function
            util.switch_frequency(starting_frequency)

            # Validate outcome of switch frequency process
            mesh_freq = util.get_mesh_freq()
            if mesh_freq != int(starting_frequency):
                if num_switch_freq_retries < max_retries:
                    logger.info("Frequency switch unsuccessful... retrying")
                    num_switch_freq_retries += 1
                    time.sleep(1)
                    continue
                else:
                    logger.info("Cannot set node to starting frequency. Maximum retries reached.")
                    sys.exit(1)
            else:
                logger.info("Frequency switch successful")
                switch_successful = True

        except Exception as e:
            logger.error(f"{str(e)}")
            if num_switch_freq_retries < max_retries:
                logger.info("Retrying...")
                num_switch_freq_retries += 1
                time.sleep(1)  # Adjust sleep duration if needed
            else:
                logger.info("Cannot set node to starting frequency. Maximum retries reached.")
                sys.exit(1)


def check_osf_connectivity(args) -> None:
    """
    Check IPv6 connectivity with a remote server.

    :param args: The jamming avoidance variables.
    """
    max_retries: int = 3
    number_retries: int = 0
    while True:
        command = ['ping6', '-c', '1', args.jamming_osf_orchestrator]
        try:
            subprocess.check_output(command, text=True)
            break
        except subprocess.CalledProcessError:
            number_retries += 1
            logger.info(f"Ping failed... retry {number_retries}")

        if number_retries == max_retries:
            logger.info("Server unreachable")
            sys.exit(1)


def setup_osf_interface(args: Options) -> None:
    """
    Set up the OSF interface using a tun0 tunnel.

    param args: Options object containing configuration options.
    """
    interface_status: bool = is_interface_up(args.osf_interface)
    if not interface_status:
        try:
            setup_osf_interface_command = ["start_tunslip6.sh", "/dev/nrf0", f"{args.osf_interface}"]
            util.run_command(setup_osf_interface_command, "Failed to set up OSF interface")
        except Exception as e:
            logger.error(f"OSF interface failed: {e}")
            sys.exit(1)


def start_server(args) -> None:
    """
    Start jamming server script.

    param args: Configuration options.
    """
    # Define server file name
    server_file = "jamming_server_fsm.py"
    # Kill any running instance of server file before starting it
    if util.is_process_running(server_file):
        util.kill_process_by_pid(server_file)
    # Start jamming server script
    logger.info(f"Started {server_file}")
    util.run_command(["python", f"{server_file}"], 'Failed to run jamming_server_fsm file')


def start_client(args) -> None:
    """
    Start jamming client script.

    param args: Configuration options.
    """
    # Define client file name
    client_file = "jamming_client_fsm.py"
    # Kill any running instance of server file before starting it
    if util.is_process_running(client_file):
        util.kill_process_by_pid(client_file)
    # Start jamming client script
    logger.info(f"Started {client_file}")
    util.run_command(["python", f"{client_file}"], 'Failed to run jamming_client_fsm file')


def start_jamming_scripts(args, osf_ipv6_address) -> None:
    """
    Start jamming-related scripts based on configuration.

    param args: Configuration options.
    param osf_ipv6_address: IPv6 address associated with the OSF interface.
    """
    # Compare jamming_osf_orchestrator with osf_ipv6_address
    try:
        if args.jamming_osf_orchestrator == osf_ipv6_address:
            server_thread = threading.Thread(target=start_server, args=(args,))
            client_thread = threading.Thread(target=start_client, args=(args,))

            # Start server and client scripts
            server_thread.start()
            client_thread.start()

            # Wait for the threads to finish
            server_thread.join()
            client_thread.join()
        else:
            # Start client script
            client_thread = threading.Thread(target=start_client, args=(args,))
            client_thread.start()
            client_thread.join()
    except Exception as e:
        logger.info(f"Error starting jamming server/client scripts: {e}")
        raise Exception(e)

def main():
    # Create Options instance
    args = Options()

    # Set up tun0 interface for OSF
    logger.info("1. Set up tun0 osf interface")
    setup_osf_interface(args)

    # Switch to starting frequency
    logger.info("2. Switch to starting frequency")
    mesh_freq = util.get_mesh_freq()
    if mesh_freq == np.nan or util.map_channel_to_freq(args.starting_channel) != mesh_freq:
        switch_frequency(args)

    # Get the IPv6 address of the tun0 interface
    logger.info("3. Get ipv6 of tun0")
    osf_ipv6_address = util.get_ipv6_addr(args.osf_interface)
    if osf_ipv6_address is None:
        raise ValueError("IPv6 address of the tun0 interface is not available.")

    # If the current node is a client, check ipv6 connectivity with server
    logger.info("4. Check connectivity")
    if args.jamming_osf_orchestrator != osf_ipv6_address:
        check_osf_connectivity(args)

    # Start jamming-related scripts
    start_jamming_scripts(args, osf_ipv6_address)


if __name__ == "__main__":
    main()
