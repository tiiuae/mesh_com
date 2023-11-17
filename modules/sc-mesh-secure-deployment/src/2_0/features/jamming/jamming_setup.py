import subprocess
import threading

import netifaces
import sys
import time
import re
import os
import asyncio
import numpy as np
import util
import options
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


async def switch_frequency(args) -> None:
    """
    Change to starting frequency.
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
            await util.switch_frequency(starting_frequency)

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


def check_client_osf_connectivity(args) -> None:
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


def check_osf_interface(args: Options) -> None:
    """
    Check if OSF interface is up.

    param args: Options object containing configuration options.
    """
    num_retries: int = 0
    max_retries: int = 3
    interface_status: bool = is_interface_up(args.osf_interface)

    while not interface_status and num_retries < max_retries:
        time.sleep(10)
        interface_status = is_interface_up(args.osf_interface)
        num_retries += 1
        if num_retries == max_retries:
            logger.error(f"OSF interface failed")
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


def get_device_identity(args) -> None:
    """
    Request device identity
    """
    # Store the current working directory
    current_directory = os.getcwd()

    try:
        # Change the directory to the scripts_path
        os.chdir(args.nats_scripts_path)

        # Define the commands to execute
        command = ["python", "_cli_command_get_identity.py"]

        # Execute the command
        max_retries = 3

        for retry_count in range(max_retries):
            try:
                subprocess.run(command, check=True)
                break  # Break if successful
            except subprocess.CalledProcessError as e:
                if retry_count < max_retries - 1:
                    logger.error(f"Retrying _cli_command_get_identity.py...")
                    time.sleep(2)
                else:
                    logger.error(f"Error executing command {command}: {e}")
                    exit(0)

    finally:
        # Change back to the original directory after executing the commands
        os.chdir(current_directory)


def validate_configuration(args) -> bool:
    """
    Validate that scan interface and scan channels configurations are valid for jamming detection feature.

    :return: A boolean to denote whether the device and parameter configurations are valid.
    """
    valid = True

    # Check that scan interface is set to 20mhz
    try:
        iw_output = os.popen('iw dev').read()
        iw_output = re.sub(r'\s+', ' ', iw_output).split(' ')

        # Extract interface sections from iw_output
        idx_list = [idx - 1 for idx, val in enumerate(iw_output) if val == "Interface"]
        if len(idx_list) > 1:
            idx_list.pop(0)

        # Calculate the start and end indices for interface sections
        start_indices = [0] + idx_list
        end_indices = idx_list + ([len(iw_output)] if idx_list[-1] != len(iw_output) else [])

        # Use zip to create pairs of start and end indices, and extract interface sections
        iw_interfaces = [iw_output[start:end] for start, end in zip(start_indices, end_indices)]

        # Get scan interface channel width
        for interface_list in iw_interfaces:
            if args.scan_interface in interface_list:
                channel_width_index = interface_list.index("width:") + 1
                channel_width = re.sub("[^0-9]", "", interface_list[channel_width_index]).split()[0]
                if channel_width != "20":
                    logger.info(f"{args.scan_interface} interface channel width must be set to 20 MHz.")
                    valid = False
                break
            else:
                logger.info(f"No {args.scan_interface} interface")
    except Exception as e:
        logger.error(f"Exception: {e}")

    # Check that list of channels to scan does not include any DFS channels
    if not all(channel in options.VALID_CHANNELS for channel in args.channels5):
        logger.info("5 GHz channels must be of the following: (36,40,44,48,149,153,157,161)")
        valid = False

    # Return validity after all checks performed
    return valid


async def main():
    # Create Options instance
    args = Options()

    # Get device identity
    logger.info("1. Get device identity")
    get_device_identity(args)
    await asyncio.sleep(3)

    # Switch to starting frequency
    logger.info("2. Switch to starting frequency")
    mesh_freq = util.get_mesh_freq()
    if mesh_freq == np.nan or util.map_channel_to_freq(args.starting_channel) != mesh_freq:
        await switch_frequency(args)

    # Validate configuration parameters
    logger.info("3. Validate configuration parameters")
    if not validate_configuration(args):
        raise Exception("Configuration validation failed. Please adjust the configurations according to the above to run the jamming avoidance feature.")

    # Check interface for OSF
    logger.info("4. Check if osf interface is up")
    check_osf_interface(args)

    # Get IPv6 address of osf interface
    logger.info("5. Get ipv6 of osf interface")
    osf_ipv6_address = util.get_ipv6_addr(args.osf_interface)
    if osf_ipv6_address is None:
        raise ValueError("IPv6 address of the tun0 interface is not available.")

    # If the current node is a client, check ipv6 connectivity with server
    if args.jamming_osf_orchestrator != osf_ipv6_address:
        logger.info("6. Check client osf connectivity with server")
        check_client_osf_connectivity(args)

    # Start jamming-related scripts
    start_jamming_scripts(args, osf_ipv6_address)


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.close()
