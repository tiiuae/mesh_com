import subprocess
import netifaces
import os
import re
import sys
import time
import numpy as np
import util
from options import Options


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

    print(f"Moving to {starting_frequency} MHz ...\n") if args.debug else None
    while not switch_successful:
        try:
            # Run commands
            cmd_rmv_ip = "ifconfig " + args.mesh_interface + " 0"
            cmd_interface_down = "ifconfig " + args.mesh_interface + " down"
            util.run_command(cmd_rmv_ip, 'Failed to set interface 0')
            util.run_command(cmd_interface_down, 'Failed to set interface down')

            # If wpa_supplicant is running, kill it before restarting
            if util.is_process_running('wpa_supplicant'):
                util.run_command('killall wpa_supplicant', 'Failed to kill wpa_supplicant')
                time.sleep(10)

            # Remove mesh interface file to avoid errors when reinitializing interface
            interface_file = '/var/run/wpa_supplicant/' + args.mesh_interface
            if os.path.exists(interface_file):
                os.remove(interface_file)

            # Read and check wpa supplicant config
            conf = util.read_file('/var/run/wpa_supplicant-11s.conf', 'Failed to read wpa supplicant config')
            if conf is None:
                print("Error: wpa supplicant config is None. Aborting.") if args.debug else None
                return

            # Edit wpa supplicant config with new mesh freq
            conf = re.sub(r'frequency=\d*', f'frequency={starting_frequency}', conf)

            # Write edited config back to file
            util.write_file('/var/run/wpa_supplicant-11s.conf', conf, 'Failed to write wpa supplicant config')

            # Restart wpa supplicant
            cmd_restart_supplicant = 'wpa_supplicant -Dnl80211 -i' + args.mesh_interface + ' -c /var/run/wpa_supplicant-11s.conf -B'
            util.run_command(cmd_restart_supplicant, 'Failed to restart wpa supplicant')
            time.sleep(4)
            util.run_command('iw dev', 'Failed to run iw dev')

            # Validate outcome of switch frequency process
            mesh_freq = util.get_mesh_freq()
            if mesh_freq != int(starting_frequency):
                if num_switch_freq_retries < max_retries:
                    print("Frequency switch unsuccessful... retrying") if args.debug else None
                    num_switch_freq_retries += 1
                    time.sleep(1)
                    continue
                else:
                    print("Error: Cannot set node to starting frequency. Maximum retries reached.") if args.debug else None
                    sys.exit(1)
            else:
                print("Frequency switch successful") if args.debug else None
                switch_successful = True

        except Exception as e:
            print(f"An error occurred: {str(e)}") if args.debug else None
            if num_switch_freq_retries < max_retries:
                print("Retrying after error...") if args.debug else None
                num_switch_freq_retries += 1
                time.sleep(1)  # Adjust sleep duration if needed
            else:
                print("Error: Cannot set node to starting frequency. Maximum retries reached.") if args.debug else None
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
            print(f"Ping failed... retry {number_retries}") if args.debug else None

        if number_retries == max_retries:
            print("Server unreachable") if args.debug else None
            sys.exit(1)


def setup_osf_interface(args: Options) -> None:
    """
    Set up the OSF interface using a tun0 tunnel.

    param args: Options object containing configuration options.
    """
    interface_status: bool = is_interface_up(args.osf_interface)
    if not interface_status:
        try:
            setup_osf_interface_command = 'start_tunslip6.sh /dev/nrf0 ' + args.osf_interface
            util.run_command(setup_osf_interface_command, "Failed to set up OSF interface")
        except Exception as e:
            print(f"OSF interface failed: {e}") if args.debug else None
            sys.exit(1)


def start_jamming_scripts(args, osf_ipv6_address):
    """
    Start jamming-related scripts based on configuration.

    param args: Configuration options.
    param osf_ipv6_address: IPv6 address associated with the OSF interface.
    """
    # Compare jamming_osf_orchestrator with osf_ipv6_address
    if args.jamming_osf_orchestrator == osf_ipv6_address:
        # Execute server.py
        print("5. jamming_server_fsm.py") if args.debug else None
        with open('jamming_server_fsm.log', 'w') as log_file:
            subprocess.Popen(['python', 'jamming_server_fsm.py'], stdout=log_file, stderr=subprocess.STDOUT)
        print("6. jamming_client_fsm.py") if args.debug else None
        util.run_command('python jamming_client_fsm.py', 'Failed to run jamming_client_fsm file')
    else:
        # Execute client.py
        print("5. jamming_client_fsm.py") if args.debug else None
        util.run_command('python jamming_client_fsm.py', 'Failed to run jamming_client_fsm file')


def main():
    args = Options()

    # Set up tun0 interface for OSF
    print("1. set up tun0 osf interface") if args.debug else None
    setup_osf_interface(args)

    # Switch to starting frequency
    print("2. switch to starting frequency") if args.debug else None
    mesh_freq = util.get_mesh_freq()
    if mesh_freq == np.nan or util.map_channel_to_freq(args.starting_channel) != mesh_freq:
        switch_frequency(args)

    # Get the IPv6 address of the tun0 interface
    print("3. get ipv6 of tun0") if args.debug else None
    osf_ipv6_address = util.get_ipv6_addr(args.osf_interface)
    if osf_ipv6_address is None:
        raise ValueError("IPv6 address of the tun0 interface is not available.")

    # If the current node is a client, check ipv6 connectivity with server
    print("4. check connectivity") if args.debug else None
    if args.jamming_osf_orchestrator != osf_ipv6_address:
        check_osf_connectivity(args)

    # Start jamming-related scripts
    start_jamming_scripts(args, osf_ipv6_address)


if __name__ == "__main__":
    main()
