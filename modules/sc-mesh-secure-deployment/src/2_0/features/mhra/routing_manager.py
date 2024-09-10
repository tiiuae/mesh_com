import subprocess
import logging

def override_batman_routing(interface, target_throughput=100):
    """
    Override routing in BATMAN using batctl.

    Parameters:
    interface (str): The interface whose throughput is being adjusted.
    target_throughput (int): The throughput to set for the interface (0 to disable).
    """
    logging.info(f"Overriding BATMAN routing for {interface} with throughput {target_throughput}...")

    try:
        command = f"batctl hardif {interface} to {target_throughput}"
        subprocess.run(command, shell=True, check=True)
        logging.info(f"BATMAN routing successfully overridden for {interface} with throughput {target_throughput}.")
    except subprocess.CalledProcessError as e:
        logging.error(f"Failed to override BATMAN routing for {interface}: {e}")


def restore_batman_routing(interface, default_throughput=0):
    """
    Restore the BATMAN routing by setting the default throughput.

    Parameters:
    interface (str): The interface to restore.
    default_throughput (int): The default throughput to restore (default is 1000).
    """
    logging.info(f"Restoring BATMAN routing for {interface} with default throughput {default_throughput}...")

    try:
        command = f"batctl hardif {interface} to {default_throughput}"
        subprocess.run(command, shell=True, check=True)
        logging.info(f"BATMAN routing restored for {interface} with throughput {default_throughput}.")
    except subprocess.CalledProcessError as e:
        logging.error(f"Failed to restore BATMAN routing for {interface}: {e}")


def add_interface_to_batman(bat_interface, interface):
    """
    Add the given interface to the specified bat0 (BATMAN interface).

    Parameters:
    bat_interface (str): The BATMAN interface (e.g., bat0).
    interface (str): The interface to add to BATMAN (e.g., wlp1s0).
    """
    logging.info(f"Adding interface {interface} to {bat_interface} using batctl...")

    try:
        command = f"batctl {bat_interface} if add {interface}"
        subprocess.run(command, shell=True, check=True)
        logging.info(f"Interface {interface} successfully added to {bat_interface}.")
    except subprocess.CalledProcessError as e:
        logging.error(f"Failed to add interface {interface} to {bat_interface}: {e}")
