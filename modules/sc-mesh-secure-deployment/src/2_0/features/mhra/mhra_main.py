from multiradio_manager import MultiRadioManager
from monitor_controller import MonitorController
from config import create_default_config, load_config
import time
import logging

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,   # DEBUG, INFO, WARNING, ERROR, and CRITICAL
    format='%(filename)s - %(levelname)s - %(message)s',
)

def main():
    config_file_path = '/opt/mhra_config.yaml'

    # Create the default configuration file if it doesn't exist
    create_default_config(config_file_path)

    # Load the configuration
    config = load_config(config_file_path)

    # Get radio interfaces from configuration
    # Currently single always_on_radio and single on_deman_radio is supported.
    always_on_radio = config['radio_config']['always_on_radios'][0]
    on_demand_radio = config['radio_config']['on_demand_radios'][0]

    # Initialize MultiRadioManager
    multi_radio_manager = MultiRadioManager()

    logging.info(f"Checking for {always_on_radio} interface...")

    # Wait for the always-on radio interface to be up
    if not multi_radio_manager.wait_for_interface(always_on_radio):
        logging.error(f"Interface {always_on_radio} did not come up within the timeout period.")
        return

    logging.info(f"{always_on_radio} interface is up.")

    time.sleep(1) 

    logging.info(f"Checking for {on_demand_radio} interface...")

    # Wait for the on-demand radio interface to be up
    if not multi_radio_manager.wait_for_interface(on_demand_radio):
        logging.error(f"Interface {on_demand_radio} is not available to proceed further!")
        return

    logging.info(f"{on_demand_radio} interface is up.")

    # Wait for the always-on radio to have mesh enabled, with a timeout
    max_wait_time = 600  # Maximum wait time of 10 minutes
    start_time = time.time()

    while not multi_radio_manager.is_mesh_mode_enabled(always_on_radio, 0):
        logging.warning(f"Mesh mode is not enabled for {always_on_radio}. Waiting...")
        time.sleep(10)  # Wait for 10 seconds before checking again
        if time.time() - start_time > max_wait_time:
            logging.error(f"Mesh mode did not enable for {always_on_radio} within the timeout period.")
            return

    logging.info(f"Mesh mode is enabled for {always_on_radio}.")

    # Disable mesh mode on the on-demand radio if it is enabled
    if multi_radio_manager.is_mesh_mode_enabled(on_demand_radio, 1):
        logging.info(f"Mesh mode is enabled for {on_demand_radio}. Disabling it...")
        
        # TBD
        # Routing Override may be required if this device joined ongoing mesh and traffic is on-going

        multi_radio_manager.stop_mesh(on_demand_radio, 1)
        logging.info(f"Mesh mode disabled for {on_demand_radio}.")

    # Initialize and start monitoring using MonitorController
    monitor_controller = MonitorController(multi_radio_manager, config, always_on_radio, on_demand_radio)
    monitor_controller.start_always_on_monitoring()

if __name__ == "__main__":
    main()
