import sys
import os
import time
import pandas as pd
import threading

path_to_decision_engine = os.path.dirname(__file__) # Path to dir containing this script
sys.path.insert(0, path_to_decision_engine)
sys.path.append(f'{path_to_decision_engine}/..')

from cbma.tools.custom_logger import CustomLogger
logger = CustomLogger("quarantine").get_logger()

# ************************************************************ Note ********************************************************************
# The code currently only has placeholders to call the blacklisting and remove from blacklist functionality from the comms controller
# These comms controller functions will actually deal with removing the malicious node's access to the network, and actions for CRL updates.
# Once available, these functions should be called in the placeholders in the start_quarantine and end_quarantine methods.
# **************************************************************************************************************************************

class Quarantine:
    """
    A class to manage the quarantine of malicious nodes.

    Attributes:
    dir (str): Directory where the local blacklist file is stored.
    file_name (str): Name of the local blacklist CSV file.
    file_path (str): Full path to the local blacklist file.
    mal_mac (str): MAC address of the malicious node.
    mal_ip (str): IP address of the malicious node.
    quarantine_period (int): Duration of the quarantine in seconds.
    blacklist_lock (threading.Lock): Lock for synchronizing access to the local blacklist file.
    blocked_ips (set): Set of IPs currently blocked.
    debug (bool): Flag to enable debug logging.

    Methods:
    start_quarantine(): Adds node to blacklist and starts quarantine.
    add_to_blacklist(): Adds node to local blacklist file.
    end_quarantine(): Removes node from blacklist and ends its quarantine.
    remove_from_blacklist(): Removes node from local blacklist file.
    """
    def __init__(self, mal_mac, mal_ip, quarantine_period, blacklist_lock, blocked_ips, blacklist_dir = './blacklist', blacklist_filename = 'blacklist.csv', debug=False):
        """
        Constructs all the necessary attributes for the Quarantine object.

        Parameters:
        mal_mac (str): MAC address of the malicious node.
        mal_ip (str): IP address of the malicious node.
        quarantine_period (int): Duration of the quarantine in seconds.
        blacklist_lock (threading.Lock): Lock for synchronizing access to the blacklist file.
        blocked_ips (set): Set of IPs currently blocked.
        blacklist_dir (str, optional): Directory where the blacklist file is stored. Defaults to './blacklist'.
        blacklist_filename (str, optional): Name of the local blacklist CSV file. Defaults to 'blacklist.csv'.
        debug (bool, optional): Flag to enable debug logging. Defaults to False.
        """
        self.dir = blacklist_dir
        self.file_name = blacklist_filename
        self.file_path = os.path.join(self.dir, self.file_name)
        self.mal_mac = mal_mac
        self.mal_ip = mal_ip
        self.quarantine_period = quarantine_period
        self.blacklist_lock = blacklist_lock
        self.blocked_ips = blocked_ips
        self.debug = debug
    def start_quarantine(self):
        """
        Adds the node to the blacklist and starts a timer for the quarantine period.

        There is a placeholder to call the comms controller blacklist functionality.
        At the end of the quarantine period, it automatically calls the end_quarantine method.

        Returns:
        threading.Timer: A timer thread that counts down the quarantine period.
        """
        self.add_to_blacklist()
        logger.info(f'Starting quarantine for node {self.mal_ip}')
        # Placeholder to call comms controller blacklisting functionality
        # Start timer thread that calls end_quarantine function once quarantine_period is over
        quarantine_timer = threading.Timer(self.quarantine_period, self.end_quarantine)
        quarantine_timer.start()
        return quarantine_timer

    def add_to_blacklist(self):
        """
        Adds the malicious node to the local blacklist file.

        The node's information is appended to the CSV file, creating the file if it doesn't exist.
        """
        # Create the directory if it doesn't exist
        if not os.path.exists(self.dir):
            os.makedirs(self.dir)

        # Initialize df
        data = {'MAC': [self.mal_mac], 'IP': [self.mal_ip], 'Quarantine_Period': [self.quarantine_period]}
        df = pd.DataFrame(data)
        # Add current timestamp as start timestamp for each row in df
        df['Start_timestamp'] = time.time()
        df['End_timestamp'] = df['Start_timestamp'] + df['Quarantine_Period']
        if self.debug:
            logger.info(f'df to add:\n{df}')
        # Append data to the CSV file
        # If the file doesn't exist, write the header row (if required)
        if not os.path.isfile(self.file_path):
            with self.blacklist_lock:
                df.to_csv(self.file_path, index=False, header=True)
        else:
            with self.blacklist_lock:
                df.to_csv(self.file_path, index=False, header=False, mode='a')

    def end_quarantine(self):
        """
        Ends the quarantine for the node by removing it from the blacklist.

        There is a placeholder to call the comms controller functionality to remove a node from being blacklisted.
        Updates the set of blocked IPs and calls the method to remove the node from the local blacklist file.
        """
        self.blocked_ips.discard(self.mal_ip)
        self.remove_from_blacklist()
        logger.info(f'Ending quarantine for node {self.mal_ip}')
        # Placeholder to call comms controller remove from blacklist functionality

    def remove_from_blacklist(self):
        """
        Removes the node from the local blacklist file.

        Reads the current blacklist, removes the node's entry, and writes the updated list back to the file.
        """
        with self.blacklist_lock:
            # Read the df from the CSV file
            df = pd.read_csv(self.file_path)
            # Remove row for malicious node
            df = df[df['IP'] != self.mal_ip]
            if self.debug:
                logger.info(f'Blacklist after removing mal node {self.mal_ip}:\n{df}')
            # Save the updated DataFrame back to the original CSV file
            df.to_csv(self.file_path, index=False, header=True, mode='w')