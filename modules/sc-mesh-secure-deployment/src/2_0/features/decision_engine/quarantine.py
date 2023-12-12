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

#TODO: block, unblock, encrypt blacklist
class Quarantine:
    def __init__(self, mal_id, mal_ip, quarantine_period, blacklist_lock, blocked_ips, blacklist_dir = './blacklist', blacklist_filename = 'blacklist.csv', debug=False):
        self.dir = blacklist_dir
        self.file_name = blacklist_filename
        self.file_path = os.path.join(self.dir, self.file_name)
        self.mal_id = mal_id
        self.mal_ip = mal_ip
        self.quarantine_period = quarantine_period
        self.blacklist_lock = blacklist_lock
        self.blocked_ips = blocked_ips
        self.debug = debug
    def start_quarantine(self):
        """
        Adds node to blacklist and starts quarantine
        Calls end_quarantine method automatically once quarantine_period is over

        :return: quarantine_timer thread
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
        Adds node to local blacklist file
        """
        # Create the directory if it doesn't exist
        if not os.path.exists(self.dir):
            os.makedirs(self.dir)

        # Initialize df
        data = {'ID': [self.mal_id], 'IP': [self.mal_ip], 'Quarantine_Period': [self.quarantine_period]}
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
        Removes node from blacklist and ends its quarantine
        """
        self.blocked_ips.discard(self.mal_ip)
        self.remove_from_blacklist()
        logger.info(f'Ending quarantine for node {self.mal_ip}')
        # Placeholder to call comms controller remove from blacklist functionality

    def remove_from_blacklist(self):
        """
        Removes node from local blacklist file
        """
        with self.blacklist_lock:
            # Read the df from the CSV file
            df = pd.read_csv(self.file_path)
            # Remove row for malicious node
            df = df[df['ID'] != self.mal_id]
            if self.debug:
                logger.info(f'Blacklist after removing mal node {self.mal_id}:\n{df}')
            # Save the updated DataFrame back to the original CSV file
            df.to_csv(self.file_path, index=False, header=True, mode='w')