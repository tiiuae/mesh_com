import logging
from logging.handlers import RotatingFileHandler

# Add a rotating file handler for log rotation (max 1 MB, keep 1 backup)
log_handler = RotatingFileHandler("/var/log/jamming_avoidance.log", maxBytes=1e6, backupCount=1)
log_handler.setLevel(logging.INFO)

# Create a formatter for the log messages
log_formatter = logging.Formatter('%(asctime)s - %(levelname)-5s - %(filename)-21s - %(message)s')
log_handler.setFormatter(log_formatter)

# Add the handler to the root loggers
logging.getLogger('').addHandler(log_handler)

# Set the log level for all loggers (including the root logger)
logging.getLogger('').setLevel(logging.INFO)

# Set up a logger with the name of the current module
logger = logging.getLogger(__name__)
