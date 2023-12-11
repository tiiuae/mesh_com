import logging
import sys
import os


class CustomLogger:
    def __init__(self, role):
        self.role = role
        self._ensure_log_directory_exists()
        self.logger = self._setup_logger()

    @staticmethod
    def _ensure_log_directory_exists():
        if not os.path.exists("logs"):
            os.makedirs("logs")

    def _setup_logger(self):
        # Create a custom logger
        logger = logging.getLogger(f"{self.role}")
        logger.setLevel(logging.INFO)

        # Create file handler
        file_path = os.path.join("logs", f'{self.role}.log')
        file_handler = logging.FileHandler(file_path, encoding='utf-8')
        file_handler.setLevel(logging.INFO)

        # Create console handler
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(logging.INFO)

        # Create a formatter
        formatter = logging.Formatter(
            f'[%(asctime)s] [{self.role}] %(levelname)s %(message)s'
        )
        file_handler.setFormatter(formatter)
        console_handler.setFormatter(formatter)

        # Add the handlers to the logger
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)

        return logger

    def get_logger(self):
        return self.logger


# Usage example:
# logger_instance = AuthLogger("Server")
# logger = logger_instance.get_logger()
# logger.info("This is an info message!")


