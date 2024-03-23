import logging
import sys
import os
import errno

from inspect import stack


LOG_DIR: str = '/var/log/cbma'
LOG_LEVEL: int = logging.INFO
LOG_FORMAT: str = '[%(asctime)s] [%(levelname)-7s] {} %(message)s'
LOG_IFACE_ALIGNMENT: int = 6  # Longest iface one would be 6 chars for wlp1s0
LOG_NAME_ALIGNMENT: int = 31  # From secure_socket.secure_connection
LOG_FUNC_ALIGNMENT: int = 33  # From __create_socket_connection_object

LOG_DIR = os.environ.get('LOG_DIR', LOG_DIR)
LOG_LEVEL = logging.getLevelName(os.environ['LOG_LEVEL']) if os.getenv('LOG_LEVEL') else LOG_LEVEL

_logger_names: list[str] = []


def __get_immediate_caller_filename() -> str:
    # Skipping first 2 frames as they contain get_logger and this function name
    for frame in stack()[2:]:
        if frame.filename.startswith('/'):
            filename: str = frame.filename
            break
    else:
        filename: str = __file__

    basename: str = os.path.basename(filename)
    basename_no_ext: str = basename.split(os.path.extsep)[0]

    pathname_list: list[str] = filename.split(os.path.sep)[:-1]
    path_list: list[str] = __file__.split(os.path.sep)[:-2]

    pathname_common_trimmed_set: set[str] = set(pathname_list).difference(path_list)
    pathname_one_parent_list: list[str] = list(pathname_common_trimmed_set)[-1:]

    pathname_one_parent_list.append(basename_no_ext)

    filename_dotted: str = os.path.extsep.join(pathname_one_parent_list)

    return filename_dotted


def __get_formatter(log_level: int, name: str) -> logging.Formatter:
    if log_level == logging.DEBUG:
        log_format = LOG_FORMAT.format('{} [%%(funcName)-%is]' % LOG_FUNC_ALIGNMENT)
    else:
        log_format = LOG_FORMAT

    return logging.Formatter(log_format.format(name))


def __add_console_handler(logger: logging.Logger, name: str, name_prefix: str) -> bool:
    if name_prefix:
        name_format = ('[{:<%i}] [{:<%i}]' % (LOG_IFACE_ALIGNMENT, LOG_NAME_ALIGNMENT)
                       ).format(name_prefix, name)
        name = os.path.extsep.join([name_prefix, name])
    else:
        name_format = ('{:<%i} [{:<%i}]' % (LOG_IFACE_ALIGNMENT + 2, LOG_NAME_ALIGNMENT)
                       ).format(' ', name)

    # Prevent duplicated log entries if creating multiple loggers with the same name
    # Note: FileHandler inherits from StreamHandler -> we check that handler isn't FileHandler
    if any([not isinstance(handler, logging.FileHandler) for handler in logger.handlers]):
        logger.debug(f"Logger '{name}' already has a console handler")
        return False

    log_level = logger.getEffectiveLevel()
    formatter = __get_formatter(log_level, name_format)
    console_handler = logging.StreamHandler(sys.stdout)

    console_handler.set_name(name)
    console_handler.setLevel(log_level)
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)

    _logger_names.append(name)

    logger.debug(f"Successfully added '{name}' console handler")
    return True


def __add_file_handler(logger: logging.Logger, log_path: str) -> bool:
    name = os.path.basename(log_path).removesuffix(os.path.extsep + 'log')
    log_dir = os.path.dirname(log_path)

    log_dir_real = os.path.realpath(os.path.expanduser(log_dir))
    LOG_DIR_real = os.path.realpath(os.path.expanduser(LOG_DIR))
    if log_dir_real == LOG_DIR_real:
        name_format = ('{:<%i} [{:<%i}]' % (LOG_IFACE_ALIGNMENT + 2, LOG_NAME_ALIGNMENT)).format(' ', name)
    else:
        subdirs_list = log_dir_real.removeprefix(LOG_DIR_real + os.path.sep).split(os.path.sep)
        subdirs_list.append(name)
        name = os.path.extsep.join(subdirs_list)
        name_prefix = subdirs_list.pop(0)
        name_format = ('[{:<%i}] [{:<%i}]' % (LOG_IFACE_ALIGNMENT, LOG_NAME_ALIGNMENT)).format(name_prefix, os.path.extsep.join(subdirs_list))

    # Prevent duplicated log entries if creating multiple loggers with the same name
    if handlers := [h for h in logger.handlers if isinstance(h, logging.FileHandler)]:
        log_path = handlers[0].baseFilename.removeprefix(LOG_DIR_real + os.path.sep)
        name = name.removeprefix(os.path.basename(log_dir) + os.path.extsep)
        log_dir = os.path.join(log_dir, os.pardir)
        log_path = os.path.normpath(os.path.join(log_dir, log_path))
        logger.debug(f"Logger '{name}' already has a file handler at '{log_path}'")
        return False

    # Exception should be handled in caller
    os.makedirs(log_dir, exist_ok=True)

    log_level = logger.getEffectiveLevel()
    formatter = __get_formatter(log_level, name_format)
    file_handler = logging.FileHandler(log_path, encoding='utf-8')

    file_handler.set_name(name)
    file_handler.setLevel(log_level)
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    logger.debug(f"Successfully added '{name}' file handler at '{log_path}'")
    return True


def __remove_console_handler(logger: logging.Logger, name: str) -> bool:
    for handler in logger.handlers:
        # Note: FileHandler inherits from StreamHandler -> we check that handler isn't FileHandler
        if not isinstance(handler, logging.FileHandler) and handler.name == name:
            logger.debug(f"Removing logging console handler '{name}'")
            logger.removeHandler(handler)
            _logger_names.remove(name)
            return True

    logger.error(f"No '{name}' logging console handlers found")
    return False


# TODO - Use logger.config.fileConfig with a logger.conf
def get_logger(name: str = '', log_dir: str = '') -> logging.Logger:
    if not name:
        name = __get_immediate_caller_filename()

    if name_prefix := log_dir.replace(os.path.sep, os.path.extsep).lstrip(os.path.extsep):
        logger_name = os.path.extsep.join([name_prefix, name])
    else:
        logger_name = name

    logger = logging.getLogger(logger_name)
    logger.setLevel(LOG_LEVEL)
    __add_console_handler(logger, name, name_prefix)

    if log_dir:
        log_path = os.path.normpath(os.path.join(LOG_DIR, log_dir, f"{name}{os.path.extsep}log"))
        try:
            __add_file_handler(logger, log_path)
        except Exception as e:
            logger.error(f"Exception when creating '{log_path}': {e}")
            logger.warning(f"Continuing without '{logger_name}' logging to file")

    return logger


def setup_global_file_logging(log_dir: str = '') -> bool:
    '''
    This function loops over all loggers and adds file handlers to them.
    NOTE: Only supposed to be used at the beginning of every process after spawning.
    '''
    if not _logger_names:
        raise Exception('No loggers found')

    LOG_DIR_parent = os.path.dirname(LOG_DIR.rstrip(os.path.sep))
    log_dir = os.path.normpath(os.path.join(LOG_DIR, log_dir))
    name_prefix = os.path.basename(log_dir) + os.path.extsep

    result = True
    for name in _logger_names.copy():
        logger = logging.getLogger(name)

        log_path = os.path.join(log_dir, f"{name}{os.path.extsep}log")
        try:
            if not __add_file_handler(logger, log_path):
                continue

            old_name = name
            new_name = name.removeprefix(name_prefix)
            if not __remove_console_handler(logger, old_name) or \
               not __add_console_handler(logger, new_name, name_prefix[:-1]):
                result = False
        except Exception as e:
            result = False
            logger.error(f"Exception when creating '{log_path}': {e}")

            if isinstance(e, OSError) and e.errno == errno.EACCES and e.filename == LOG_DIR_parent:
                logger.warning('Continuing without logging to file')
                break
            logger.warning(f"Continuing without logging to '{log_path}'")

    return result
