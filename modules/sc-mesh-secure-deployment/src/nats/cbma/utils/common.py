import subprocess

from typing import Tuple

from . import logging


logger = logging.get_logger()


def run_command(cmd_list: list[str], quiet_on_error: bool = False) -> Tuple[str, str, int]:
    proc = subprocess.run(cmd_list, capture_output=True, text=True)

    proc_stdout = proc.stdout
    proc_stderr = proc.stderr
    cmd_str = ' '.join(cmd_list)

    ret_code = proc.returncode
    if ret_code > 0 and not quiet_on_error:
        logger.error(f"'{cmd_str}' failed with error code {ret_code}")
        logger.error(f"'{cmd_str}' STDOUT: {proc_stdout}")
        logger.error(f"'{cmd_str}' STDERR: {proc_stderr}")
    else:
        logger.debug(f"'{cmd_str}' return code: {ret_code}")
        logger.debug(f"'{cmd_str}' OUTPUT: {proc_stdout}")
        logger.debug(f"'{cmd_str}' STDERR: {proc_stderr}")

    return proc_stdout, proc_stderr, ret_code


# Python bug workaround to get the correct exit code from shell 'exit <code>'
def run_script_bug_workaround(cmd_list: list[str], quiet_on_error: bool = False) -> Tuple[str, str, int]:
    proc_stdout, proc_stderr, ret_code = run_command(cmd_list, quiet_on_error)

    cmd_str = ' '.join(cmd_list)

    if ret_code == 0:
        last_line = proc_stderr.splitlines()[-1]
        if 'exit' in last_line:
            ret_code = int(last_line.split()[-1])
            if ret_code > 0 and not quiet_on_error:
                logger.error(f"'{cmd_str}' failed with error code {ret_code}")
                logger.error(f"'{cmd_str}' STDOUT: {proc_stdout}")
                logger.error(f"'{cmd_str}' STDERR: {proc_stderr}")
        else:
            logger.warning(f"'{cmd_str}' execution doesn't end in 'exit' - Unable to retrieve real return code")

    return proc_stdout, proc_stderr, ret_code


def run_script_bug_workaround_retcode(cmd_list: list[str], quiet_on_error: bool = False) -> int:
    return run_script_bug_workaround(cmd_list, quiet_on_error)[2]


def run_command_retcode(cmd_list: list[str], quiet_on_error: bool = False) -> int:
    return run_command(cmd_list, quiet_on_error)[2]


def run_command_output(cmd_list: list[str], quiet_on_error: bool = False) -> str:
    proc_stdout, _, ret_code = run_command(cmd_list, quiet_on_error)
    return '' if ret_code else proc_stdout
