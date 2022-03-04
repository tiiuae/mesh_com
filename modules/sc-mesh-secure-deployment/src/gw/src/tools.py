import subprocess
import logging


def find_batman_wifi_iface():
    ret, value = run_shell_command("batctl if")

    if value != "" and ret == 0:
        return value.strip().split(":")[0]

    return "no_device"


def check_interface_connectivity(interface):
    # todo other tests than ping?
    # cmd = f"curl google.com --interface {interface} --connect-timeout 2"
    cmd = f"ping -c 1 -I {interface} 8.8.8.8 -W 2"
    ret, stdout = run_shell_command(cmd)

    if ret == 0:
        return True
    return False


def setup_logger(name):
    formatter = logging.Formatter(fmt='%(asctime)s %(levelname)-8s %(message)s',
                                  datefmt='%Y-%m-%d %H:%M:%S')
    handler = logging.FileHandler('gw-log.txt', mode='w')
    handler.setFormatter(formatter)
    # screen_handler = logging.StreamHandler(stream=sys.stdout)
    screen_handler = logging.StreamHandler(stream=None)
    screen_handler.setFormatter(formatter)
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)
    logger.addHandler(handler)
    logger.addHandler(screen_handler)
    return logger


def run_shell_command(cmd):
    # cmd_split = cmd.split(" ")
    cmd_split = ["bash", "-c", cmd]
    result = subprocess.run(cmd_split, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    # print("   *** execute: " + cmd)
    # print("   *** ret: " + str(result.returncode))
    # print("   *** return: " + str(result.stdout.decode("utf-8")))
    return result.returncode, result.stdout.decode("utf-8")
