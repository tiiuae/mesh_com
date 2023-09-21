"""
This module is used to control the provisioning LED on the SC Mesh Secure Deployment board.
"""
import os
import sys

# pylint: disable=too-few-public-methods
class LedControl:
    """
    This class is used to control the provisioning LED on the SC Mesh Secure Deployment board.
    """

    def __init__(self):
        try:
            with open("/etc/comms_pcb_version", "r", encoding="utf8") as version_file:
                self.comms_pcb_version = float(version_file.read().split("=")[-1].strip())
        except FileNotFoundError:
            sys.exit(1)

    def provisioning_led_control(self, state):
        """
        Control the provisioning LED.
        :param state: start, active, stop, fail
        :return: None
        """

        path = "/sys/class/leds/mesh"
        trigger_seq = "timer"
        trigger_none = "none"
        start_seq = 1000
        active_seq = 100

        trigger_used = ""
        seq_used = 0

        if state == "start":
            seq_used = start_seq
            trigger_used = trigger_seq
        elif state == "active":
            seq_used = active_seq
            trigger_used = trigger_seq
        elif state == "stop":
            seq_used = 0
            trigger_used = trigger_none
        elif state == "fail":
            seq_used = 1
            trigger_used = trigger_none

        if self.comms_pcb_version == 1:
            print("test")
        elif self.comms_pcb_version in (0.5, 0):
            try:
                with open(os.path.join(path, "trigger"), "w", encoding="utf-8") as trigger_file:
                    trigger_file.write(trigger_used)
            except (FileNotFoundError, PermissionError):
                pass

            try:
                with open(os.path.join(path, "delay_off"), "w", encoding="utf-8") as delay_off_file:
                    delay_off_file.write(str(seq_used))
            except (FileNotFoundError, PermissionError):
                pass

            try:
                with open(os.path.join(path, "delay_on"), "w", encoding="utf-8") as delay_on_file:
                    delay_on_file.write(str(seq_used))
            except (FileNotFoundError, PermissionError):
                pass

            try:
                with (open(os.path.join(path, "brightness"), "w", encoding="utf-8")
                      as brightness_file):
                    brightness_file.write(str(seq_used))
            except (FileNotFoundError, PermissionError):
                pass

if __name__ == "__main__":
    led = LedControl()
    import time
    led.provisioning_led_control("start")
    time.sleep(5)
    led.provisioning_led_control("active")
    time.sleep(5)
    led.provisioning_led_control("stop")
    time.sleep(5)
    led.provisioning_led_control("fail")
