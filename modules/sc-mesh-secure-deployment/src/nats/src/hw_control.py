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
        self.not_supported = False
        try:
            with open("/etc/comms_pcb_version", "r", encoding="utf8") as version_file:
                self.comms_pcb_version = float(version_file.read().split("=")[-1].strip())
        except FileNotFoundError:
            self.not_supported = True

    @staticmethod
    def _write_to_file(path, file, value):
        try:
            with open(os.path.join(path, file), "w", encoding="utf-8") as engine:
                engine.write(value)
        except (FileNotFoundError, PermissionError):
            pass

    def _led_control_0(self, state) -> None:
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

        trigger_used = "none"
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

        # write to sys class led
        self._write_to_file(path, "trigger", trigger_used)
        self._write_to_file(path, "delay_off", str(seq_used))
        self._write_to_file(path, "delay_on", str(seq_used))
        self._write_to_file(path, "brightness", str(seq_used))

    def _led_control_1(self, state) -> None:
        """
        Control the provisioning LED.
        :param state: start, active, stop, fail
        :return: None
        """
        path = "/sys/class/leds/rgb_leds:channel0/device"
        led_matrix = "000000001"
        code_used = "9d094000a001"

        # LASM compiler:
        # .segment program1      ; segment begins
        #     mux_sel 9          ; select led 9 blue
        # loop1:          set_pwm 2Fh
        #     wait 0.4
        #     wait 0.4
        #     wait 0.2
        #     set_pwm 00h
        #     wait 0.4
        #     wait 0.4
        #     wait 0.2
        #     branch 0,loop1
        start_code = "9d09402f740074005a004000740074005a00a001"
        # LASM compiler:
        # .segment program1      ; segment begins
        #     mux_sel 9          ; select led 9 blue
        # loop1:          set_pwm 2Fh
        #     wait 0.1
        #     set_pwm 00h
        #     wait 0.1
        #     branch 0,loop1
        active_code = "9d09402f4c0040004c00a001"

        if state == "start":
            code_used = start_code
        elif state == "active":
            code_used = active_code
        elif state == "stop":
            #            pwm   00
            code_used = "9d094000a001"
        elif state == "fail":
            #            pwm   2f
            code_used = "9d09402fa001"

        # write to led driver
        self._write_to_file(path, "engine3_mode", "disabled")
        self._write_to_file(path, "engine3_mode", "load")
        self._write_to_file(path, "engine3_load", code_used)
        self._write_to_file(path, "engine3_leds", led_matrix)
        self._write_to_file(path, "engine3_mode", "run")


    def provisioning_led_control(self, state):
        """
        Control the provisioning LED.
        :param state: start, active, stop, fail
        :return: None
        """
        if self.not_supported:
            return None
        
        if self.comms_pcb_version == 1:
            self._led_control_1(state)
        elif self.comms_pcb_version in (0.5, 0):
            self._led_control_0(state)

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
