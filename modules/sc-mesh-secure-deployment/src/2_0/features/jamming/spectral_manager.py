#!/usr/bin/python
import math
import os
import struct
import subprocess
import sys
import time

import pandas as pd

from options import Options, VALID_CHANNELS
from util import run_command, map_freq_to_channel

HEADER_SIZE = 3
TYPE1_PACKET_SIZE = 17 + 56
TYPE2_PACKET_SIZE = 24 + 128
TYPE3_PACKET_SIZE = 26 + 64
SC_WIDE = 0.3125  # ieee 802.11 constants (in MHz)


class Spectral:
    def __init__(self):
        self.VALUES = dict()
        self.args = Options()

    def get_values(self) -> dict:
        return self.VALUES

    def get_driver(self) -> str:
        """
        Get device driver.
        """
        drivers = ["ath10k"]
        driver = os.popen('ls /sys/kernel/debug/ieee80211/phy* | grep ath').read().strip()
        if driver not in drivers:
            sys.exit(1)

        return driver

    def initialize_scan(self, driver: str) -> None:
        """
        Initialize spectral scan.
        """
        if driver == "ath10k":
            try:
                os.path.exists("/sys/kernel/debug/ieee80211/phy0/ath10k/spectral_scan_ctl")
            except Exception:
                print("Err: No spectral_scan_ctl file") if self.args.debug else None

            cmd_background = "echo background > /sys/kernel/debug/ieee80211/phy0/ath10k/spectral_scan_ctl"
            cmd_trigger = "echo trigger > /sys/kernel/debug/ieee80211/phy0/ath10k/spectral_scan_ctl"
            run_command(cmd_background, 'Failed to run cmd_background')
            run_command(cmd_trigger, 'Failed to run cmd_trigger')
        else:
            raise Exception(f"Invalid driver: {driver}")

    def execute_scan(self, frequencies: str, driver: str, bin_file: str) -> None:
        """
        Execute spectral scan.

        param interface: A string of the interface to use to perform the spectral scan.
        param frequencies: A string of the frequencies to scan.
        """
        # Validate that passed frequencies list is valid
        freq_list = [int(freq) for freq in frequencies.split(' ')]
        channel_list = [map_freq_to_channel(freq) for freq in freq_list]
        if not all(channel in VALID_CHANNELS for channel in channel_list):
            raise ValueError(f"Invalid frequencies: {frequencies}")

        # Execute scan
        do_scan_cmd = f"iw dev {self.args.scan_interface} scan freq {frequencies} flush"
        interface_up = f"ip link set dev {self.args.scan_interface} up"  # activate and enable the interface

        max_retries = 10
        for _ in range(max_retries):
            try:
                subprocess.call(interface_up, shell=True, stderr=subprocess.STDOUT, stdout=subprocess.DEVNULL)
                subprocess.call(do_scan_cmd, shell=True, stderr=subprocess.STDOUT, stdout=subprocess.DEVNULL)
                break
            except Exception:
                time.sleep(0.1)

        cmd_disable = f"echo disable > /sys/kernel/debug/ieee80211/phy0/{driver}/spectral_scan_ctl"
        cmd_dump = f"cat /sys/kernel/debug/ieee80211/phy0/{driver}/spectral_scan0 > {bin_file}"
        run_command(cmd_disable, 'Failed to run cmd_disable')
        run_command(cmd_dump, 'Failed to run cmd_dump')

    def read(self, bin_file) -> None:
        """
        Read spectral dump binary file.

        :return: A dataframe of the spectral scan performed.
        """
        try:
            binary_scan_file = self.file_open(bin_file)
            file_stats = os.stat(bin_file)
            data = binary_scan_file.read(file_stats.st_size)
            self.file_close(binary_scan_file)
            pos = 0
            count = 0
            self.VALUES = dict()
            while pos < len(data):
                (stype, slen) = struct.unpack_from(">BH", data, pos)
                if not ((stype == 1 and slen == TYPE1_PACKET_SIZE) or
                        (stype == 2 and slen == TYPE2_PACKET_SIZE) or
                        (stype == 3 and slen == TYPE3_PACKET_SIZE)):
                    print("skip malformed packet") if self.args.debug else None
                    break
                # 20 MHz
                if stype == 1:
                    if pos >= len(data) - HEADER_SIZE - TYPE1_PACKET_SIZE + 1:
                        break
                    pos += HEADER_SIZE
                    (max_exp, freq, rssi, noise, max_mag, max_index, hweight, tsf) = \
                        struct.unpack_from(">BHbbHBBQ", data, pos)
                    pos += 17

                    sdata = struct.unpack_from("56B", data, pos)
                    pos += 56

                    # calculate power in dBm
                    sum_square_sample = 0
                    samples = []
                    for raw_sample in sdata:
                        if raw_sample == 0:
                            sample = 1
                        else:
                            sample = raw_sample << max_exp
                        sum_square_sample += sample * sample
                        samples.append(sample)

                    if sum_square_sample == 0:
                        sum_square_sample = 1
                    sum_square_sample = 10 * math.log10(sum_square_sample)

                    sc_total = 56  # HT20: 56 OFDM subcarrier, HT40: 128
                    first_sc = freq - SC_WIDE * (sc_total / 2 + 0.5)

                    for i, sample in enumerate(samples):
                        subcarrier_freq = first_sc + i * SC_WIDE
                        sigval = noise + rssi + 20 * math.log10(sample) - sum_square_sample
                        self.VALUES[count] = (tsf, subcarrier_freq, noise, rssi, sigval)
                        count = count + 1

                # 40 MHz
                elif stype == 2:
                    if pos >= len(data) - HEADER_SIZE - TYPE2_PACKET_SIZE + 1:
                        break
                    pos += HEADER_SIZE
                    (chantype, freq, rssi_l, rssi_u, tsf, noise_l, noise_u,
                     max_mag_l, max_mag_u, max_index_l, max_index_u,
                     hweight_l, hweight_u, max_exp) = \
                        struct.unpack_from(">BHbbQbbHHbbbbb", data, pos)
                    pos += 24

                    sdata = struct.unpack_from("128B", data, pos)
                    pos += 128

                    sc_total = 128  # HT20: 56 ODFM subcarrier, HT40: 128

                    # unpack bin values
                    samples = []
                    for raw_sample in sdata:
                        if raw_sample == 0:
                            sample = 1
                        else:
                            sample = raw_sample << max_exp
                        samples.append(sample)

                    # create lower + upper binsum:
                    sum_square_sample_lower = 0
                    for sl in samples[0:63]:
                        sum_square_sample_lower += sl * sl
                    sum_square_sample_lower = 10 * math.log10(sum_square_sample_lower)

                    sum_square_sample_upper = 0
                    for su in samples[64:128]:
                        sum_square_sample_upper += su * su
                    sum_square_sample_upper = 10 * math.log10(sum_square_sample_upper)

                    # adjust center freq, depending on HT40+ or -
                    if chantype == 2:  # NL80211_CHAN_HT40MINUS
                        freq -= 10
                    elif chantype == 3:  # NL80211_CHAN_HT40PLUS
                        freq += 10
                    else:
                        print("got unknown chantype: %d" % chantype) if self.args.debug else None
                        raise

                    first_sc = freq - SC_WIDE * (sc_total / 2 + 0.5)
                    for i, sample in enumerate(samples):
                        if i < 64:
                            sigval = noise_l + rssi_l + 20 * math.log10(sample) - sum_square_sample_lower
                        else:
                            sigval = noise_u + rssi_u + 20 * math.log10(sample) - sum_square_sample_upper

                        subcarrier_freq = first_sc + i * SC_WIDE
                        self.VALUES[count] = (subcarrier_freq, (noise_l + noise_u) / 2, (rssi_l + rssi_u) / 2, sigval, (max_mag_l + max_mag_u) / 2)
                        count = count + 1

                # ath10k
                elif stype == 3:
                    if pos >= len(data) - HEADER_SIZE - TYPE3_PACKET_SIZE + 1:
                        break
                    pos += HEADER_SIZE
                    (chanwidth, freq1, freq2, noise, max_mag, gain_db,
                     base_pwr_db, tsf, max_index, rssi, relpwr_db, avgpwr_db,
                     max_exp) = \
                        struct.unpack_from(">bHHhHHHQBbbbb", data, pos)
                    pos += 26

                    sdata = struct.unpack_from("64B", data, pos)
                    pos += 64

                    self.VALUES[count] = (freq1, noise, max_mag, gain_db, base_pwr_db, rssi, relpwr_db, avgpwr_db)
                    count = count + 1
        except FileNotFoundError:
            print("File not found. Make sure the file exists.") if self.args.debug else None
        except PermissionError:
            print("Permission error. Check if you have the necessary permissions to access the file.") if self.args.debug else None
        except Exception as e:
            print(f"An error occurred: {str(e)}") if self.args.debug else None

    def create_dataframe(self, driver) -> pd.DataFrame:
        """
        Creates a dataframe of the spectral scan performed with the relevant features.

        return: A dataframe of the spectral scan.
        """
        vals_list = []
        spectral_capture_df = pd.DataFrame()

        for key, value in self.VALUES.items():
            vals_list.append(list(value))

        if driver == "ath10k":
            ath10k_scan_features = ["freq1", "noise", "max_magnitude", "total_gain_db", "base_pwr_db", "rssi", "relpwr_db", "avgpwr_db"]
            spectral_capture_df = pd.DataFrame(vals_list, columns=ath10k_scan_features)

        return spectral_capture_df

    @staticmethod
    def file_close(file_pointer):
        """Close the spectral binary dump file."""
        file_pointer.close()

    @staticmethod
    def file_open(fn="data"):
        """
        Open spectral binary dump file.

        param: The name of the binary dump file.
        """
        if not os.path.exists(fn):
            raise FileNotFoundError("File not found.")
        else:
            return open(fn, 'rb')
