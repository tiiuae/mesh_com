#!/usr/bin/python
import math
import os
import stat
import struct
import subprocess
import sys
import time
from datetime import datetime

import pandas as pd

HEADER_SIZE = 3
TYPE1_PACKET_SIZE = 17 + 56
TYPE2_PACKET_SIZE = 24 + 128
TYPE3_PACKET_SIZE = 26 + 64

# ieee 802.11 constants
SC_WIDE = 0.3125  # in MHz

DRIVERS = ["ath9k", "ath10k"]
DRIVER = os.popen('ls /sys/kernel/debug/ieee80211/phy* | grep ath').read().strip()
if DRIVER not in DRIVERS: sys.exit("No driver detected.")

outfile = "scan"

class Spectral:

    def __init__(self):
        self.VALUES = dict()

    def read(self, spectral_bin, size, channels, scan_count):
        self.VALUES = dict()

        data = spectral_bin.read(size)  # just read 2048 bytes
        count = 0
        pos = 0
        while pos < len(data):
            (stype, slen) = struct.unpack_from(">BH", data, pos)
            if not ((stype == 1 and slen == TYPE1_PACKET_SIZE) or
                    (stype == 2 and slen == TYPE2_PACKET_SIZE) or
                    (stype == 3 and slen == TYPE3_PACKET_SIZE)):
                print("skip malformed packet")
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

                    if (self.debug):
                        print("TSF: %d Freq: %d Noise: %d Rssi: %d Signal: %f" % (
                            tsf, subcarrier_freq, noise, rssi, sigval))
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
                    print("got unknown chantype: %d" % chantype)
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

        vals_list = []
        for key, value in self.VALUES.items():
            vals_list.append(list(value))

        if (DRIVER == "ath10k"):
            spectral_capture_df = pd.DataFrame(vals_list, columns=["freq1", "noise", "max_magnitude", "total_gain_db",
                                                                   "base_pwr_db", "rssi", "relpwr_db", "avgpwr_db"])

            if(spectral_capture_df["freq1"].nunique() != len(channels.split())):
               print("Missing: ", set(channels.split()) ^ set(spectral_capture_df["freq1"].unique()))
               valid = 0
               return valid

            else:
                valid = 1
                print(f"Scan {scan_count} complete")
                spectral_capture_df.to_csv(f'{outfile}_{scan_count}.csv', index=False)
                return valid

        if (DRIVER == "ath9k"):
            spectral_capture_df = pd.DataFrame(vals_list, columns=["freq1", "noise", "rssi", "signal", "max magnitude"])
            spectral_capture_df = spectral_capture_df.reindex(
                columns=["freq1", "noise", "signal", "max_magnitude", "total_gain_db", "base_pwr_db", "rssi",
                         "relpwr_db", "avgpwr_db"])
            return spectral_capture_df

    def get_values(self):
        return self.VALUES

    def initialize_scan(self):

        write_fix = f"echo -n disable > /sys/kernel/debug/ieee80211/phy0/{DRIVER}/spectral_scan_ctl"

        if (DRIVER == "ath9k"):
            # cmd_function =  "echo background > /sys/kernel/debug/ieee80211/phy0/ath9k/spectral_scan_ctl"
            cmd_function = "echo manual > /sys/kernel/debug/ieee80211/phy0/ath9k/spectral_scan_ctl"
            # cmd_count = "echo 25 > /sys/kernel/debug/ieee80211/phy0/ath9k/spectral_count"
            cmd_trigger = "echo trigger > /sys/kernel/debug/ieee80211/phy0/ath9k/spectral_scan_ctl"

            #subprocess.call(write_fix, shell=True)
            subprocess.call(cmd_function, shell=True, stderr=subprocess.STDOUT, stdout=subprocess.DEVNULL)
            # subprocess.call(cmd_count, shell=True)
            subprocess.call(cmd_trigger, shell=True, stderr=subprocess.STDOUT, stdout=subprocess.DEVNULL)

        elif (DRIVER == "ath10k"):
            file_exists = os.path.exists("/sys/kernel/debug/ieee80211/phy0/ath10k/spectral_scan_ctl")
            if (file_exists == False):
                print("no file spectral_scan_ctl")
            cmd_background = "echo background > /sys/kernel/debug/ieee80211/phy0/ath10k/spectral_scan_ctl"
            cmd_trigger = "echo trigger > /sys/kernel/debug/ieee80211/phy0/ath10k/spectral_scan_ctl"

            #subprocess.call(write_fix, shell=True)
            subprocess.call(cmd_background, shell=True, stderr=subprocess.STDOUT, stdout=subprocess.DEVNULL)
            subprocess.call(cmd_trigger, shell=True, stderr=subprocess.STDOUT, stdout=subprocess.DEVNULL)

    def execute_scan(self, interface, channels):
        scan = False
        do_scan_cmd = f"iw dev {interface} scan freq {channels} flush"

        while (scan == False):
            try:
                subprocess.run(do_scan_cmd, shell=True, stderr=subprocess.STDOUT,stdout=subprocess.DEVNULL, check=True)
                scan = True
            except:
                time.sleep(0.1)
                scan = False

            if (scan == True):
                break

        #cmd_background = f"echo -n background > /sys/kernel/debug/ieee80211/phy0/{DRIVER}/spectral_scan_ctl"
        #cmd_scan = f"echo -n trigger > /sys/kernel/debug/ieee80211/phy0/{DRIVER}/spectral_scan_ctl"
        cmd_disable = f"echo disable > /sys/kernel/debug/ieee80211/phy0/{DRIVER}/spectral_scan_ctl"
        cmd_dump = f"cat /sys/kernel/debug/ieee80211/phy0/{DRIVER}/spectral_scan0 > /tmp/data"

        #subprocess.call(cmd_background, shell=True)
        #subprocess.call(cmd_scan, shell=True)
        subprocess.call(cmd_disable, shell=True, stderr=subprocess.STDOUT, stdout=subprocess.DEVNULL)
        subprocess.call(cmd_dump, shell=True, stderr=subprocess.STDOUT, stdout=subprocess.DEVNULL)


    @staticmethod
    def file_close(file_pointer):
        file_pointer.close()

    @staticmethod
    def file_open(fn="data"):
        file_exists = os.path.exists(fn)
        if (file_exists):
            return open(fn, 'rb')
        elif (file_exists == False):
            os.system("touch " + fn)
            return open(fn, 'rb')
