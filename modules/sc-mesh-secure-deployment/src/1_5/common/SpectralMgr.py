#!/usr/bin/python
import struct
import math
import os


class Spectral:
    header_size = 3
    type1_packet_size = 17 + 56
    type2_packet_size = 24 + 128
    type3_packet_size = 26 + 64

    # ieee 802.11 constants
    sc_wide = 0.3125  # in MHz

    VALUES = []

    def __init__(self):
        self.VALUES = dict()

    def read(self, spectral_bin, size):
        self.VALUES = dict()

        data = spectral_bin.read(size)  # just read 2048 bytes
        count = 0
        pos = 0
        while pos < len(data):
            (stype, slen) = struct.unpack_from(">BH", data, pos)
            if not ((stype == 1 and slen == self.type1_packet_size) or
                    (stype == 2 and slen == self.type2_packet_size) or
                    (stype == 3 and slen == self.type3_packet_size)):
                print("skip malformed packet")
                break
            # 20 MHz
            if stype == 1:
                if pos >= len(data) - self.header_size - self.type1_packet_size + 1:
                    break
                pos += self.header_size
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
                first_sc = freq - self.sc_wide * (sc_total / 2 + 0.5)

                for i, sample in enumerate(samples):
                    subcarrier_freq = first_sc + i * self.sc_wide
                    sigval = noise + rssi + 20 * math.log10(sample) - sum_square_sample
                    self.VALUES[count] = (tsf, subcarrier_freq, noise, rssi, sigval)
                    #CHECK1
                    print("TSF: %d Freq: %d Noise: %d Rssi: %d Signal: %f" % (tsf, subcarrier_freq, noise, rssi, sigval))
                    count = count + 1
            # 40 MHz
            elif stype == 2:
                if pos >= len(data) - self.header_size - self.type2_packet_size + 1:
                    break
                pos += self.header_size
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

                first_sc = freq - self.sc_wide * (sc_total / 2 + 0.5)
                for i, sample in enumerate(samples):
                    if i < 64:
                        sigval = noise_l + rssi_l + 20 * math.log10(sample) - sum_square_sample_lower
                    else:
                        sigval = noise_u + rssi_u + 20 * math.log10(sample) - sum_square_sample_upper
                    subcarrier_freq = first_sc + i * self.sc_wide
                    self.VALUES[count] = (tsf, subcarrier_freq, (noise_l + noise_u) / 2, (rssi_l + rssi_u) / 2, sigval)
                    #CHECK2
                    print("TSF: %d Freq: %d Noise: %d Rssi: %d Signal: %f" % (tsf, subcarrier_freq, (noise_l+noise_u)/2, (rssi_l + rssi_u) / 2, sigval))
                    count = count + 1

            # ath10k
            elif stype == 3:
                if pos >= len(data) - self.header_size - self.type3_packet_size + 1:
                    break
                pos += self.header_size
                (chanwidth, freq1, freq2, noise, max_mag, gain_db,
                 base_pwr_db, tsf, max_index, rssi, relpwr_db, avgpwr_db,
                 max_exp) = \
                    struct.unpack_from(">bHHhHHHQBbbbb", data, pos)
                pos += 26

                sdata = struct.unpack_from("64B", data, pos)
                pos += 64
                self.VALUES[count] = (tsf, freq1, noise, rssi, sdata)
                #CHECK3
                #print("TSF: %d Freq: %d Noise: %d Rssi: %d Signal: %f" % (tsf, freq1, noise, rssi, sdata))
                print("TSF: %d Freq: %d Noise: %d Rssi: %d" % (tsf, freq1, noise, rssi))
                count = count + 1

    def get_values(self):
        return self.VALUES

    @staticmethod
    def initialize_scan():
        driver = os.popen('ls /sys/kernel/debug/ieee80211/phy* | grep ath').read().strip()

        if(driver == "ath9k"):
           # cmd_function =  "echo background > /sys/kernel/debug/ieee80211/phy0/ath9k/spectral_scan_ctl"
           cmd_function = "echo manual > /sys/kernel/debug/ieee80211/phy0/ath9k/spectral_scan_ctl"
           cmd_count = "echo 25 > /sys/kernel/debug/ieee80211/phy0/ath9k/spectral_count"
           cmd_trigger = "echo trigger > /sys/kernel/debug/ieee80211/phy0/ath9k/spectral_scan_ctl"
   
           os.popen(cmd_function)
           os.popen(cmd_count)
           os.popen(cmd_trigger)

        elif(driver == "ath10k"):
           cmd_background = "echo background > /sys/kernel/debug/ieee80211/phy0/ath10k/spectral_scan_ctl"
           cmd_trigger = "echo trigger > /sys/kernel/debug/ieee80211/phy0/ath10k/spectral_scan_ctl"

           os.popen(cmd_background)
           os.popen(cmd_trigger)


    @staticmethod
    def execute_scan():
        driver = os.popen('ls /sys/kernel/debug/ieee80211/phy* | grep ath').read().strip()        

        if (driver == "ath10k"):
            do_scan_cmd = "iw dev wlp1s0 scan"
            os.popen(do_scan_cmd)        

        cmd_scan = f"echo trigger > /sys/kernel/debug/ieee80211/phy0/{driver}/spectral_scan_ctl"
        cmd_dump = f"cat /sys/kernel/debug/ieee80211/phy0/{driver}/spectral_scan0 > /tmp/data"

        os.popen(cmd_scan)
        os.popen(cmd_dump)

    @staticmethod
    def file_close(file_pointer):
        file_pointer.close()

        
    @staticmethod
    def file_open(fn="data"):
        file_exists = os.path.exists(fn)
        
        if(file_exists):
            return open(fn, 'rb')
        elif(file_exists == False):
            os.system("touch " + fn)
            return open(fn, 'rb')
        
