#from __future__ import annotations

import gpsd
import subprocess


class WifiInfo:

    def __init__(self):
        self.__gpsdConnected = False
        #
        self.channel = ""
        self.country = ""
        self.rssi = ""
        self.txpower = ""
        self.noise = ""
        #self.snr = ""
        self.tx_mcs = ""
        self.rx_mcs = ""

        self.altitude = ""
        self.latitude = ""
        self.longitude = ""

    def get_channel(self):
        return self.channel

    def get_country(self):
        return self.country

    def get_rx_mcs(self):
        return self.rx_mcs

    def get_tx_mcs(self):
        return self.tx_mcs

    def get_txpower(self):
        return self.txpower

    def get_rssi(self):
        return self.rssi

    def get_noise(self):
        return self.noise



    def __update_location(self) -> tuple[float, float, float]:
        if not self.__gpsdConnected:
            gpsd.connect()
            self.__gpsdConnected = True

        gps_response = gpsd.get_current()
        try:
            return gps_response.lat, gps_response.lon, gps_response.alt
        except gpsd.NoFixError:
            return 0, 0, 0

    @staticmethod
    def __update_channel_and_twpower() -> tuple[str, str]:

        #iw_cmd = ['iw', 'dev']
        #iw_proc = subprocess.Popen(iw_cmd, stdout=subprocess.PIPE)
        #out = iw_proc.communicate()[0].decode().rstrip()

        out ="""phy#1
        Interface wlan0
                ifindex 44
                wdev 0x100000001
                addr e4:5f:01:2e:0a:25
                ssid comms_sleeve#0a25
                type AP
                channel 11 (2462 MHz), width: 20 MHz, center1: 2462 MHz
                txpower 31.00 dBm
phy#0
        Interface wlp1s0
                ifindex 50
                wdev 0x2
                addr 00:30:1a:4f:17:64
                type mesh point
                channel 36 (5180 MHz), width: 40 MHz, center1: 5190 MHz
                txpower 23.00 dBm
                multicast TXQ:
                        qsz-byt qsz-pkt flows   drops   marks   overlmt hashcol tx-bytes        tx-packets
                        0       0       6210    0       0       0       0       665525          6210

  """
        lines = out.split("\n")

        channel = "NULL"
        txpower = "NULL"
        interface_ok = False

        CHANNEL_STR = "channel "
        TXPWR_STR = "txpower "

        for line in lines:
            # Correct interface is wlp1s0
            if "Interface wlp1s0" in line:
                interface_ok = True
            if CHANNEL_STR in line and interface_ok:
                channel = line[line.index(CHANNEL_STR) + len(CHANNEL_STR):]
                channel = channel.split(' ')[0]
            elif TXPWR_STR in line and interface_ok:
                txpower = line[line.index(TXPWR_STR) + len(TXPWR_STR):]
                interface_ok = False

        return channel, txpower

    def __update_mcs(self) -> tuple[str, str]:

        # iw_cmd = ['iw', 'dev', 'wlp1s0', 'station', 'dump']
        # iw_proc = subprocess.Popen(iw_cmd, stdout=subprocess.PIPE)
        # out = iw_proc.communicate()[0].decode().rstrip()
        out = """ Station 00:30:1a:4f:17:65 (on wlp1s0)
        inactive time:  108 ms
        rx bytes:       20826892
        rx packets:     169662
        tx bytes:       878039082
        tx packets:     559496
        tx retries:     32375
        tx failed:      0
        rx drop misc:   315
        signal:         -29 [-35, -30, -42] dBm
        signal avg:     -28 [-34, -30, -42] dBm
        Toffset:        18446744073694444112 us
        tx bitrate:     405.0 MBit/s MCS 23 40MHz
        tx duration:    41727813 us
        rx bitrate:     405.0 MBit/s MCS 23 40MHz
        rx duration:    8703620 us
        airtime weight: 256
        expected throughput:    137.877Mbps
        mesh llid:      0
        mesh plid:      0
        mesh plink:     ESTAB
        mesh airtime link metric: 59
        mesh connected to gate: no
        mesh connected to auth server:  no
        mesh local PS mode:     ACTIVE
        mesh peer PS mode:      ACTIVE
        mesh non-peer PS mode:  ACTIVE
        authorized:     yes
        authenticated:  yes
        associated:     yes
        preamble:       long
        WMM/WME:        yes
        MFP:            no
        TDLS peer:      no
        DTIM period:    2
        beacon interval:1000
        connected time: 1939 seconds
        associated at [boottime]:       88.253s
        associated at:  88253 ms
        current time:   2027854 ms"""

        lines = out.split("\n")

        tx_mcs = ""
        rx_mcs = ""

        for line in lines:
            if "tx bitrate:" in line:
                if "MCS" in line:
                    tx_mcs = line[line.index("MCS")+4:line.index("MCS")+6]
            elif "rx bitrate:" in line:
                if "MCS" in line:
                    rx_mcs = line[line.index("MCS")+4:line.index("MCS") + 6]

        return rx_mcs, tx_mcs


    def __update_noise(self):

        #iw_cmd = ['iw', 'dev', 'wlp1s0', 'survey', 'dump']
        #iw_proc = subprocess.Popen(iw_cmd, stdout=subprocess.PIPE)
        #out = iw_proc.communicate()[0].decode().rstrip()

        out  =""" Survey data from wlp1s0
        frequency:                      2412 MHz
        noise:                          -73 dBm
        channel active time:            137 ms
        channel busy time:              115 ms
        channel receive time:           26 ms
        channel transmit time:          1 ms
Survey data from wlp1s0
        frequency:                      2417 MHz
        noise:                          -93 dBm
        channel active time:            62 ms
        channel busy time:              46 ms
        channel receive time:           20 ms
        channel transmit time:          1 ms
Survey data from wlp1s0
        frequency:                      5180 MHz [in use]
        noise:                          -94 dBm
        channel active time:            2630806 ms
        channel busy time:              530640 ms
        channel receive time:           505455 ms
        channel transmit time:          42790 ms
Survey data from wlp1s0
        frequency:                      5200 MHz
        noise:                          -94 dBm
        channel active time:            62 ms
        channel busy time:              6 ms
        channel receive time:           6 ms
        channel transmit time:          0 ms """

        lines = out.split("\n")
        found_in_use = False
        NOISE_STR = "noise:"
        noise = ""

        for line in lines:
            if "[in use]" in line:
                found_in_use = True
            if NOISE_STR in line and found_in_use:
                noise = line[line.index(NOISE_STR) + len(NOISE_STR):]
                break

        return noise.strip()

    def __update_country_code(self):
        #iw_cmd = ['iw', 'reg', 'get']
        #iw_proc = subprocess.Popen(iw_cmd, stdout=subprocess.PIPE)
        #out = iw_proc.communicate()[0].decode().rstrip()

        out = """ global
country FI: DFS-ETSI
	(2400 - 2483 @ 40), (N/A, 20), (N/A)
	(5150 - 5250 @ 80), (N/A, 23), (N/A), NO-OUTDOOR, AUTO-BW
	(5250 - 5350 @ 80), (N/A, 20), (0 ms), NO-OUTDOOR, DFS, AUTO-BW
	(5470 - 5725 @ 160), (N/A, 26), (0 ms), DFS
	(5725 - 5875 @ 80), (N/A, 13), (N/A)
	(57000 - 66000 @ 2160), (N/A, 40), (N/A)

phy#0
country 99: DFS-UNSET
	(2402 - 2472 @ 40), (N/A, 20), (N/A)
	(5140 - 5360 @ 80), (N/A, 30), (N/A), PASSIVE-SCAN
	(5715 - 5860 @ 80), (N/A, 30), (N/A), PASSIVE-SCAN

phy#1
country 99: DFS-UNSET
	(2402 - 2482 @ 40), (6, 20), (N/A)
	(2474 - 2494 @ 20), (6, 20), (N/A)
	(5140 - 5360 @ 160), (6, 20), (N/A)
	(5460 - 5860 @ 160), (6, 20), (N/A)
"""

        lines = out.split("\n")

        COUNTRY_STR = "country "
        country = ""
        for line in lines:
            if COUNTRY_STR in line:
                country = line[len(COUNTRY_STR):len(COUNTRY_STR)+2]
                break

        return country

    @staticmethod
    def __get_connected_nodes() -> int:
        """
        Returns number of connected nodes (including caller) in BATMAN protocol
        """

        batctl_cmd = ['batctl', 'o', '-H', '-t', '1']
        grep_cmd = ['grep', '-c', '*']

        batctl_proc = subprocess.Popen(batctl_cmd,
                                       stdout=subprocess.PIPE)

        grep_proc = subprocess.Popen(grep_cmd,
                                     stdin=batctl_proc.stdout,
                                     stdout=subprocess.PIPE)

        out = grep_proc.communicate()

        return int(out[0].decode().rstrip()) + 1

    def update(self):
        self.channel, self.txpower = self.__update_channel_and_twpower()
        self.noise = self.__update_noise()
        self.country = self.__update_country_code()
        self.rx_mcs, self.tx_mcs = self.__update_mcs()
        #self.latitude, self.longitude, self.altitude = self.__update_location()