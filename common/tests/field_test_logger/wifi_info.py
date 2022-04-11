import subprocess

EXPECTED_INTERFACE = "wlp1s0"

class WifiInfo:

    def __init__(self):
        #
        self.channel = ""
        self.country = ""
        self.rssi = ""
        self.txpower = ""
        self.noise = ""
        #self.snr = ""
        self.tx_mcs = ""
        self.rx_mcs = ""
        #
        self.__phyname = ""

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


    @staticmethod
    def __update_channel_and_twpower() -> tuple[str, str]:

        iw_cmd = ['iw', 'dev']
        iw_proc = subprocess.Popen(iw_cmd, stdout=subprocess.PIPE)
        out = iw_proc.communicate()[0].decode().rstrip()

        lines = out.split("\n")

        channel = "NaN"
        txpower = "NaN"
        interface_ok = False
        phyname = ""

        CHANNEL_STR = "channel "
        TXPWR_STR = "txpower "

        for line in lines:
            # We need phyname for later
            if "phy#" in line:
                phyname = line.rstrip()

            # Correct interface is wlp1s0
            if f"Interface {EXPECTED_INTERFACE}" in line:
                interface_ok = True
                # Capture correct phyname for later usage
                __phyname = phyname.replace('#', '')

            if CHANNEL_STR in line and interface_ok:
                channel = line[line.index(CHANNEL_STR) + len(CHANNEL_STR):]
                channel = channel.split(' ')[0]

            elif TXPWR_STR in line and interface_ok:
                txpower = line[line.index(TXPWR_STR) + len(TXPWR_STR):]
                interface_ok = False

        print(f"channel: {channel}")
        print(f"txpower: {txpower}")

        return channel, txpower

    def __update_mcs(self) -> tuple[str, str]:

        iw_cmd = ['iw', 'dev', f"{EXPECTED_INTERFACE}", 'station', 'dump']
        iw_proc = subprocess.Popen(iw_cmd, stdout=subprocess.PIPE)
        out = iw_proc.communicate()[0].decode().rstrip()

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

        iw_cmd = ['iw', 'dev', f"{EXPECTED_INTERFACE}", 'survey', 'dump']
        iw_proc = subprocess.Popen(iw_cmd, stdout=subprocess.PIPE)
        out = iw_proc.communicate()[0].decode().rstrip()

        lines = out.split("\n")
        found_in_use = False
        NOISE_STR = "noise:"
        noise = "NaN"

        for line in lines:
            if "[in use]" in line:
                found_in_use = True
            if NOISE_STR in line and found_in_use:
                noise = line[line.index(NOISE_STR) + len(NOISE_STR):]
                break

        return noise.strip()

    def __update_country_code(self):
        iw_cmd = ['iw', 'reg', 'get']
        iw_proc = subprocess.Popen(iw_cmd, stdout=subprocess.PIPE)
        out = iw_proc.communicate()[0].decode().rstrip()

        lines = out.split("\n")

        COUNTRY_STR = "country "
        country = "NaN"
        for line in lines:
            if COUNTRY_STR in line:
                country = line[len(COUNTRY_STR):len(COUNTRY_STR)+2]
                break

        #print(f"country: {country}")

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
