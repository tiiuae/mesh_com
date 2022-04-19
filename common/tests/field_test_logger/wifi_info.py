import subprocess

EXPECTED_INTERFACE = "wlp1s0"

class WifiInfo:

    def __init__(self, interval):
        #
        self.neighbors = ""
        self.channel = ""
        self.country = ""
        self.rssi = ""
        self.txpower = ""
        self.noise = ""
        #self.snr = ""
        self.tx_mcs = ""
        self.rx_mcs = ""
        self.rx_throughput = 0
        self.tx_throughput = 0
        #
        self.__phyname = ""
        self.__old_rx_bytes = 0
        self.__old_tx_bytes = 0
        self.__interval_seconds = interval

    def get_neighbors(self):
        return self.neighbors

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

    def get_rx_throughput(self):
        return self.rx_throughput

    def get_tx_throughput(self):
        return self.tx_throughput

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

        #print(f"channel: {channel}")
        #print(f"txpower: {txpower}")

        return channel, txpower

    @staticmethod
    def __update_mcs_and_rssi() -> tuple[str, str, str]:

        iw_cmd = ['iw', 'dev', f"{EXPECTED_INTERFACE}", 'station', 'dump']
        iw_proc = subprocess.Popen(iw_cmd, stdout=subprocess.PIPE)
        out = iw_proc.communicate()[0].decode().rstrip()

        lines = out.split("\n")

        tx_mcs = ""
        rx_mcs = ""
        rssi = ""

        for line in lines:
            if "signal:" in line:
                rssi = line[line.index("signal:")+len("signal:"):].strip()

            if "tx bitrate:" in line:
                if "MCS" in line:
                    tx_mcs = line[line.index("MCS")+4:line.index("MCS")+6]
            elif "rx bitrate:" in line:
                if "MCS" in line:
                    rx_mcs = line[line.index("MCS")+4:line.index("MCS") + 6]

        #print(f"rssi: {rssi}")

        return rx_mcs, tx_mcs, rssi

    @staticmethod
    def __update_noise() -> str:

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

    @staticmethod
    def __update_country_code() -> str:
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

    def __update_throughputs(self):

        self.rx_throughput = 0
        self.tx_throughput = 0

        lines = []

        try:
            with open("/proc/net/dev", 'r') as f:
                lines = f.readlines()

        except FileNotFoundError:
            print("cannot read /proc/net/dev")

        rx_bytes = 0
        tx_bytes = 0

        for line in lines:
            if EXPECTED_INTERFACE in line:
                parts = line.split()
                rx_bytes = int(parts[1]) - self.__old_rx_bytes
                tx_bytes = int(parts[9]) - self.__old_tx_bytes
                self.__old_rx_bytes = int(parts[1])
                self.__old_tx_bytes = int(parts[9])
                break

        # Bytes to Bit/s
        self.rx_throughput = (rx_bytes * 8) / self.__interval_seconds
        self.tx_throughput = (tx_bytes * 8) / self.__interval_seconds

        #print(f"rx throughput: {self.rx_throughput}")
        #print(f"tx throughput: {self.tx_throughput}")

    def __update_batman_neighbors(self):

        batctl_cmd = ['batctl', 'n', '-H']

        batctl_proc = subprocess.Popen(batctl_cmd,
                                       stdout=subprocess.PIPE)

        out = batctl_proc.communicate()[0].decode().rstrip()

        lines = out.split("\n")

        nodes = ""

        for line in lines:
            node_stats = line.split()
            if len(node_stats) == 3:
                nodes = f"{nodes}{node_stats[1]},{node_stats[2]};"

        # Remove semicolon after last node in list
        self.neighbors = nodes[:-1]
        print(self.neighbors)



    def update(self):
        self.channel, self.txpower = self.__update_channel_and_twpower()
        self.noise = self.__update_noise()
        self.country = self.__update_country_code()
        self.rx_mcs, self.tx_mcs, self.rssi = self.__update_mcs_and_rssi()
        self.__update_throughputs()
        self.__update_batman_neighbors()
