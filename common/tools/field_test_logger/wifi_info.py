import subprocess

EXPECTED_INTERFACE = "wlp1s0"

class WifiInfo:

    def __init__(self, interval):
        self.__neighbors = ""
        self.__originators = ""
        self.__channel = ""
        self.__country = ""
        self.__txpower = ""
        self.__noise = ""
        self.__rx_throughput = 0
        self.__tx_throughput = 0
        self.__stations = {}
        self.__phyname = ""
        self.__old_rx_bytes = 0
        self.__old_tx_bytes = 0
        self.__interval_seconds = interval

    # ----------------------------------------

    def get_mac_addr(self):
        try:
            with open(f"/sys/class/net/{EXPECTED_INTERFACE}/address", 'r') as f:
                value = f.readline()
                return value.strip()
        except:
            return "NaN"

    def get_neighbors(self):
        return self.__neighbors

    def get_originators(self):
        return self.__originators

    def get_channel(self):
        return self.__channel

    def get_country(self):
        return self.__country

    def get_rssi(self):
        out = ""
        for i in self.__stations.keys():
            # List index 0 contains RSSI
            out = f"{out}{i},{self.__stations[i][0]};"

        # Remove semicolon after last node in list
        out = out[:-1]

        return out

    def get_rx_mcs(self):
        out = ""
        for i in self.__stations.keys():
            # List index 2 contains RX_MCS
            out = f"{out}{i},{self.__stations[i][2]};"

        # Remove semicolon after last node in list
        out = out[:-1]

        return out

    def get_tx_mcs(self):
        out = ""
        for i in self.__stations.keys():
            # List index 1 contains TX_MCS
            out = f"{out}{i},{self.__stations[i][1]};"

        # Remove semicolon after last node in list
        out = out[:-1]

        return out

    def get_txpower(self):
        return self.__txpower

    def get_noise(self):
        return self.__noise

    def get_rx_throughput(self):
        return self.__rx_throughput

    def get_tx_throughput(self):
        return self.__tx_throughput

    # ----------------------------------------

    def __update_channel_and_twpower(self):
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
                txpower = txpower.split()[0]
                interface_ok = False

        #print(f"channel: {channel}")
        #print(f"txpower: {txpower}")
        self.__channel = channel
        self.__txpower = txpower

    def __update_mcs_and_rssi(self):
        iw_cmd = ['iw', 'dev', f"{EXPECTED_INTERFACE}", 'station', 'dump']
        iw_proc = subprocess.Popen(iw_cmd, stdout=subprocess.PIPE)
        out = iw_proc.communicate()[0].decode().rstrip()

        lines = out.split("\n")

        station_mac = "NaN"
        tx_mcs = "NaN"
        rx_mcs = "NaN"
        rssi = "NaN"

        for line in lines:
            if "Station" in line:
                # Line format is 'Station AA:BB:CC:DD:EE:FF (on wlp1s0)'
                station_mac = line.split()[1]

            if "signal:" in line and "]" in line:
                # Line format is 'signal: -21 [-26, -30, -24] dBm'
                rssi = line[line.index("signal:")+len("signal:"):].strip()
                rssi = rssi[:rssi.index("]")+1]

            if "tx bitrate:" in line:
                if "MCS" in line:
                    tx_mcs = line[line.index("MCS")+4:line.index("MCS") + 6]
            elif "rx bitrate:" in line:
                if "MCS" in line:
                    rx_mcs = line[line.index("MCS")+4:line.index("MCS") + 6]

                self.__stations[station_mac] = [rssi, tx_mcs, rx_mcs]

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
                noise = noise.split()[0].strip()
                break

        self.__noise = noise

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

        self.__country = country

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
        self.__rx_throughput = (rx_bytes * 8) / self.__interval_seconds
        self.__tx_throughput = (tx_bytes * 8) / self.__interval_seconds

        #print(f"rx throughput: {self.rx_throughput}")
        #print(f"tx throughput: {self.tx_throughput}")

    def __update_batman_neighbors(self):
        batctl_cmd = ['batctl', 'n', '-n', '-H']

        batctl_proc = subprocess.Popen(batctl_cmd,
                                       stdout=subprocess.PIPE)

        out = batctl_proc.communicate()[0].decode().rstrip()

        lines = out.split("\n")

        nodes = ""

        for line in lines:
            node_stats = line.split()
            if len(node_stats) == 3:
                nodes = f"{nodes}{node_stats[1]},{node_stats[2][:-1]};"

        # Remove semicolon after last node in list
        self.__neighbors = nodes[:-1]
        #print(self.neighbors)

    def __update_batman_originators(self):
        batctl_cmd = ['batctl', 'o', '-n', '-H']

        batctl_proc = subprocess.Popen(batctl_cmd,
                                       stdout=subprocess.PIPE)

        out = batctl_proc.communicate()[0].decode().rstrip()

        lines = out.split("\n")

        nodes = ""

        for line in lines:
            node_stats = line.split()
            if len(node_stats) == 7:
                nodes = f"{nodes}{node_stats[1]},{node_stats[4]};"

        # Remove semicolon after last node in list
        self.__originators = nodes[:-1]
        #print(self.originators)
    # ----------------------------------------

    def update(self):
        """
        Update variables with latest info
        """
        self.__update_channel_and_twpower()
        self.__update_noise()
        self.__update_country_code()
        self.__update_mcs_and_rssi()
        self.__update_throughputs()
        self.__update_batman_neighbors()
