import subprocess
import re


class WifiInfo:

    def __init__(self, interval, interface, batman_interface):
        self.__neighbors = ""
        self.__originators = ""
        self.__channel = ""
        self.__country = ""
        self.__txpower = ""
        self.__noise = ""
        self.__snr = "NaN"         # halow specific
        self.__rx_throughput = 0
        self.__tx_throughput = 0
        self.__stations = {}
        self.__phyname = ""
        self.__old_rx_bytes = 0
        self.__old_tx_bytes = 0
        self.__interval_seconds = interval
        self.__interface = interface
        self.__batman_interface = batman_interface

    # ----------------------------------------

    def __run_command(self, cmd):
        try:
            with subprocess.Popen(cmd, stdout=subprocess.PIPE) as proc:
                return proc.communicate()[0].decode().rstrip()
        except FileNotFoundError:
            print(f"Error running command: {cmd}")
            return ""

    def get_mac_addr(self):
        try:
            with open(f"/sys/class/net/{self.__interface}/address", 'r') as f:
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

    def get_snr(self):
        return self.__snr

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

    def get_expected_throughput(self):
        out = ""
        for i in self.__stations.keys():
            # Expected throughput is at index 3
            out = f"{out}{i},{self.__stations[i][3]};"

        # Remove semicolon after the last station
        return out[:-1]

    def get_inactive_time(self):
        out = ""
        for i in self.__stations.keys():
            # Inactive Time is at index 4
            out = f"{out}{i},{self.__stations[i][4]};"

        # Remove semicolon after the last station
        return out[:-1]

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
        out = self.__run_command(iw_cmd)

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
            if f"Interface {self.__interface}" in line:
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
        iw_cmd = ['iw', 'dev', f"{self.__interface}", 'station', 'dump']
        out = self.__run_command(iw_cmd)
        lines = out.split("\n")

        station_mac = "NaN"
        tx_mcs = "NaN"
        rx_mcs = "NaN"
        rssi = "NaN"
        expected_throughput = "NaN"
        inactive_time = "NaN"

        # halow station info fetched from cli_app if needed.
        halow_stations = None

        for line in lines:
            if "Station" in line:
                # Line format is 'Station AA:BB:CC:DD:EE:FF (on wlp1s0)'
                station_mac = line.split()[1]

            if "signal:" in line:
                rssi = line[line.index("signal:")+len("signal:"):].strip()
                if "]" in line:
                    # Line format is 'signal: -21 [-26, -30, -24] dBm'
                    rssi = rssi[:rssi.index("]")+1]
                else:
                    # halow station dump does not contain [].
                    # rssi: "-63 dBm" --> drop the last 4 characters
                    rssi = rssi[:-4]

            if "tx bitrate:" in line:
                if "MCS" in line:
                    tx_mcs = line[line.index("MCS")+4:line.index("MCS") + 6]
                elif self.__interface.startswith("halow"):
                    if halow_stations is None:
                        halow_stations = self.get_halow_stations()
                    try:
                        tx_mcs = halow_stations.get(station_mac)[0]
                    except (IndexError, TypeError):
                        pass
            elif "rx bitrate:" in line:
                if "MCS" in line:
                    rx_mcs = line[line.index("MCS")+4:line.index("MCS") + 6]
                elif self.__interface.startswith("halow"):
                    if halow_stations is None:
                        halow_stations = self.get_halow_stations()
                    try:
                        rx_mcs = halow_stations.get(station_mac)[1]
                    except (IndexError, TypeError):
                        pass

            if "expected throughput:" in line:
                # Extract the value and remove the "Mbps" suffix
                throughput_str = line.split("expected throughput:")[1].strip().replace("Mbps", "").strip()
                try:
                    expected_throughput = float(throughput_str)
                except ValueError:
                    pass

            if "inactive time:" in line:
                # Extract the value and remove the "ms" suffix
                inactive_time_str = line.split("inactive time:")[1].strip().replace("ms", "").strip()
                try:
                    inactive_time = int(inactive_time_str)
                except ValueError:
                    pass

        self.__stations[station_mac] = [rssi, tx_mcs, rx_mcs, expected_throughput, inactive_time]

    def get_halow_stations(self) -> dict:
        cli_app_cmd = ['/usr/local/bin/cli_app', 'show', 'sta', '0', 'all']
        out = self.__run_command(cli_app_cmd)
        lines = out.split("\n")
        data_begins = False
        halow_stations = {}
        for line in lines:
            if data_begins:
                # 0          84:25:3f:9c:0a:e9    2      ASSOC   3.00MBit/s(MCS 7)  0.30MBit/s(MCS 0)
                cols = line.split()
                if len(cols) == 8:
                    halow_stations[cols[1]] = [cols[5][:-1], cols[7][:-1]]
            if "=======" in line:
                data_begins = True
            if "Duplicate" in line:
                break
        return halow_stations

    def __update_snr(self):
        if self.__interface.startswith("halow"):
            cmd = ["/usr/local/bin/cli_app", "show", "signal"]
            out = self.__run_command(cmd)
            mac_snr = {}
            for line in out.splitlines():
                splitted = line.split()
                if "MAC addr" in line and len(splitted) > 3:
                    mac = splitted[3]
                if "snr" in line and len(splitted) > 8:
                    mac_snr[mac] = splitted[9]

            self.__snr = ';'.join([f"{k},{v}" for k, v in mac_snr.items()])
        else:
            self.__snr = "NaN"


    def __update_noise(self):
        iw_cmd = ['iw', 'dev', f"{self.__interface}", 'survey', 'dump']
        out = self.__run_command(iw_cmd)

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
        out = self.__run_command(iw_cmd)

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
            if self.__interface in line:
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
        batctl_cmd = ['batctl', 'meshif', self.__batman_interface, 'n']
        out = self.__run_command(batctl_cmd)

        routing_algo = None
        lines = out.split('\n')
        for line in lines:
            if "B.A.T.M.A.N. adv" in line:
                if "BATMAN_V" in line:
                    routing_algo = "BATMAN_V"
                    break

        if routing_algo == "BATMAN_V":
            node_index = 0
        else:
            node_index = 1

        lines = out.split("\n")

        nodes = ""

        for line in lines:
            if "Neighbor" in line or "B.A.T.M.A.N. adv" in line:
                continue
            node_stats = line.split()
            if len(node_stats) >= node_index + 2:
                nodes = f"{nodes}{node_stats[node_index]},{node_stats[node_index + 1]};"

        # Remove semicolon after last node in list
        self.__neighbors = nodes[:-1]
        #print(self.neighbors)

    def __update_batman_originators(self):
        batctl_cmd = ['batctl', 'meshif', self.__batman_interface, 'o', '-n', '-H']
        out = self.__run_command(batctl_cmd)

        # Remove parentheses and square brackets
        out = re.sub('[()\\[\\]]', '', out)

        lines = out.split("\n")
        nodes = ""

        for line in lines:
            node_stats = line.split()
            if len(node_stats) == 6 and node_stats[0] == '*':
                nodes = f"{nodes}{node_stats[1]},{node_stats[4]};"

        # Remove semicolon after last node in list
        self.__originators = nodes[:-1]

    # ----------------------------------------

    @staticmethod
    def is_up(interface):
        cmd = ['ip', 'link', 'show', 'dev', interface]
        try:
            output = subprocess.check_output(cmd, stderr=subprocess.STDOUT, text=True)
            # Check if the interface state is not "DOWN"
            if 'state UP' in output:
                return True
            return False
        except subprocess.CalledProcessError as e:
            print(f"Error running command: {e}")
            return False

    @staticmethod
    def is_mesh(interface):
        cmd = ['iw', 'dev', interface, 'info']
        try:
            output = subprocess.check_output(cmd, stderr=subprocess.STDOUT, text=True)
            # Check if the interface type is "mesh point"
            if 'type mesh point' in output:
                return True
            return False
        except subprocess.CalledProcessError as e:
            print(f"Error running command: {e}")
            return False

    def update(self):
        """
        Update variables with latest info
        """
        self.__update_channel_and_twpower()
        self.__update_noise()
        self.__update_country_code()
        self.__update_mcs_and_rssi()
        self.__update_snr()
        self.__update_throughputs()
        self.__update_batman_neighbors()
        self.__update_batman_originators()
