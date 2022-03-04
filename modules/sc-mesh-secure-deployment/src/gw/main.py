import time
import re
import ipaddress
from src.tools import setup_logger, \
    run_shell_command, \
    find_batman_wifi_iface, check_interface_connectivity


class AutoGateway:
    def __init__(self, init_gw_mode: str="server") -> None:

        self.old_mesh_gateway_mac_and_ip = ("", "")
        self.masquerade_set = None
        self.gw_mode = init_gw_mode

        self.logger = setup_logger("agw")
        self.logger.debug("AutoGateWay Initialisation Started.")
        self.logger.debug("===================================")
        # self.logger.error("Error example")

        self.wifi_name = find_batman_wifi_iface()
        self.logger.debug(f"mesh {self.wifi_name=}")

        self.wan_iface = self.find_wwan_iface(self.wifi_name)
        self.logger.debug(f"{self.wan_iface=}")

        return_code, mode = run_shell_command("batctl gw_mode")

        if return_code != 0:
            self.logger.error("Error in getting gw_mode")
        else:
            mode = mode.split(" ")[0].strip()
            self.logger.debug(f"{mode=}")
        self.logger.debug("===================================")
        self.logger.debug("Make a clean and start...")
        self.clean_and_start()

    def clean_and_start(self) -> None:
        return_code, null = run_shell_command(
            f"sysctl net.ipv4.ip_forward=1")
        if return_code != 0:
            self.logger.error("Error setting: sysctl net.ipv4.ip_forward=1")

        ret, val = run_shell_command("iptables -t nat -F")
        if ret != 0:
            self.logger.error("Error iptables -t nat -F failed")

        # delete bat0/br-lan routing
        ret, val = run_shell_command(f"ip route show|grep -e bat0 -e br-lan|grep default|xargs ip route del >/dev/null")
        if ret != 0:
            self.logger.error("Error ip route del failed as no previous configuration")

        # without wwan_iface -> start with client mode
        if self.wan_iface == "":
            run_shell_command("batctl gw_mode client")
            self.configure_mesh_gateway(self.wifi_name, self.find_mesh_ipv4_subnet())
        else:
            run_shell_command("batctl gw_mode server")
            self.set_local_gateway(self.wan_iface)

    def find_wwan_iface(self, wifi) -> str:
        # use  "$wifi_device" and update wwan_name
        ret, value = run_shell_command("ifconfig")
        if ret != 0:
            self.logger.error("Error in ifconfig")
        network_if_list = re.findall(r'(\w+-*\w+):?\s+(?:\bLink\b|\bflags\b)', value)
        copy_nw_list = list(network_if_list)
        self.logger.debug(f"Network interface candidates: {network_if_list}")
        _remove_candidates = ["lo", "docker", "bat", "veth", "br", "eno", wifi]

        for removable in _remove_candidates:
            for nw in copy_nw_list:
                if nw.startswith(removable):
                    network_if_list.remove(nw)

        self.logger.debug(f"Usable interfaces: {network_if_list=}")

        for nw_if in network_if_list:
            ret = check_interface_connectivity(nw_if)
            if ret is True:
                self.logger.debug(f"internet is available: {nw_if}")
                return nw_if
            self.logger.debug(f"internet not available {nw_if}")

        return ""

    # server
    def set_local_gateway(self, wwan) -> None:
        # todo can this setting be done somewhere else
        ret, val = run_shell_command("iptables -P FORWARD ACCEPT")
        if ret != 0:
            self.logger.error("Error in iptables FORWARD ACCEPT")

        ret, val = run_shell_command(f"iptables -t nat -A POSTROUTING -o {wwan} -j MASQUERADE")
        if ret != 0:
            self.logger.error("Error in adding iptables MASQUERADE")

        ret, val = run_shell_command(f"ip route show|grep -e bat0 -e br-lan|grep default|xargs ip route del ")
        if ret != 0:
            self.logger.error("Error in ip route del as no previous configuration")
        else:
            self.old_mesh_gateway_mac_and_ip = ("", "")

    # client
    def configure_mesh_gateway(self, wifi, subnet) -> None:
        ret, val = run_shell_command("iptables -t nat -F")
        if ret != 0:
            self.logger.error("Error in iptables -t nat -F")

        # filter gateway node mac candidates
        ret, gateway_mac = run_shell_command("batctl gwl | grep -e " +
                                             r'"^\*"' +
                                             " | awk '{print $2}'")
        gateway_mac = gateway_mac.strip()
        if ret != 0:
            self.logger.error("Error in batctl gwl")

        if gateway_mac != "":
            self.logger.debug(f"Gateway available in MAC: {gateway_mac}")

            # find IP address for MAC
            if gateway_mac == self.old_mesh_gateway_mac_and_ip[0]:
                self.logger.debug(f"No need to change gateway")
            else:
                ret2, val2 = run_shell_command(f"ip route del default  >/dev/null")
                if ret2 != 0:
                    self.logger.error("Error ip route del failed for default route")
                # add local default mesh gw
                else:
                    self.logger.debug("Default route deleted successfully")
                command = "arp-scan " + str(subnet) + " --interface " + wifi + " 2>/dev/null|grep " + gateway_mac + " | awk '{print $1}'"
                ret, new_gateway_ip = run_shell_command(command)
                new_gateway_ip = new_gateway_ip.strip()
                if ret == 0:
                    self.logger.debug(
                        f"Gateway available new IP: {new_gateway_ip} old: {self.old_mesh_gateway_mac_and_ip[1]}")
                    # delete old possible local default mesh gw
                    ret, val = run_shell_command(
                        f"ip route del default via {self.old_mesh_gateway_mac_and_ip[1]} >/dev/null")
                    if ret != 0:
                        self.logger.error("Error ip route del failed for old gateway")
                    # add local default mesh gw
                    else:
                        self.logger.debug("Route Gateway Default Route added successfully")
                    ret, val = run_shell_command(f"ip route add default via {new_gateway_ip} >/dev/null")
                    if ret != 0:
                        self.logger.error("Error ip route add failed as no previous configuration")
                    else:
                        self.old_mesh_gateway_mac_and_ip = (gateway_mac, new_gateway_ip)
                else:
                    self.logger.error("Error arp-scan failed")
        else:
            self.logger.error("No gateway MAC available")

    def find_mesh_ipv4_subnet(self) -> str:
        ret, value = run_shell_command("ip -o -f inet addr show | grep -e bat0 -e br-lan | awk '{print $4}'")

        if ret != 0:
            self.logger.error("Error ip inet addr show failed")

        interface = ipaddress.IPv4Interface(value.strip())
        return str(interface.network)

    def gateway_client_activity(self) -> None:
        self.logger.debug("*** gateway_client_activity")
        self.wan_iface = self.find_wwan_iface(self.wifi_name)
        own_gateway = check_interface_connectivity(self.wan_iface)

        if own_gateway:
            self.logger.debug("change mode to server")
            return_code, null = run_shell_command("batctl gw_mode server")
            if return_code != 0:
                self.logger.error("Error in setting gw_mode")
            else:
                self.gw_mode = "server"
            self.set_local_gateway(self.wan_iface)
        else:
            self.logger.debug("configure_mesh_gateway")
            self.configure_mesh_gateway(self.wifi_name, self.find_mesh_ipv4_subnet())

    def gateway_server_activity(self) -> None:
        self.logger.debug("*** gateway_server_activity")
        self.wan_iface = self.find_wwan_iface(self.wifi_name)
        own_gateway = check_interface_connectivity(self.wan_iface)
        if not own_gateway:
            self.logger.debug("change mode to client")
            return_code, null = run_shell_command("batctl gw_mode client")
            if return_code != 0:
                self.logger.error("Error in setting gw_mode")
            else:
                self.gw_mode = "client"

            if self.masquerade_set is True:
                return_code, null = run_shell_command(
                    f"iptables -t nat -D POSTROUTING -s {str(self.find_mesh_ipv4_subnet())} "
                    f"-o {self.wan_iface} -j MASQUERADE")
                if return_code != 0:
                    self.logger.error("Error deleting MASQUERADE")
                else:
                    self.masquerade_set = True
        else:
            if self.masquerade_set is False:
                return_code, null = run_shell_command(
                    f"iptables -t nat -A POSTROUTING -s {str(self.find_mesh_ipv4_subnet())} "
                    f"-o {self.wan_iface} -j MASQUERADE")

                if return_code != 0:
                    self.logger.error("Error setting MASQUERADE")
                else:
                    self.masquerade_set = False

    def run(self):
        while True:
            if self.gw_mode == "client":
                self.gateway_client_activity()
            elif self.gw_mode == "server":
                self.gateway_server_activity()
            else:
                if self.wifi_name == "no_device":
                    self.clean_and_start()

            self.logger.debug("------------------------------------------------------------------------")
            # self.logger.debug(self.run_shell_command("ip route show")[1])
            # self.logger.debug(self.run_shell_command("iptables -t nat -L POSTROUTING")[1])
            # self.logger.debug("------------------------------------------------------------------------")
            # wait that mesh network is initialised, mesh-11s.sh change
            time.sleep(5)


if __name__ == "__main__":
    runner = AutoGateway()
    runner.run()
