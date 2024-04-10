class Interface:
    """
    Class to store interface name, operationstatus and MAC address
    """

    def __init__(self, interface_name: str, operstat: str, mac_address: str):
        self.interface_name = interface_name
        self.operstat = operstat
        self.mac_address = mac_address
