import os
import netifaces
import subprocess


#class Setup:
#   def __init__(self):
#       self.scan_interface = [("ath9k", "wlp3s0"),("ath10k","wlp2s0"),("halow", "halow1")]

# Channel to frequency and frequency to channel mapping
CH_TO_FREQ = {1: 2412, 2: 2417, 3: 2422, 4: 2427, 5: 2432, 6: 2437, 7: 2442, 8: 2447, 9: 2452, 10: 2457, 11: 2462,
              36: 5180, 40: 5200, 44: 5220, 48: 5240, 52: 5260, 56: 5280, 60: 5300, 64: 5320, 100: 5500, 104: 5520,
              108: 5540, 112: 5560, 116: 5580, 120: 5600, 124: 5620, 128: 5640, 132: 5660, 136: 5680, 140: 5700,
              149: 5745, 153: 5765, 157: 5785, 161: 5805}

FREQ_TO_CH = {v: k for k, v in CH_TO_FREQ.items()}



def get_interface_operstate(interface : str) -> bool:
    """
    Check if a network interface is up through sysfs.
    Path : /sys/class/net/<interface>/operstate
    Arguments:
    interface: str -- Name of the network interface to check.
    Return:
    Bool -- True if the interface is up, False otherwise.
    """
    operstate_path = f"/sys/class/net/{interface}/operstate"
    try:
        with open(operstate_path, "r") as file:
            operstate = file.read().strip()
        if operstate.lower() == 'up':
            print(f"The operational state of {interface} is: up")
            return True
        else:
            print(f"The operational state of {interface} is: {operstate}")
            return False
    except FileNotFoundError:
        print(f"Interface {interface} not found.")
    except Exception as e:
        print(f"Error reading operstate: {e}")
        
def get_phy_interface(self, driver: str) -> str:
    """
    Get phy interface value associated with the driver 
    Arguments:
    driver : str -- driver name of the radio interface
    Return: 
    str: Phy interface value associated with the driver name
    """
    found = False
    for self.driver, self.interface in self.scan_interface:
        if self.driver == driver:
            self.phy_interface_path = f'/sys/class/net/{self.interface}/phy80211/name'
            print(f"Valid driver: {self.driver}, phy_interface_path is set to : {self.phy_interface_path}")
            found = True
            break
        else:
            continue 
    if found:
        try: 
            with open(self.phy_interface_path, "r") as file:
                self.phy_interface = file.read().strip()
                print(f"Phy interface: {self.phy_interface} for {driver}")
                return self.phy_interface
        except FileNotFoundError:
            print(f"Phy interface is not found for {driver}.")
        except Exception as e:
            print(f"Error reading phy_interface: {e}")    
    else:
        print("Invalid driver")
        
def get_ipv6_addr(interface) -> str:
    """
    Get the IPv6 address of the Radio network interface.

    :param interface: The name of the network interface.
    :return: The IPv6 address as a string.
    """
    # Retrieve the IPv6 addresses associated with the osf_interface
    ipv6_addresses = netifaces.ifaddresses(interface).get(netifaces.AF_INET6, [])
    if ipv6_addresses:
        for addr_info in ipv6_addresses:
            if 'addr' in addr_info and addr_info['addr'].startswith('fd'):
                return addr_info['addr']
            else:
                return None
    else:
        return None
    
def get_channel_bw(interface) -> int:
    
    run_cmd = f"iw dev {interface} info | grep 'width' | awk '{{print $6}}'"
    try:
        result = subprocess.run(run_cmd, 
                            shell=True, 
                            capture_output=True, 
                            text=True)

        if result.returncode == 0 and result.stdout.strip():
                width = int(result.stdout.strip())  
                print(f"Channel Width: {width}")
                return width
        else:
            print(f"Command failed or no output returned. Return code: {result.returncode}")
            return None
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")
        return None
    
def switch_frequency(frequency: int, interface: str, bandwidth: int, beacons_count: int ) -> None:
    run_cmd = f"iw dev {interface} switch freq {frequency} {bandwidth}MHz beacons {beacons_count}"
    try:
        result = subprocess.run(run_cmd, 
                            shell=True, 
                            capture_output=True, 
                            text=True)
        if(result.returncode != 0):
            print("Failed to execute the switch frequency command")

    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")
        return None
