import os



class Setup:
    def __init__(self):
        self.scan_interface = [("ath9k", "wlp3s0"),("ath10k","wlp2s0"),("halow", "halow1")]

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
        
        
