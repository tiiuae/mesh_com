"""
Unittests for validations.py
"""
# pylint: disable=import-error, wrong-import-position, unused-import, \
# disable=unresolved-reference, undefined-variable, too-long
import unittest
from .context import validation


class TestValidation(unittest.TestCase):
    """
    Test cases for validations.py
    """

    # create test for validate_ssid(ssid)
    def test_validate_ssid(self):
        """
        Test cases for validate_ssid(ssid)
        """
        # ssid is valid
        self.assertTrue(validation.validate_ssid('mesh'))
        # ssid is invalid
        self.assertFalse(validation.validate_ssid('meshmeshmessshmeshmessshmeshmesss'))
        # ssid is invalid
        self.assertFalse(validation.validate_ssid(1))
        # print ascii chars in the range of ascii code 32 to 126 (decimal)
        ssid = [chr(char) for char in range(32, 126)]
        self.assertTrue(validation.validate_ssid("".join(ssid)[0:32]))
        self.assertTrue(validation.validate_ssid("".join(ssid)[33:64]))
        self.assertTrue(validation.validate_ssid("".join(ssid)[65:94]))

    def test_validate_wpa3_psk(self):
        """
        Test cases for validate_wpa3_psk(psk)
        """
        # psk is invalid
        self.assertFalse(validation.validate_wpa3_psk(1))
        # psk is valid
        self.assertTrue(validation.validate_wpa3_psk('12345678'))
        # psk is invalid (short)
        self.assertFalse(validation.validate_wpa3_psk('1234567'))
        # psk with invalid (long)
        self.assertFalse(
            validation.validate_wpa3_psk("""12345678901234567890123456789012345678901234567890123456789012345"""))

    def test_validate_ip_address(self):
        """
        Test cases for validate_ip_address(ip)
        """
        # ip is valid
        self.assertTrue(validation.validate_ip_address('10.10.10.10'))
        # ip is invalid
        self.assertFalse(validation.validate_ip_address('1000.0.0.0'))
        # ip is invalid
        self.assertFalse(validation.validate_ip_address(1))
        # ip is invalid
        self.assertFalse(validation.validate_ip_address('0.0.0'))
        # ip is invalid
        self.assertFalse(validation.validate_ip_address('0.0.256.0'))

    def test_validate_netmask(self):
        """
        Test cases for validate_netmask(netmask)
        """
        # netmask is valid
        self.assertTrue(validation.validate_netmask('255.255.255.0'))
        # netmask is invalid
        self.assertFalse(validation.validate_netmask('0.0.0.255'))
        # netmask is invalid
        self.assertFalse(validation.validate_netmask('a.b.c.d'))
        # netmask is invalid
        self.assertFalse(validation.validate_netmask(1))

    def test_validate_tx_power(self):
        """
        Test cases for validate_tx_power(power_in_dbm)
        """
        # power is valid
        self.assertTrue(validation.validate_tx_power(1))
        # power is valid
        self.assertTrue(validation.validate_tx_power(30))
        # power is invalid
        self.assertFalse(validation.validate_tx_power(101))
        # power is invalid
        self.assertFalse(validation.validate_tx_power(-1))
        # power is Valid
        self.assertTrue(validation.validate_tx_power("10"))

    def test_validate_country_code(self):
        """
        Test cases for validate_country_code(country_code)
        """
        # country code is valid
        self.assertTrue(validation.validate_country_code('DE'))
        # country code is invalid
        self.assertFalse(validation.validate_country_code('DEU'))
        # country code is invalid
        self.assertFalse(validation.validate_country_code('D'))
        # country code is invalid
        self.assertFalse(validation.validate_country_code('DEUTSCHLAND'))
        # country code is invalid
        self.assertFalse(validation.validate_country_code(''))
        # country code is invalid
        self.assertFalse(validation.validate_country_code(' '))
        # country code is invalid
        self.assertFalse(validation.validate_country_code('  '))
        # country code is invalid
        self.assertFalse(validation.validate_country_code('   '))
        # country code is invalid
        self.assertFalse(validation.validate_country_code(1))

    def test_validate_wifi_mode(self):
        """
       Test cases for validate_wifi_mode(mode)
       """
        self.assertFalse(validation.validate_mode('ap'))
        self.assertFalse(validation.validate_mode(1))
        self.assertFalse(validation.validate_mode('sta'))
        self.assertTrue(validation.validate_mode('mesh'))

    def test_validate_frequency(self):
        """
        Test cases for validate_frequency(frequency)
        """
        # frequency is valid
        self.assertTrue(validation.validate_frequency(2412))
        # frequency is valid
        self.assertTrue(validation.validate_frequency(2462))
        # frequency is valid
        self.assertTrue(validation.validate_frequency(5180))
        # frequency is valid
        self.assertTrue(validation.validate_frequency(5825))
        # frequency is invalid
        self.assertFalse(validation.validate_frequency(2411))
        # frequency is invalid
        self.assertFalse(validation.validate_frequency(2463))
        # frequency is invalid
        self.assertFalse(validation.validate_frequency(5179))
        # frequency is invalid
        self.assertFalse(validation.validate_frequency(5826))
        # frequency is invalid
        self.assertFalse(validation.validate_frequency("2412"))

    def test_validate_routing(self):
        """
        Test cases for validate_routing(routing)
        """
        # routing is invalid
        self.assertFalse(validation.validate_routing('none'))
        # routing is valid
        self.assertTrue(validation.validate_routing('olsr'))
        # routing is valid
        self.assertTrue(validation.validate_routing('batman-adv'))
        # routing is invalid
        self.assertFalse(validation.validate_routing('nonee'))
        # routing is invalid
        self.assertFalse(validation.validate_routing('olsrr'))
        # routing is invalid
        self.assertFalse(validation.validate_routing('batmann'))
        # routing is invalid
        self.assertFalse(validation.validate_routing(1))

    def test_validate_priority(self):
        """
        Test cases for validate_priority(priority)
        """
        # priority is invalid
        self.assertFalse(validation.validate_priority('none'))
        # priority is valid
        self.assertTrue(validation.validate_priority('long_range'))
        # priority is valid
        self.assertFalse(validation.validate_priority('high'))
        # priority is invalid
        self.assertFalse(validation.validate_priority('nonee'))
        # priority is invalid
        self.assertFalse(validation.validate_priority('long_rang'))
        # priority is invalid
        self.assertFalse(validation.validate_priority('hig'))
        # priority is invalid
        self.assertFalse(validation.validate_priority(1))
        # priotity is valid
        self.assertTrue(validation.validate_priority("high_throughput"))

    def test_validate_radio_index(self):
        """
        Test cases for validate_radio_index(radio_index)
        """
        # radio index is valid
        self.assertTrue(validation.validate_radio_index("0"))
        # radio index is valid
        self.assertTrue(validation.validate_radio_index("1"))
        # radio index is valid
        self.assertTrue(validation.validate_radio_index("2"))
        # radio index is invalid
        self.assertTrue(validation.validate_radio_index("3"))
        # radio index is invalid
        self.assertFalse(validation.validate_radio_index("-1"))
        self.assertFalse(validation.validate_radio_index(-1))

    def test_validate_mesh_vif(self):
        """
        Test cases for validate_mesh_vif(mesh_vif)
        """
        # mesh vif is valid
        self.assertTrue(validation.validate_mesh_vif("wlp2s0"))
        # mesh vif is valid
        self.assertTrue(validation.validate_mesh_vif("wlp3s0"))
        # mesh vif is valid
        self.assertTrue(validation.validate_mesh_vif("halow1"))
        # mesh vif is valid
        self.assertTrue(validation.validate_mesh_vif("wlp2s1"))
        # mesh vif is valid
        self.assertTrue(validation.validate_mesh_vif("wlp3s1"))
        # mesh vif is valid
        self.assertTrue(validation.validate_mesh_vif("halow2"))
        # mesh vif is invalid
        self.assertFalse(validation.validate_mesh_vif("lan1"))
        # mesh vif is invalid
        self.assertFalse(validation.validate_mesh_vif("eth0"))
        # mesh vif is invalid
        self.assertFalse(validation.validate_mesh_vif("usb0"))
        # mesh vif is invalid
        self.assertFalse(validation.validate_mesh_vif(0))

    def test_validate_batman_iface(self):
        """
        Test cases for validate_batman_iface(batman_iface)
        """
        # batman iface is valid
        self.assertTrue(validation.validate_batman_iface("bat0"))
        # batman iface is valid
        self.assertTrue(validation.validate_batman_iface("bat1"))
        # batman iface is valid
        self.assertTrue(validation.validate_batman_iface("bat2"))
        # batman iface is invalid
        self.assertFalse(validation.validate_batman_iface("3"))
        # batman iface is invalid
        self.assertFalse(validation.validate_batman_iface(0))





if __name__ == '__main__':
    unittest.main()
