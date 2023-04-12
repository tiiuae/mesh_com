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
        # print ascii chars in the range of ascii code 32 to 126 (decimal)
        ssid = [chr(char) for char in range(32, 126)]
        self.assertTrue(validation.validate_ssid("".join(ssid)[0:32]))
        self.assertTrue(validation.validate_ssid("".join(ssid)[33:64]))
        self.assertTrue(validation.validate_ssid("".join(ssid)[65:96]))

    def test_validate_wpa3_psk(self):
        """
        Test cases for validate_wpa3_psk(psk)
        """
        # psk is valid
        self.assertTrue(validation.validate_wpa3_psk('12345678'))
        # psk is invalid (short)
        self.assertFalse(validation.validate_wpa3_psk('1234567'))
        # psk with invalid (long)
        self.assertFalse(validation.validate_wpa3_psk("""12345678901234567890123456789012345678901234567890123456789012345"""))

    def test_validate_ip_address(self):
        """
        Test cases for validate_ip_address(ip)
        """
        # ip is valid
        self.assertTrue(validation.validate_ip_address('10.10.10.10'))
        # ip is invalid
        self.assertFalse(validation.validate_ip_address('1000.0.0.0'))
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

if __name__ == '__main__':
    unittest.main()
