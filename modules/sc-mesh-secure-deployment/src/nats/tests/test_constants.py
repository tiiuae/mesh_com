"""
Main purpose to test immutable values in Constants class
"""

import unittest
from src.constants import Constants, ConfigType, StatusType

class TestConstants(unittest.TestCase):

    def test_constants_values_are_immutable(self):
        with self.assertRaises(AttributeError):
            Constants.YAML_FILE.value = "new_value"
            ConfigType.MESH_CONFIG.value = "new_value"
            StatusType.DOWNLOAD_MESH_CONFIG.value = "new_value"

    def test_constants_values_are_correct(self):
        assert Constants.YAML_FILE.value == "/opt/mesh_com/modules/sc-mesh-secure-deployment/src/2_0/features.yaml"
        assert Constants.DOWNLOADED_CBMA_UPPER_PATH.value == "/opt/certs/CBMA/UpperCBMA"
        assert Constants.DOWNLOADED_CBMA_LOWER_PATH.value == "/opt/certs/CBMA/LowerCBMA"
        assert Constants.DOWNLOADED_CBMA_BIRTHCERTS_PATH.value == "/opt/certs/CBMA/BirthCerts"
        assert Constants.GENERATED_CERTS_PATH.value == "/opt/crypto"

        assert ConfigType.MESH_CONFIG.value == "mesh_conf"
        assert ConfigType.BIRTH_CERTIFICATE.value == "birth_certificate"
        assert ConfigType.UPPER_CERTIFICATE.value == "upper_certificates"
        assert ConfigType.LOWER_CERTIFICATE.value == "lower_certificates"
        assert ConfigType.FEATURES.value == "features"
        assert ConfigType.DEBUG_CONFIG.value == "debug_conf"

        assert StatusType.DOWNLOAD_MESH_CONFIG.value == "download_mesh_config"
        assert StatusType.DOWNLOAD_FEATURES.value == "download_features"
        assert StatusType.DOWNLOAD_CERTIFICATES.value == "download_certificates"
        assert StatusType.UPLOAD_CERTIFICATES.value == "upload_certificates"