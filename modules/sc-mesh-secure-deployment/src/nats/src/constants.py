"""
Constants for the NATS module
"""
from enum import Enum

class ConfigType(str, Enum):
    """
    Config type
    """

    MESH_CONFIG: str = "mesh_conf"
    BIRTH_CERTIFICATE: str = "birth_certificate"
    UPPER_CERTIFICATE: str = "upper_certificates"
    LOWER_CERTIFICATE: str = "lower_certificates"
    FEATURES: str = "features"
    DEBUG_CONFIG: str = "debug_conf"


class StatusType(str, Enum):
    """
    Status type
    """

    # status
    DOWNLOAD_MESH_CONFIG: str = "download_mesh_config"
    DOWNLOAD_FEATURES: str = "download_features"
    DOWNLOAD_CERTIFICATES: str = "download_certificates"
    UPLOAD_CERTIFICATES: str = "upload_certificates"

# pylint: disable=too-few-public-methods, too-many-instance-attributes, disable=invalid-name
class Constants(Enum):
    """
    Constants class
    """

    RED_INTERFACE: str = "RED"
    BLACK_INTERFACE: str = "BLACK"
    WHITE_INTERFACE: str = "WHITE"

    # Root of the mesh_shield

    ROOT_PATH: str = "/opt"

    # Features YAML file
    YAML_FILE: str = (
        f"{ROOT_PATH}/mesh_com/modules/sc-mesh-secure-deployment/src/2_0/features.yaml"
    )

    # filebased certs and keys
    DOWNLOADED_CBMA_UPPER_PATH: str = f"{ROOT_PATH}/certs/CBMA/UpperCBMA"
    DOWNLOADED_CBMA_LOWER_PATH: str = f"{ROOT_PATH}/certs/CBMA/LowerCBMA"
    DOWNLOADED_CBMA_BIRTHCERTS_PATH: str = f"{ROOT_PATH}/certs/CBMA/BirthCerts"
    GENERATED_CERTS_PATH: str = f"{ROOT_PATH}/crypto"

    # CBMA
    CBMA_PORT_UPPER: int = 15002
    CBMA_PORT_LOWER: int = 15001

    # MDM agent
    GET_CONFIG: str = "public/config"
    GET_DEVICE_CONFIG: str = "public/get_device_config"  # + config_type
    PUT_DEVICE_CONFIG: str = "public/put_device_config"  # + config_type

    GET_DEVICE_CERTIFICATES: str = "public/get_device_certificates"
    PUT_DEVICE_CERTIFICATES: str = "public/put_device_certificates"

    OK_POLLING_TIME_SECONDS: int = 600
    FAIL_POLLING_TIME_SECONDS: int = 1
