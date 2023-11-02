import datetime
from unittest import TestCase, mock
import pytest
import logging
import OpenSSL
logger = logging.getLogger("Test")
import add_syspath
from tools.verification_tools import verify_cert, CertificateDifferentCN, CertificateExpiredError, _verify_certificate_chain, validation

# Get the current date and time
current_date = datetime.datetime.now()

# Format the current date and time to match the format you desire
not_after = (current_date + datetime.timedelta(days=365 * 80)).strftime('%Y%m%d%H%M%SZ')  # 80 years from now
not_before = (current_date - datetime.timedelta(days=365 * 2)).strftime('%Y%m%d%H%M%SZ')  # 2 years ago

# Mock OpenSSL certificate object
mock_cert_obj = mock.Mock()
mock_cert_obj.get_notAfter.return_value = not_after.encode()
mock_cert_obj.get_notBefore.return_value = not_before.encode()
mock_cert_obj.get_subject.return_value.CN = 'some_mac_address'

# Mock data
cert = b'some_cert_data'
ca_cert = 'path_to_ca_cert'
IPaddress = 'some_ipv6_address'


@mock.patch('tools.verification_tools._verify_certificate_chain', return_value=True)
@mock.patch('tools.verification_tools.OpenSSL.crypto.dump_publickey', return_value=b'some_public_key_data')
@mock.patch('tools.verification_tools.OpenSSL.crypto.load_certificate', return_value=mock_cert_obj)
@mock.patch('tools.verification_tools.extract_mac_from_ipv6', return_value='some_mac_address')
def test_verify_cert_success(mock_extract_mac, mock_load_certificate, mock_dump_publickey, mock_verify_chain):
    assert verify_cert(cert, ca_cert, IPaddress, mock.Mock()) is True


# Sample of ca cert
ca_cert_pem = b"""-----BEGIN CERTIFICATE-----
MIIBcTCCARegAwIBAgIUe3U5E1wplyB8SQ9XVAIJrCAPp74wCgYIKoZIzj0EAwIw
DjEMMAoGA1UEAwwDVElJMB4XDTIzMTAyNzA5MjQxM1oXDTI0MTAyNjA5MjQxM1ow
DjEMMAoGA1UEAwwDVElJMFkwEwYHKoZIzj0CAQYIKoZIzj0DAQcDQgAEb0ey66LW
i4IbPAXcAXg5sp56HXGLemQVGfTiE1Nlwxzu/PTOybOUZe70ssceR1gZo6YNtywj
HXMG+GA0f1+OOaNTMFEwHQYDVR0OBBYEFDgm8kYZpzMYxPJQmcFey89qzaoCMB8G
A1UdIwQYMBaAFDgm8kYZpzMYxPJQmcFey89qzaoCMA8GA1UdEwEB/wQFMAMBAf8w
CgYIKoZIzj0EAwIDSAAwRQIhAJ+eESqOmyJikkNshd0vE6GcgG/sXNyi94i9eoJe
0xi1AiBbAN95OGIsDC6kXm03l26kjGOEbTKMtfw98m1+SjjyXg==
-----END CERTIFICATE-----"""

# Sample of ca signed certificate
cert_pem = b"""-----BEGIN CERTIFICATE-----
MIIBJDCBywIUSPf4jQ/6c2+pkOrQT+dd84Brk38wCgYIKoZIzj0EAwIwDjEMMAoG
A1UEAwwDVElJMB4XDTIzMTAyNzA5MjQzMloXDTI0MTAyNjA5MjQzMlowHDEaMBgG
A1UEAwwRMDQ6ZjA6MjE6OWU6NmI6MzkwWTATBgcqhkjOPQIBBggqhkjOPQMBBwNC
AAR9f+vKjysitibhyyVdXgHfszGmTnigLPo2jMikw6YvM78C3LBrXie4WGEKMxBU
1oZHrit1bf/UtKYJR7KjuDLtMAoGCCqGSM49BAMCA0gAMEUCIQDKbKoESzjZqbeY
W9hUVtMr6tBeOeQBu4k1Ob6yBI4gowIgZDMOUsItvJn1CmHGxT/q8FdPEG5/w4qL
a/RrY89BYTc=
-----END CERTIFICATE-----"""


@mock.patch('builtins.open', mock.mock_open(read_data=ca_cert_pem))
def test_verify_certificate_chain_pass():
    # Convert the certificate from PEM to DER format
    x509 = OpenSSL.crypto.load_certificate(OpenSSL.crypto.FILETYPE_PEM, cert_pem)
    cert_der = OpenSSL.crypto.dump_certificate(OpenSSL.crypto.FILETYPE_ASN1, x509)

    # Run the function under test with valid certificate
    result = _verify_certificate_chain(cert_der, 'path_to_ca_cert', mock.MagicMock())

    # Assertions
    assert result

@mock.patch('builtins.open', mock.mock_open(read_data=ca_cert_pem))
def test_verify_certificate_chain_failure():
    # Run the function under test with fake certificate
    result = _verify_certificate_chain("Fake certificate", 'path_to_ca_cert', mock.MagicMock())

    # Assertions
    assert not result


@mock.patch('tools.verification_tools.OpenSSL.crypto.load_certificate', return_value=mock_cert_obj)
@mock.patch('tools.verification_tools.extract_mac_from_ipv6', return_value='different_mac_address')
def test_certificate_different_cn(mock_extract_mac, mock_load_certificate):
    with pytest.raises(CertificateDifferentCN): # Checking that exception is raised
        validation(cert, ca_cert, IPaddress, mock.Mock())
    assert not verify_cert(cert, ca_cert, IPaddress, mock.Mock()) # Assert that certificate verification fails


@mock.patch('tools.verification_tools.OpenSSL.crypto.load_certificate', return_value=mock_cert_obj)
def test_certificate_expired(mock_load_certificate):
    # Change the 'notAfter' value to a date in the past
    mock_cert_obj.get_notAfter.return_value = (current_date - datetime.timedelta(days=1)).strftime('%Y%m%d%H%M%SZ').encode()
    with pytest.raises(CertificateExpiredError): # Checking that exception is raised
        validation(cert, ca_cert, IPaddress, mock.Mock())
    assert not verify_cert(cert, ca_cert, IPaddress, mock.Mock()) # Assert that certificate verification fails

# Remove log directory during teardown
import shutil
import os
def remove_logs_directory():
    logs_directory = 'logs'
    if os.path.exists(logs_directory) and os.path.isdir(logs_directory):
        shutil.rmtree(logs_directory)

def teardown_module():
    remove_logs_directory()