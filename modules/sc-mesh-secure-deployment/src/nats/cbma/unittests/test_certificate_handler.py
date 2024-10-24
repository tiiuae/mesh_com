import os
import sys
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1]))

from certificates.certificates import OpenSSLCertificate

TEST_DATA_FOLDER_PATH: str = f'{Path(__file__).parents[0]}/test_data/certificates'
ca_file: str = os.path.join(TEST_DATA_FOLDER_PATH, "ca.crt")


def test_openssl_certificate() -> None:
    certificate: OpenSSLCertificate = OpenSSLCertificate(ca_file)

    skid = certificate.get_subject_key_identifier()
    assert skid

    akid = certificate.get_authority_key_identifier()
    assert akid

    end_data: datetime = certificate.get_end_date()
    assert end_data

    assert '04:14:75:05:b3:00:68:0b:9c:d0:c0:46:e5:b5:aa:58:cb:8b:44:a2:92:d5' == skid.hex(':')
    assert '2024-03-17 08:21:07' == str(end_data)
    assert '30:16:80:14:14:38:d5:36:b1:5d:ab:f3:30:83:23:f0:6f:5b:c7:bf:2b:51:f5:9c' == akid.hex(':')



