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

    assert '64:73:81:83:B3:F1:72:AD:28:6D:BE:9B:0F:A1:D9:D4:7F:75:72:C2' == str(skid)
    assert '04:14:64:73:81:83:b3:f1:72:ad:28:6d:be:9b:0f:a1:d9:d4:7f:75:72:c2' == skid.hex(':')
    assert '2024-05-13 09:16:24' == str(end_data)
    assert ('keyid:64:73:81:83:B3:F1:72:AD:28:6D:BE:9B:0F:A1:D9:D4:7F:75:72:C2\n'
            'DirName:/CN=Stop-Gap Insecure CA\n'
            'serial:13:51:37:43:26:0A:A9:DE:32:FD:1C:EF:18:F2:9B:E7:A6:16:54:EF') == str(akid)
