from OpenSSL import crypto

from utils.logging import get_logger


OID = '1.3.6.1.1.1.1.22'

oid_bytes = b''.join(int(n).to_bytes(length=1, byteorder='big') for n in OID.split('.'))
oid_bytes = (oid_bytes[0] * 40 + oid_bytes[1]).to_bytes(length=1, byteorder='big') + oid_bytes[2:]

logger = get_logger()


def get_mac_from_san(x509_cert: crypto.X509) -> str:
    try:
        ext_count = x509_cert.get_extension_count()
        for i in range(ext_count):
            x509_ext = x509_cert.get_extension(i)
            if b'subjectAltName' != x509_ext.get_short_name():
                continue
            san_bytes = x509_ext.get_data()
            if (idx := san_bytes.find(oid_bytes)) != -1:
                mac_idx = idx + len(oid_bytes) + 4
                mac_str = san_bytes[mac_idx:mac_idx + 17].decode("utf-8")
                return mac_str
    except Exception as e:
        logger.error(f"Exception when retrieving the MAC from cert SAN: {e}")
    return ''
