from struct import pack

from OpenSSL import crypto

from utils.logging import get_logger


OID = '1.3.6.1.1.1.1.22'

oid_bytes = bytes(int(n) for n in OID.split('.'))
oid_bytes = pack('=b6s', oid_bytes[0] * 40 + oid_bytes[1], oid_bytes[2:])
# ^ OID encoding as per X.690 BER standard, section 8.19.4 - https://www.itu.int/rec/T-REC-X.690

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
                # 5 is the offset between the OID and the MAC corresponding to the ;UTF8: part
                mac_idx = idx + len(oid_bytes) + 4  # 4 here because len() adds +1 to the offset
                # 17 is the full length of the MAC including colons: AA:BB:CC:DD:EE:FF
                mac_str = san_bytes[mac_idx:mac_idx + 17].decode('utf-8')
                return mac_str
    except Exception as e:
        logger.error(f"Exception when retrieving MAC address from certificate's SAN: {e}")
    raise ValueError("Unable to find MAC address in the certificate's SAN")
