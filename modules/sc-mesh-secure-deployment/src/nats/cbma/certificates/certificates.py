from datetime import datetime

from OpenSSL import crypto

from models.certificates import ICertificate


class OpenSSLCertificate(ICertificate):
    def __init__(self, cert_path: str) -> None:
        with open(cert_path, 'rb') as file:
            self.x509_cert: crypto.X509 = crypto.load_certificate(crypto.FILETYPE_PEM, file.read())

        self.extensions: list[crypto.X509Extension] = []
        for ext_id in range(self.x509_cert.get_extension_count()):
            self.extensions.append(self.x509_cert.get_extension(ext_id))


    @property
    def certificate(self) -> crypto.X509:
        return self.x509_cert


    def __get_extension_by_name(self, name: str) -> crypto.X509Extension:
        encoded_name: bytes = name.encode('utf-8')
        for extension in self.extensions:
            if encoded_name in extension.get_short_name():
                return extension
        raise ValueError(f"Failed to get the extension {name.capitalize()}")


    def get_subject_key_identifier(self) -> bytes:
        skid = self.__get_extension_by_name('subjectKeyIdentifier')
        return skid.get_data()


    def get_authority_key_identifier(self) -> bytes:
        akid = self.__get_extension_by_name('authorityKeyIdentifier')
        return akid.get_data()


    def get_end_date(self) -> datetime:
        if expiration_bytes := self.x509_cert.get_notAfter():
            return datetime.strptime(expiration_bytes.decode('ascii'), '%Y%m%d%H%M%SZ')
        raise ValueError("Certificate doesn't have expiration date")
