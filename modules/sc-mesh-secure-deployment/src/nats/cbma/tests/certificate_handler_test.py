from certificates.certificates import OpenSSLCertificate

cert_path = "/path/to/your/certificate.crt"

cert_handler = OpenSSLCertificate(cert_path)

subject_key_identifier = cert_handler.get_subject_key_identifier()
print("Subject Key Identifier:", subject_key_identifier)

authority_key_identifier = cert_handler.get_authority_key_identifier()
print("Authority Key Identifier:", authority_key_identifier)

end_date = cert_handler.get_end_date()
print("End Date:", end_date)
