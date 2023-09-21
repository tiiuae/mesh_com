"""
This module implements provisioning agent whose main purpose
is to obtain authentication certificate from the provisioning
server.

Attributes:
    server (str) -- Provisioning server IP.
    port (str) -- Server port number.
    outdir (str) -- Output directory where to store CSR file.

Usage:
    $ python comms_provisioning.py -s 127.0.0.1 -p 80 -o /mydir

"""

import argparse
import asyncio
import os
import requests
import time
from hw_control import LedControl


from cryptography.x509 import load_pem_x509_certificate

import comms_hsm_controller


class CommsProvisioning:
    """
    Comms Provisioning class.
    """
    def __init__(self, server: str = "localhost", port: str = "80", outdir: str = "/opt") -> None:
        self.__session = None
        self.__outdir = outdir
        self.__device_id_file = self.__outdir + "/identity"
        self.__client_csr_file = self.__outdir + "/csr/client_csr.csr"
        self.__server_csr_file = self.__outdir + "/csr/server_csr.csr"
        self.__server_url = "http://" + server + ":" + port + "/api/mesh/provision"
        self.__device_id = self.__get_device_id()
        self.__pcb_version = self.__get_comms_pcb_version("/opt/hardware/comms_pcb_version")

        self.__auth_key_id = "99887766"
        self.__auth_key_label = "CommsDeviceAuth"
        self.__server_cert_label = "CommsServerCert"
        self.__hsm_ctrl = comms_hsm_controller.CommsHSMController(
            self.__outdir, self.__pcb_version)
        self.__session = self.__hsm_ctrl.open_session()

    def close_session(self):
        """
        Close HSM controller session.
        """
        if self.__session:
            self.__hsm_ctrl.close_session()
            self.__session = None

    def __del__(self):
        self.close_session()

    def __get_device_id(self):
        try:
            with open(self.__device_id_file, 'r', encoding='utf-8') as file:
                device_id = file.readline()
                return device_id
        except FileNotFoundError:
            # FIXME: temporary for testing
            return "None"

    def __get_comms_pcb_version(self, file_path):
        version = 0
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                content = file.read()
                for line in content.splitlines():
                    if line.startswith("COMMS_PCB_VERSION="):
                        version = line.split("=")[1]
                        return float(version)
        except FileNotFoundError:
            return float(version)

    def do_provisioning(self):
        """
        Creates and sends a CSR to the provisioning server.

        Returns:
            bool: True if provisioning is ready, False otherwise.
        """
        if self.__hsm_ctrl.get_certificate(self.__auth_key_id, self.__auth_key_label) is None or \
           self.__hsm_ctrl.get_certificate(self.__auth_key_id, self.__server_cert_label) is None:
            if not self.__hsm_ctrl.has_private_key(self.__auth_key_id, self.__auth_key_label):
                self.__hsm_ctrl.generate_rsa_keypair_via_openssl(self.__auth_key_id,
                                                                 self.__auth_key_label)
                """
                self.__hsm_ctrl.generate_ec_keypair_via_openssl(self.__auth_key_id,
                                                                 self.__auth_key_label)
                self.__hsm_ctrl.generate_rsa_keypair(self.__auth_key_id, self.__auth_key_label)
                self.__hsm_ctrl.generate_ec_keypair(self.__auth_key_id, self.__auth_key_label)
                """

            # Create certificate signing request to get client certificate
            """
            client_csr_created = self.__hsm_ctrl.create_csr_via_openssl(priv_key_id=self.__auth_key_id,
                                                                        device_id=self.__device_id,
                                                                        filename=self.__client_csr_file)
            """
            client_csr_created = self.__hsm_ctrl.create_csr(priv_key_id=self.__auth_key_id,
                                                            device_id=self.__device_id,
                                                            filename=self.__client_csr_file)
            if not client_csr_created:
                print("Problem creating client CSR")
                return False
            else:
                client_req_status = self.__request_client_certificate()
                if not client_req_status:
                    print("Problem getting client certificate")
                    return False

            # Create certificate signing request to get server certificate
            server_csr_created = self.__hsm_ctrl.create_csr_via_openssl(priv_key_id=self.__auth_key_id,
                                                                        device_id=self.__device_id,
                                                                        filename=self.__server_csr_file,
                                                                        is_server=True)
            # server_csr_created = self.__hsm_ctrl.create_csr(priv_key_id=self.__auth_key_id,
            #                                                device_id=self.__device_id,
            #                                                filename=self.__server_csr_file,
            #                                                server=True)
            if not server_csr_created:
                print("Problem creating server CSR")
                return False
            else:
                server_req_status = self.__request_server_certificate()
                if not server_req_status:
                    print("Problem getting server certificate")
                    return False
        else:
            return True

    def __request_client_certificate(self):
        with open(self.__client_csr_file, 'rb') as file:
            csr = file.read()
        try:
            # Post the CSR to the signing server
            url = self.__server_url
            headers = {"Content-Type": "application/json"}
            payload = {"csr": csr.decode("utf-8")}
            response = requests.post(url, headers=headers, json=payload, timeout=5)
            response.raise_for_status()
        except requests.exceptions.RequestException as e:
            print(f"Connection error: {e}")
            return False

        # Extract the signed certificate from the response
        response_json = response.json()
        signed_certificate_data = response_json["certificate"]
        ca_certificate = response_json["caCertificate"]

        # Save received certificates into filesystem
        certificate = "/etc/ssl/certs/comms_auth_cert.pem"
        root_certificate = "/etc/ssl/certs/root-ca.cert.pem"

        # Create directories if they don't exist
        os.makedirs(os.path.dirname(certificate), exist_ok=True)

        # Open the file in write mode and write the PEM data
        try:
            with open(certificate, "w") as file:
                file.write(signed_certificate_data)
            with open(root_certificate, "w") as file:
                file.write(ca_certificate)
        except Exception as e:
            print(f"Error saving certificate files: {str(e)}")

        # Save certificates to HSM:
        # Provisioning server response "certificate" contains two
        # certificates i.e. the device id specific client certificate
        # and the provisioning CA certificate.
        # => Split the PEM data into individual certificates
        signed_certificates = signed_certificate_data.split('-----END CERTIFICATE-----\n')

        # Remove any empty strings from the split operation
        signed_certificates = [cert.strip() for cert in signed_certificates if cert.strip()]

        for index, cert_data in enumerate(signed_certificates):
            cert_data = cert_data + "\n-----END CERTIFICATE-----\n"

            try:
                # Read certificate as PEM format
                certificate = load_pem_x509_certificate(cert_data.encode("utf-8"))
            except Exception as e:
                print("Could not load certificate at index {index}", e)
                return False

            device_id = self.__device_id
            cert_name = self.__auth_key_label

            if device_id in certificate.subject.rfc4514_string():
                label = cert_name
            else:
                label = cert_name + " Issuer " + str(index)
            saved = self.__hsm_ctrl.save_certificate(certificate, self.__auth_key_id, label)
            if not saved:
                print("Failed to store certificate with label {label} in index {index}")
                return False

        try:
            # Save root CA certificate
            certificate = load_pem_x509_certificate(ca_certificate.encode("utf-8"))
        except Exception as e:
            print("Could not load root CA certificate", e)
            return False
        else:
            label = "Root CA Certificate"
            saved = self.__hsm_ctrl.save_certificate(certificate, self.__auth_key_id, label)
            return saved

    def __request_server_certificate(self):
        with open(self.__server_csr_file, 'rb') as file:
            csr = file.read()
        try:
            # Post the CSR to the signing server
            url = self.__server_url
            headers = {"Content-Type": "application/json"}
            payload = {"csr": csr.decode("utf-8")}
            response = requests.post(url, headers=headers, json=payload, timeout=5)
            response.raise_for_status()
        except requests.exceptions.RequestException as e:
            print(f"Connection error: {e}")
            return False

        # Extract the signed certificate from the response
        response_json = response.json()
        signed_certificate_data = response_json["certificate"]

        # Save received certificates into filesystem
        certificate = "/etc/ssl/certs/comms_server_cert.pem"

        # Create directories if they don't exist
        os.makedirs(os.path.dirname(certificate), exist_ok=True)

        # Open the file in write mode and write the PEM data
        try:
            with open(certificate, "w") as file:
                file.write(signed_certificate_data)
        except Exception as e:
            print(f"Error saving server certificate file: {str(e)}")

        # Save certificate to HSM:
        signed_certificates = signed_certificate_data.split('-----END CERTIFICATE-----\n')

        # Remove any empty strings from the split operation
        signed_certificates = [cert.strip() for cert in signed_certificates if cert.strip()]

        for index, cert_data in enumerate(signed_certificates):
            cert_data = cert_data + "\n-----END CERTIFICATE-----\n"

            try:
                # Read certificate as PEM format
                certificate = load_pem_x509_certificate(cert_data.encode("utf-8"))
            except Exception as e:
                print("Could not load certificate at index {index}", e)
                return False

            device_id = self.__device_id
            cert_name = self.__server_cert_label

            if device_id in certificate.subject.rfc4514_string():
                label = cert_name
            else:
                label = cert_name + " Issuer " + str(index)
            saved = self.__hsm_ctrl.save_certificate(certificate, self.__auth_key_id, label)
            if not saved:
                print("Failed to store certificate with label {label} in index {index}")
                return False
        return saved


async def main(server, port, outdir, timeout):
    """
    main
    """
    led_status = LedControl()
    led_status.provisioning_led_control("start")
    # start time of provisioning
    start_time = time.time()

    prov_agent = CommsProvisioning(server, port, outdir)

    while True:
        led_status.provisioning_led_control("active")
        status = prov_agent.do_provisioning()
        if status:
            prov_agent.close_session()
            led_status.provisioning_led_control("stop")
            break

        led_status.provisioning_led_control("start")
        time.sleep(10)
        # if provisioning takes more than 30 seconds, break out
        if time.time() - start_time > int(timeout):
            led_status.provisioning_led_control("fail")
            break

        time.sleep(5)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Provisioning agent settings')
    parser.add_argument('-s', '--server', help='Provisioning Server IP', required=False)
    parser.add_argument('-p', '--port', help='Server port', required=False)
    parser.add_argument('-o', '--outdir', help='Output folder for files', required=False)
    parser.add_argument('-t', '--timeout', help='Timeout for provisioning trial', required=False)
    args = parser.parse_args()

    loop = asyncio.new_event_loop()
    loop.run_until_complete(main(args.server, args.port, args.outdir, args.timeout))
    loop.close()
