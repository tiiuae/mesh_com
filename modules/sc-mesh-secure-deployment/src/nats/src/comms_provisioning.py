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
import requests

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
        self.__csr_file = self.__outdir + "/csr/prov_csr.csr"
        self.__server_url = "http://" + server + ":" + port + "/api/devices/provision"
        self.__device_id = self.__get_device_id()
        self.__pcb_version = self.__get_comms_pcb_version("/opt/hardware/comms_pcb_version")

        self.__auth_key_id = "99887766"
        self.__auth_key_label = "CommsDeviceAuth"
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
        if self.__hsm_ctrl.get_certificate(self.__auth_key_id, self.__auth_key_label) is None:
            if not self.__hsm_ctrl.has_private_key(self.__auth_key_id, self.__auth_key_label):
                self.__hsm_ctrl.generate_rsa_keypair(self.__auth_key_id, self.__auth_key_label)
                # self.__hsm_ctrl.generate_ec_keypair(self.__auth_key_id, self.__auth_key_label)

            # Create certificate signing request
            # csr_created = self.__hsm_ctrl.create_csr_via_openssl(priv_key_id=self.__auth_key_id,
            #                                                     device_id=self.__device_id,
            #                                                     filename=self.__csr_file)
            csr_created = self.__hsm_ctrl.create_csr(priv_key_id=self.__auth_key_id,
                                                     device_id=self.__device_id,
                                                     filename=self.__csr_file)
            if csr_created:
                # TODO: Works with RSA key only for now.
                # EC key needs to be tested once with such
                # provisioning server that uses EC keys also.
                req_status = self.__request_certificate()
                return req_status
            else:
                return False
        else:
            return True

    def __request_certificate(self):
        with open(self.__csr_file, 'rb') as file:
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
        ca_certificate = response_json["ca_certificate"]

        print("### Printing received certificates from server: ###")
        print(signed_certificate_data)
        print("### Finished printing received certificates from server: ###")

        print("### Printing root CA certificate from server ###")
        print(ca_certificate)
        print("### Finished printing CA certificate from server ###")

        # Provisioning server response "certificate" contains two
        # certificates i.e. the device id specific client certificate
        # and the provisioning CA certificate.
        # => Split the PEM data into individual certificates
        signed_certificates = signed_certificate_data.split('-----END CERTIFICATE-----\n')

        # Remove any empty strings from the split operation
        signed_certificates = [cert.strip() for cert in signed_certificates if cert.strip()]

        # Save certificates
        for index, cert_data in enumerate(signed_certificates):
            cert_data = cert_data + "\n-----END CERTIFICATE-----\n"

            try:
                # Read certificate as PEM format
                certificate = load_pem_x509_certificate(cert_data.encode("utf-8"))
            except Exception as e:
                print("Could not load certificate at index {index}", e)
                return False

            device_id = self.__device_id
            cert_name = self.__auth_key_label + " Certificate"

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


async def main(server, port, outdir):
    """
    main
    """
    prov_agent = CommsProvisioning(server, port, outdir)

    while True:
        status = prov_agent.do_provisioning()
        if status:
            prov_agent.close_session()
            break
        await asyncio.sleep(10)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Provisioning agent settings')
    parser.add_argument('-s', '--server', help='Provisioning Server IP', required=False)
    parser.add_argument('-p', '--port', help='Server port', required=False)
    parser.add_argument('-o', '--outdir', help='Output folder for files', required=False)
    args = parser.parse_args()

    loop = asyncio.new_event_loop()
    loop.run_until_complete(main(args.server, args.port, args.outdir))
    loop.close()
