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
import time
import subprocess
import threading
import logging
from datetime import datetime
import requests
from cryptography.x509 import load_pem_x509_certificate

from comms_hsm_controller import CommsHSMController
from comms_hsm_controller import KeyParams
from comms_hsm_controller import CsrParams
from hw_control import LedControl
import comms_service_discovery

class CommsProvisioning:
    """
    Comms Provisioning class.
    """
    def __init__(self, server: str = "localhost", port: str = "80",
                 outdir: str = "/opt", algorithm: str = "rsa") -> None:
        self.__session = None
        self.__outdir = outdir
        self.__device_id_file = self.__outdir + "/identity"
        self.__client_csr_file = self.__outdir + "/csr/client_csr.csr"
        self.__server_csr_file = self.__outdir + "/csr/server_csr.csr"
        self.__server_api = "/api/mesh/provision"
        self.__server_url = "http://" + server + ":" + port + self.__server_api
        self.__server_available = False

        # Base logger for all provisioning related modules
        self.__main_logger = logging.getLogger("provisioning")
        self.__main_logger.setLevel(logging.DEBUG)
        log_formatter = logging.Formatter(
            fmt='%(asctime)s :: %(name)-18s :: %(levelname)-8s :: %(message)s')
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(log_formatter)
        self.__main_logger.addHandler(console_handler)

        # logger for this module and derived from main logger
        self.log = self.__main_logger.getChild("agent")

        self.__device_id = self.__get_device_id()
        self.__pcb_version = self.__get_comms_pcb_version("/opt/hardware/comms_pcb_version")
        self.__reference_date = datetime(2023, 9, 1)

        self.__auth_key_id = "99887766"
        self.__auth_key_label = "CommsDeviceAuth"
        self.__server_cert_label = "CommsServerCert"

        self.__crypto_files_location = "/etc/ssl"
        self.__client_cert_file = self.__crypto_files_location + "/certs/comms_auth_cert.pem"
        self.__server_cert_file = self.__crypto_files_location + "/certs/comms_server_cert.pem"
        self.__root_ca_file = self.__crypto_files_location + "/certs/root-ca.cert.pem"

        self.__key_params = KeyParams(
            id=self.__auth_key_id,
            label=self.__auth_key_label,
            private_key=self.__crypto_files_location + "/private/comms_auth_private_key.pem",
            public_key=self.__crypto_files_location + "/public/comms_auth_public_key.pem",
            algorithm=algorithm,
            use_openssl=True  # This is needed in order to write keys to filesystem.
        )

        self.service_monitor = comms_service_discovery.CommsServiceMonitor(
            service_name="TII Provisioning server",
            service_type="_provisioning._tcp.local.",
            service_cb=self.__provisioning_server_address_cb,
            logger=self.__main_logger,
        )
        self.__service_monitor_thread = threading.Thread(
            target=self.service_monitor.run
        )

        # Openssl needs to be used to create CSR with EC keys as SoftHSM signing
        # algorithm is not supported by the Provisioning Server.
        if self.__key_params.algorithm == "ec":
            csr_use_openssl = True
        else:
            csr_use_openssl = False

        self.__csr_params = CsrParams(
            key_id=self.__auth_key_id,
            device_id=self.__device_id,
            filename=self.__client_cert_file,
            use_openssl=csr_use_openssl
        )
        self.__hsm_ctrl = CommsHSMController(self.__outdir, self.__pcb_version,
                                             self.__main_logger.getChild("hsm_control"))
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
            # FIXME: temporary for workstatiom testing
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

    def __provisioning_server_address_cb(self, address: str, status: bool, **kwargs) -> None:
        """
        Callback for provisionign server address
        :param address: Provisionign server address
        :param status: Provisioning server connection status
        :return: -
        """
        self.__server_url = "http://" + address + self.__server_api
        self.__server_available = status

    def start_service_discovery(self):
        """
        Starts service monitor thread
        :param: -
        :return: -
        """
        if not self.service_monitor.running:
            self.log.debug("Start service monitor thread")
            self.__service_monitor_thread.start()

    def stop_service_discovery(self):
        """
        Stops service monitor thread
        :param: -
        :return: -
        """
        if self.service_monitor.running:
            self.log.debug("Stop service monitor thread")
            self.service_monitor.close()
            self.__service_monitor_thread.join()

    def do_provisioning(self):
        """
        Creates and sends a CSR to the provisioning server.

        Returns:
            bool: True if provisioning is ready, False otherwise.
        """
        client_certificate = self.__hsm_ctrl.get_certificate(self.__auth_key_id, self.__auth_key_label)
        server_certificate = self.__hsm_ctrl.get_certificate(self.__auth_key_id, self.__server_cert_label)

        if datetime.now() < self.__reference_date and client_certificate is not None:
            # Use client certificate to set system time
            self.__update_system_time(client_certificate.validity_start)

        if client_certificate is None or server_certificate is None:
            if not self.__hsm_ctrl.has_private_key(self.__auth_key_id, self.__auth_key_label):
                self.__hsm_ctrl.generate_keypair(self.__key_params)

            # Create certificate signing request to get client certificate
            self.__csr_params.is_server = False
            self.__csr_params.filename = self.__client_csr_file
            client_csr_created = self.__hsm_ctrl.create_csr(self.__csr_params)
            if not client_csr_created:
                self.log.debug("Problem creating client CSR")
                return False
            if self.__server_available:
                client_req_status = self.__request_client_certificate()
                if not client_req_status:
                    self.log.debug("Problem getting client certificate")
                    return False

            # Create certificate signing request to get server certificate
            self.__csr_params.is_server = True
            self.__csr_params.filename = self.__server_csr_file
            server_csr_created = self.__hsm_ctrl.create_csr(self.__csr_params)
            if not server_csr_created:
                self.log.debug("Problem creating server CSR")
                return False
            if self.__server_available:
                server_req_status = self.__request_server_certificate()
                if not server_req_status:
                    self.log.debug("Problem getting server certificate")
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
            self.log.debug("Connection error: %s", e)
            return False

        # Extract the signed certificate from the response
        response_json = response.json()
        signed_certificate_data = response_json["certificate"]
        ca_certificate = response_json["caCertificate"]

        # Save certificates into filesystem. Note! This is temporary
        # until all modules are able to use keys and certificates via HSM.
        os.makedirs(os.path.dirname(self.__client_cert_file), exist_ok=True)
        try:
            with open(self.__client_cert_file, "w") as file:
                file.write(signed_certificate_data)
            with open(self.__root_ca_file, "w") as file:
                file.write(ca_certificate)
        except Exception as e:
            self.log.debug("Error saving certificate files: %s", e)

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
                self.log.debug("Could not load certificate at index %d, err: %s", index, e)
                return False

            device_id = self.__device_id
            cert_name = self.__auth_key_label

            if device_id in certificate.subject.rfc4514_string():
                label = cert_name
            else:
                label = cert_name + " Issuer " + str(index)
            saved = self.__hsm_ctrl.save_certificate(certificate, self.__auth_key_id, label)
            if not saved:
                self.log.debug("Failed to store certificate with label %s in index %d", label, index)
                return False

        try:
            # Save root CA certificate
            certificate = load_pem_x509_certificate(ca_certificate.encode("utf-8"))
        except Exception as e:
            self.log.debug("Could not load root CA certificate, %s", e)
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
            self.log.debug("Connection error: %s", e)
            return False

        # Extract the signed certificate from the response
        response_json = response.json()
        signed_certificate_data = response_json["certificate"]

        # Save certificate into filesystem. Note! This is temporary
        # until all modules are able to use keys and certificates via HSM.
        os.makedirs(os.path.dirname(self.__server_cert_file), exist_ok=True)
        try:
            with open(self.__server_cert_file, "w") as file:
                file.write(signed_certificate_data)
        except Exception as e:
            self.log.debug("Error saving server certificate file: %s", e)

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
                self.log.debug("Could not load certificate at index %d, err: %s", index, e)
                return False

            device_id = self.__device_id
            cert_name = self.__server_cert_label

            if device_id in certificate.subject.rfc4514_string():
                label = cert_name
            else:
                label = cert_name + " Issuer " + str(index)
            saved = self.__hsm_ctrl.save_certificate(certificate, self.__auth_key_id, label)
            if not saved:
                self.log.debug("Failed to store certificate with label %s in index %d", label, index)
                return False
        return saved

    def __update_system_time(self, new_datetime):
        new_time_str = new_datetime.strftime("%Y-%m-%d %H:%M:%S")
        subprocess.run(["date", "-s", new_time_str])
        subprocess.run(["hwclock", "--systohc"])


async def main(server, port, outdir, timeout=None, algorithm=None):
    """
    main
    """
    led_status = LedControl()
    led_status.provisioning_led_control("start")
    # start time of provisioning
    start_time = time.time()

    if timeout is None:
        timeout = 0
    if algorithm is None:
        algorithm = "rsa"
    if server is None:
        server = "localhost"
    if port is None:
        port = "80"
    if outdir is None:
        outdir = "/opt"

    prov_agent = CommsProvisioning(server, port, outdir, algorithm)
    prov_agent.start_service_discovery()

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
    prov_agent.stop_service_discovery()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Provisioning agent settings')
    parser.add_argument('-s', '--server', help='Provisioning Server IP', required=False)
    parser.add_argument('-p', '--port', help='Server port', required=False)
    parser.add_argument('-o', '--outdir', help='Output folder for files', required=False)
    parser.add_argument('-t', '--timeout', help='Timeout for provisioning trial', required=False)
    parser.add_argument('-a', '--alg', help='Keypair algorithm (rsa or ec)', required=False)
    args = parser.parse_args()

    loop = asyncio.new_event_loop()
    loop.run_until_complete(main(args.server, args.port, args.outdir, args.timeout, args.alg))
    loop.close()
