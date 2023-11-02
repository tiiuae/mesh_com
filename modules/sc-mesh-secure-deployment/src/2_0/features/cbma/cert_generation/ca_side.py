"""
for running this code, it is necessary to add the pub key for ssh on the other node.
Steps:
1)  ssh-keygen -t ecdsa
2) ssh-copy-id user@somedomain (to server)
"""
# TODO: change to ntas
import os
import subprocess
import socket
import threading
import time
import shutil
import argparse
import sys

sys.path.insert(0, '../')
from tools.custom_logger import CustomLogger

logger_instance = CustomLogger("ca-side")
logger = logger_instance.get_logger()

# Create the CSR directory if it doesn't exist
csr_directory = "/tmp/request/"  # Update this with the actual path
if not os.path.exists(csr_directory):
    os.makedirs(csr_directory)


# Function to run a command and get its output
def run_command(command):
    try:
        return subprocess.run(command, shell=True, text=True, capture_output=True, check=True)
    except subprocess.CalledProcessError as e:
        logger.error(f"Error running command: {command}")
        logger.error(e.stderr)
        raise


# Function to generate certificates and send files
def generate_and_send_certificates(csr_filename, IPAddress):
    try:
        # Generate the certificate using the Bash script
        generate_cert_script = "./generate_certificates.sh"
        output = run_command(f"{generate_cert_script} {csr_filename} {csr_directory}")
        logger.info(output.stdout)
        # Rename certificate files
        crt = f"{os.path.splitext(csr_filename)[0]}.crt"
        crt_filename = crt.split(os.sep)[-1]
        shutil.copy(f"certificates/{crt_filename}", f"{csr_directory}{crt_filename}")
        ca_crt_filename = "ca.crt"
        shutil.copy(f"certificates/{ca_crt_filename}", f"{csr_directory}{ca_crt_filename}")

        # Send CA.crt and signed certificate (crt) back to the client via SCP
        files_to_send = [crt_filename, ca_crt_filename]

        for file_to_send in files_to_send:
            scp_command = f"scp {csr_directory}{file_to_send} root@{IPAddress}:{csr_directory}"
            run_command(scp_command)

        logger.info("Certificates sent successfully.")
    except Exception as e:
        logger.error(f"Error generating and sending certificates: {e}")
        raise

    # Clean up
    # os.remove(crt_filename)
    # os.remove(ca_crt_filename)


# Function to handle client connection
def handle_client(client_socket, IPaddress):
    try:
        # Receive the CSR filename from the client
        csr_filename = client_socket.recv(1024).decode()
        if "CSR uploaded " in csr_filename:
            csr_filename = csr_filename.split("CSR uploaded ")[1]
        # Acknowledge the filename
        client_socket.send(b"Filename received")

        # Verify the CSR filename in the monitoring directory
        full_path = os.path.join(csr_directory, csr_filename)
        if os.path.exists(full_path) and csr_filename.endswith(".csr") and csr_filename != '':
            generate_and_send_certificates(full_path, IPaddress)
            os.remove(full_path)  # Remove the processed CSR file
    except Exception as e:
        logger.error(f"Error handling client connection: {e}")


# Function to monitor a directory for new CSR files
def monitor_csr_directory(directory, existing_files):
    try:
        while True:
            for filename in os.listdir(directory):
                if filename.endswith(".csr") and filename not in existing_files:
                    full_path = os.path.join(directory, filename)
                    existing_files.add(filename)
            time.sleep(10)  # Adjust the interval as needed
    except Exception as e:
        logger.error(f"Error monitoring CSR directory: {e}")


def main():
    try:
        # Parse command-line arguments
        parser = argparse.ArgumentParser(description="Server for handling CSR files")
        parser.add_argument("--port", type=int, default=12345, help="Port to listen on")
        args = parser.parse_args()

        # Server configuration
        host = "0.0.0.0"
        port = args.port

        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind((host, port))
        server_socket.listen(1)

        logger.info("Server listening on port %d", port)

        # Start the CSR monitoring thread
        existing_files = set()
        monitor_thread = threading.Thread(target=monitor_csr_directory, args=(csr_directory, existing_files,))
        monitor_thread.start()

        while True:
            # Handle the client connection
            client_socket, client_address = server_socket.accept()
            logger.info("Accepted connection from %s", client_address)
            handle_client(client_socket, client_address[0])
            client_socket.close()

    except Exception as e:
        logger.error(f"Main loop error: {e}")


if __name__ == "__main__":
    main()
