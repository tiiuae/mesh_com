"""
for running this code, it is necessary to add the pub key for ssh on the other node.
Steps:
1)  ssh-keygen -t ecdsa
2) ssh-copy-id user@somedomain (to server)
"""
import subprocess
import socket
import os
import time
import glob
import shutil

# Constants
REMOTE_PATH = "/tmp/request"
if not os.path.exists(REMOTE_PATH):
    os.makedirs(REMOTE_PATH)

LOCAL_PATH = "certificates/"
if not os.path.exists(LOCAL_PATH):
    os.makedirs(LOCAL_PATH)

CUSTOM_PORT = 12345
CSR_SCRIPT_PATH = "./generate-csr.sh"


def run_command(command):
    try:
        return subprocess.run(command, shell=True, text=True, capture_output=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error running command: {command}. Error: {e.stderr}")
        exit(1)


def is_server_reachable(server_ip, port=CUSTOM_PORT):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(2)
            s.connect((server_ip, port))
        return True
    except (socket.timeout, ConnectionRefusedError):
        return False


def upload_file_to_server(local_file, server_ip, username, remote_path=REMOTE_PATH):
    scp_command = f"scp -i ~/.ssh/id_rsa {local_file} {username}@{server_ip}:{remote_path}"
    run_command(scp_command)


def are_files_received(csr_filename, path=REMOTE_PATH):
    # Paths for the required files
    ca_crt_path = os.path.join(path, "ca.crt")
    csr_crt_path = os.path.join(path, f"{os.path.splitext(csr_filename)[0]}.crt")

    # Check if both files exist
    return os.path.exists(ca_crt_path) and os.path.exists(csr_crt_path)


def main():
    server_ip = input("Enter the server IP address: ")
    username = input("Enter your username: ")

    if not is_server_reachable(server_ip):
        print("Error: The server is not reachable.")
        exit(1)

    run_command(CSR_SCRIPT_PATH)
    csr_filename = glob.glob("*.csr")[0]
    crt = csr_filename.split(".csr")[0]+".crt"
    upload_file_to_server(csr_filename, server_ip, username)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        client_socket.connect((server_ip, CUSTOM_PORT))
        message = f"CSR uploaded {csr_filename}".encode()
        client_socket.send(message)

        # This will allow the server some time to process and send the files. You can adjust the sleep time if needed.
        time.sleep(10)

        if are_files_received(csr_filename):
            print(f"The crt file ({crt}) and the ca.crt have been successfully received locally!")
            for f in glob.glob(f"{REMOTE_PATH}/*"):
                shutil.copy2(f, LOCAL_PATH)
            shutil.copy2(csr_filename.split(".csr")[0]+".key", LOCAL_PATH)
        else:
            print("Failed to confirm the receipt of the CSR file and ca.crt locally.")


if __name__ == "__main__":
    main()
