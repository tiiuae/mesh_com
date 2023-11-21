import subprocess
import re
import csv

def get_mac_addresses():
    try:
        # Execute the 'batctl n' command and decode the output
        output = subprocess.check_output(['batctl', 'n']).decode('utf-8')
    except subprocess.CalledProcessError as e:
        print(f"Error executing batctl command: {e}")
        return []
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return []

    lines = output.split('\n')
    mac_addresses = []

    for line in lines:
        # Search for MAC addresses in each line
        match = re.search(r'\s+(\w{2}:\w{2}:\w{2}:\w{2}:\w{2}:\w{2})\s+', line)
        if match:
            mac_address = match.group(1)
            mac_addresses.append(mac_address)

    return mac_addresses

def save_to_csv(mac_addresses, filename):
    try:
        full_path = filename  # Assuming full path is provided
        with open(full_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['MAC Address'])
            for mac_address in mac_addresses:
                writer.writerow([mac_address])
    except IOError as e:
        print(f"Error writing to file: {e}")

# Retrieve MAC addresses and save them to a CSV file
try:
    mac_addresses = get_mac_addresses()
    if mac_addresses:
        save_to_csv(mac_addresses, './mac_addresses.csv')
    else:
        print("No MAC addresses found.")
except Exception as e:
    print(f"An error occurred: {e}")


