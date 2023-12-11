import subprocess
import time
import csv
import os
import glob

def execute_script(script_name, args=None):
    """Execute a script with optional arguments."""
    if args is None:
        args = []
    subprocess.Popen(["python3", script_name] + args)

def find_latest_csv(pattern):
    """Find the latest CSV file matching a given pattern."""
    csv_files = glob.glob('/tmp/' + pattern)
    if not csv_files:
        print("No CSV file found.")
        return None
    return max(csv_files, key=os.path.getmtime)

def process_csv(input_file, output_file):
    """Process CSV file and write to output."""
    
    input_path = os.path.join('./', input_file)
    output_path = os.path.join('./', output_file)
    with open(input_path, 'r') as csv_in, open(output_path, 'w', newline='') as csv_out:
        reader = csv.reader(csv_in)
        writer = csv.writer(csv_out)
        next(reader)  # Skip the header row

        transposed_rows = list(map(list, zip(*(extract_values(row) for row in reader))))
        writer.writerows(transposed_rows)

def extract_values(row):
    """Extract and process values from a row."""
    values_str = row[1]
    values_list = [value.strip() for value in values_str.split('\n') if value.strip()]
    return [int(value) for value in values_list if is_valid_number(value)]

def is_valid_number(value):
    """Check if a string represents a valid number."""
    return value.isdigit() or (value.startswith('-') and value[1:].isdigit())

def create_flag_file():
    """Create a flag file to signal the scripts to continue."""
    open('/tmp/run_flag.txt', 'w').close()

def delete_flag_file():
    """Delete the flag file to signal the scripts to stop."""
    if os.path.exists('/tmp/run_flag.txt'):
        os.remove('/tmp/run_flag.txt')

def main():
    create_flag_file()

    # Start mon_rssi.py for continuous RSSI monitoring
    execute_script("F_mon_rssi.py", ["-i", "wlp1s0", "-r", "1"])

    try:
        while True:
            # Periodically execute bat_mac_list_v1.py
            execute_script("bat_mac_list_v1.py")
            time.sleep(10)  # Adjust the sleep time as needed for periodic execution

            # Periodically check and process new CSV files
            latest_csv_file = find_latest_csv('rssi*.csv')
            if latest_csv_file:
                process_csv(latest_csv_file, 'output_processed.csv')
            time.sleep(5)  # Adjust the sleep time as needed for processing
    except KeyboardInterrupt:
        print("Stopping scripts and processing...")
    finally:
        delete_flag_file()

if __name__ == "__main__":
    main()

