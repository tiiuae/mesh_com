#!/bin/bash

# Check if the script is being run by root
if [ "$(id -u)" -ne 0 ]; then
    echo "This script must be run with root privileges."
    exit 1
fi

# preconditions
if [ ! -f "$(pwd)/$(basename $0)" ]; then
    echo "Script is not being executed in the same folder"
    exit 1
fi

needed_apps="batctl swig"
for app in $needed_apps; do
    if ! command -v $app &> /dev/null
    then
        echo "$app is not installed. exit.  Tips: sudo apt-get install $app"
        exit 1
    fi
done

# python virtualenv
python3 -m venv unittest
source unittest/bin/activate

# install dependencies to virtualenv
pip install coverage==7.4.4  # this is for testing purpose
pip install -r requirements.txt

# List of files not to used for coverage calculation.
# Files tested elsewhere or not needed to be tested or not mesh shield content
not_used="batadvvis.py,batstat.py,fmo_agent.py,comms_nats_discovery.py,cbma/*,debug_tests/*,comms_mesh_telemetry.py,comms_interface_info.py"

# discover and run unittests
coverage run --omit="$not_used" -m unittest discover -v
REPORT=$(coverage report -m)

# print and save coverage report
echo "$REPORT" | tee coverage_report.txt
echo -e "Not tested files as not MDM content or tested elsewhere:\n $not_used" >> coverage_report.txt
# deactivate virtualenv
deactivate

# Clean up __pycache__ directories
find . -type d -name '__pycache__' -exec rm -rf {} +
# Clean up unittest venv
rm -rf unittest
# Clean up coverage tool's SQL database
rm -f .coverage
