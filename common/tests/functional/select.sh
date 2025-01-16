#!/bin/bash

# Function to run multicast sender script
run_sender() {
    ./multicast_sender.sh
}

# Function to run multicast receiver script
run_receiver() {
    ./multicast_receiver.sh
}

# Prompt user for selection
echo "Select an option:"
echo "1. Run as Sender"
echo "2. Run as Receiver"
read -p "Enter your choice (1 or 2): " choice

# Check user's choice and run the appropriate script
case $choice in
    1) run_sender;;
    2) run_receiver;;
    *) echo "Invalid choice. Please enter 1 or 2.";;
esac

