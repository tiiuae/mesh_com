#!/bin/bash

# Run the batctl command and capture the output
output=$(batctl meshif bat1 n 2>&1)  # Redirect stderr to stdout

# Check for error message
if echo "$output" | grep -q "Error - interface bat1 is not present or not a batman-adv interface"; then
    echo "Error: bat1 interface is not present or not a batman-adv interface."
    echo "Fail"
    exit 1
fi

# Count the number of lines in the output
line_count=$(echo "$output" | wc -l)

# Check if the line count is greater than 2 (header line + neighbor info)
if [ "$line_count" -gt 2 ]; then
    echo "Neighbor(s) found."
    echo "Pass"
else
    echo "No neighbor found."
    echo "Fail"
fi
