#!/bin/bash

# Run the command and capture the output
output=$(ip macsec show 2>&1)

# Check for error in the output
if echo "$output" | grep -q -i "error"; then
    echo "Error detected in 'ip macsec show' output"
    echo "Fail"
    exit 1
fi

# Function to check for TXSC and RXSC
check_txsc_rxsc() {
    local interface=$1
    local output=$2

    # Extract the relevant block for the interface
    local interface_block=$(echo "$output" | awk -v iface="$interface:" '$0 ~ iface {flag=1; next} /offload:/ {flag=0} flag')

    # Check for TXSC and RXSC in the extracted block
    if echo "$interface_block" | grep -q "TXSC" && echo "$interface_block" | grep -q "RXSC"; then
        return 0 # Found both TXSC and RXSC
    else
        return 1 # Missing TXSC or RXSC
    fi
}

# Function to check interfaces with a specific prefix
check_interfaces_with_prefix() {
    local prefix=$1
    local output="$2"
    local fail=0
    local found=0

    # Extract interfaces starting with the prefix
    local interfaces=$(echo "$output" | grep -oE "${prefix}[[:alnum:]]+:" | tr -d ':')

    for interface in $interfaces; do
        found=1
        if ! check_txsc_rxsc "$interface" "$output"; then
            echo "Interface $interface does not have both TXSC and RXSC"
            fail=1
        fi
    done

    if [[ $found -eq 0 ]]; then
        echo "No interface found with prefix $prefix"
        fail=1
    fi

    return $fail
}

# Check for lms* and ums* interfaces
if ! check_interfaces_with_prefix "lms" "$output"; then
    echo "Fail"
    exit 1
fi

if ! check_interfaces_with_prefix "ums" "$output"; then
    echo "Fail"
    exit 1
fi

# If script reaches this point, all checks passed
echo "Pass"
