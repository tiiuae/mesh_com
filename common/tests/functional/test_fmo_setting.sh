#!/bin/bash

YAML_FILE="/opt/mesh_com/modules/sc-mesh-secure-deployment/src/2_0/features.yaml"

# Function to extract a value from the YAML file
extract_features_value() {
    local key=$1    # key name searched
    local file=$2   # file name to search from
    # shellcheck disable=SC2155
    local value=$(grep "^${key}:" "$file" | sed "s/^${key}: //")

    # Check if the value is found
    if [ -z "$value" ]; then
        echo "not_found"  # Return false if the key is not found
    else
        echo "$value"
    fi
}

test1() {
    value=$(extract_features_value "FMO" $YAML_FILE)
    if [ "$value" != "not_found" ]; then
        echo "PASS"
    else
        echo "FAIL"
    fi
}

test2() {
    value=$(extract_features_value "FMO" $YAML_FILE)
    if [ "$value" == "true" ]; then
        echo "PASS"
    else
        echo "FAIL"
    fi
}

# Function to run a test and check its result
run_test() {
    test_name="$1"
    command_to_run="$2"
    expected_result="$3"
    echo "Running test: $test_name"
    echo "Command: $command_to_run"
    result=$(eval "$command_to_run")
    if [ "$result" == "$expected_result" ]; then
        echo "Test passed: $test_name"
    else
        echo "Test failed: $test_name"
        echo "Expected result: $expected_result"
        echo "Actual result: $result"
    fi
}

run_test "FMO setting available in features.yaml" "test1" "PASS"
run_test "FMO setting is set to true and enabled in features.yaml" "test2" "PASS"
