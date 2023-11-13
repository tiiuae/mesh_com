#!/bin/bash

# Check if the bridge "br-lan" exists
if ip link show br-lan &> /dev/null; then
  # Delete the bridge "br-lan" if it exists
  echo "Deleting bridge br-lan..."
  ip link set br-lan down
  brctl delbr br-lan
  echo "Bridge br-lan deleted."
else
  echo "Bridge br-lan does not exist."
fi

# Check if the batman interface "bat0" exists
if ip link show bat0 &> /dev/null; then
  # Delete the batman interface "bat0" if it exists
  echo "Deleting batman interface bat0..."
  ip link set bat0 down


  echo "Batman interface bat0 deleted."
else
  echo "Batman interface bat0 does not exist."
fi