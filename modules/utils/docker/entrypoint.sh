#! /bin/bash

#start mesh service if mesh provisoning is done
hw_platform=$(cat /proc/cpuinfo | grep Model | awk '{print $5}')
if [ $hw_platform == "Compute" ]; then
  if [ -f "/opt/S9011sMesh" ]; then
    echo "starting 11s mesh service"
    /opt/S9011sMesh start
    sleep 2
  fi
else
  if [ -f "/opt/S90mesh" ]; then
    echo "starting ibss mesh service"
    /opt/S90mesh start
    sleep 2
  fi
fi
