#!/bin/bash
#Francis 
if ! [ -x "$(type -P iperf3)" ]; then
  echo "ERROR: script requires iperf3"
  echo "For Debian and friends get it with 'apt-get install iperf3'"
  echo "If you have it, perhaps you don't have permissions to run it, try 'sudo $(basename $0)'"
  exit 1
fi

if [ "$#" -ne "2" ]; then
  echo "ERROR: script needs four arguments, where:"
  echo
  echo "1. Number of times to repeat test (e.g. 10)"
  echo "2. Host running 'iperf3 -s' (e.g. somehost)"
  echo
  echo "Example:"
  echo "  $(basename $0) 10 somehost"
  echo
  echo "The above will run 'iperf3 -c' 10 times on the client and report totals and average."
  exit 1
else
  runs=$1
  host=$2
fi

log=qos_olsr.txt

if [ -f "$log" ]; then
  echo removing $log
  rm $log
fi

echo "=================================================================="
echo " Results"
echo "=================================================================="
echo " target host .... $host"
echo "------------------------------------------------------------------"

for run in $(seq 1 $runs); do
  iperf3 -c $host -f m >> $log
  echo -e " run $run: \t $(awk '/Bitrate/ {getline}; END{print $7, $8}' $log)"
done


echo
echo "see $log for details"

#######################################################################
# Define the file path
file_path="$log"

# Check if the file exists
if [ -f "$file_path" ]; then
  # Use grep and awk to extract Bitrate from the receiver line in the file
  average_bitrate=$(awk '/receiver/ {sum += $7; count++} END {if (count > 0) print sum/count}' "$file_path")

  # Check if the value was found
  if [ -n "$average_bitrate" ]; then
    echo "average_bitrate: $average_bitrate Mbits/sec"
  else
    echo "average_bitrate receiver not found in the file."
  fi
else
  echo "File not found: $file_path"
fi



########################################################################
if [ -n "$average_bitrate" ]; then
    # Compare the Bitrate value with 25
    if (( $(echo "$average_bitrate > 25" | bc -l) )); then
      echo "Average Bitrate receiver: $average_bitrate Mbits/sec - Pass"
    else
      echo "Bitrate receiver: $average_bitrate Mbits/sec - Fail"
    fi
fi



