#!/bin/bash

robot_url="http://192.168.43.1:9000/matchLogs"
output_dir="matchLogs"
output_file="$output_dir/match-logs-$(date +%Y%m%d-%H%M%S).zip"

mkdir -p "$output_dir"

echo "Downloading match logs to $output_file"
out=$(curl -f -L "$robot_url" -o "$output_file" -m 10 2>&1)
rc=$?

if [ $rc -ne 0 ]; then
  rm -f "$output_file"
  echo "Downloading match logs failed with error $rc."
  echo "Check if you're connected to the robot WiFi and some OpMode is running."
  echo "Output was:"
  echo "$out"
  exit $rc
fi

echo "OK"
