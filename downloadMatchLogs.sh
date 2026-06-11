#!/bin/bash

robot_url="http://192.168.43.1:9000/matchLogs"
output_dir="matchLogs"
output_file="$output_dir/match-logs-$(date +%Y%m%d-%H%M%S).zip"
extract_dir="${output_file%.zip}"

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

mkdir -p "$extract_dir"

echo "Extracting match logs to $extract_dir"
out=$(unzip -q "$output_file" -d "$extract_dir" 2>&1)
rc=$?

if [ $rc -ne 0 ]; then
  rm -rf "$extract_dir"
  echo "Extracting match logs failed with error $rc."
  echo "Output was:"
  echo "$out"
  exit $rc
fi

echo "Renaming log files to .log"
out=$(find "$extract_dir" -type f -name "*.txt" -exec sh -c '
  for file do
    target="${file%.txt}.log"
    if [ -e "$target" ]; then
      echo "Refusing to overwrite $target"
      exit 1
    fi
    mv "$file" "$target" || exit $?
  done
' sh {} + 2>&1)
rc=$?

if [ $rc -ne 0 ]; then
  rm -rf "$extract_dir"
  echo "Renaming log files failed with error $rc."
  echo "Output was:"
  echo "$out"
  exit $rc
fi

rm -f "$output_file"

echo "OK"
