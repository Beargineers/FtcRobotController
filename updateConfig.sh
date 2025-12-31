#!/bin/bash

cfg_path="$(git status --porcelain | awk '$1 ~ /M/ && $2 ~ /(^|\/)config\.properties$/ {print $2; exit}')"

if [ -z "$cfg_path" ]; then
  echo "No changed config file found, nothing to update from"
else
  echo "Updating $cfg_path"
  out=$(curl -X POST "http://192.168.43.1:9000/" -F "settings=@$cfg_path" -m 3 2>&1 >/dev/null )

  rc=$?
  if [ $rc -ne 0 ]; then
    echo "Updating config failed with error $rc."
    echo "Check if you're connected to the robot WiFi and some OpMode is running."
    echo "Output was:"
    echo $out
    exit $rc
  fi

  echo "OK"
fi