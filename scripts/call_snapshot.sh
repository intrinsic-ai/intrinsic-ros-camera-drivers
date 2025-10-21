#!/bin/bash
set -o verbose
ros2 service call /orbbec_CPEH552000FV/snapshot snapshot_interfaces/srv/Snapshot > output.txt
