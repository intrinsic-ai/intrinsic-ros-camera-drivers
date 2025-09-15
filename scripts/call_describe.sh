#!/bin/bash
set -o verbose
#ros2 service call /cameras/discover snapshot_interfaces/srv/Discover
ros2 service call /orbbec_CPEH552000FV/describe snapshot_interfaces/srv/Describe
