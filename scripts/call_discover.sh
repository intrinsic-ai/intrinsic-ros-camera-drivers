#!/bin/bash
set -o verbose
ros2 service call /cameras/discover snapshot_interfaces/srv/Discover
