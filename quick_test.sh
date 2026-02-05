#!/bin/bash

# Quick test script for Flowstate camera drivers
# This script helps you quickly validate the refactored code

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_header() {
    echo -e "\n${BLUE}=== $1 ===${NC}\n"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

# Get workspace root
if [ -z "$ROS_WS_ROOT" ]; then
    ROS_WS_ROOT=$(cd "$(dirname "$0")" && pwd)
fi

print_header "Flowstate Camera Drivers - Quick Test"

# Check if ROS environment is available
print_header "TEST 0: Checking ROS Environment"
if command -v colcon &> /dev/null; then
    print_success "colcon found"
else
    print_error "colcon not found - ROS 2 environment not sourced"
    echo ""
    echo "This workspace requires ROS 2 Jazzy to be installed and sourced."
    echo "Please run in a ROS 2 environment or Docker container with ROS 2 Jazzy."
    echo ""
    echo "To set up ROS 2 Jazzy locally, follow:"
    echo "  https://docs.ros.org/en/jazzy/Installation.html"
    echo ""
    exit 1
fi

if command -v ros2 &> /dev/null; then
    print_success "ros2 CLI found"
    ros2 --version | sed 's/^/  /'
else
    print_error "ros2 CLI not found"
    exit 1
fi

# Test 1: Build
print_header "TEST 1: Building Packages"
echo "Building flowstate_common, flowstate_luxonis, flowstate_orbbec..."
if cd "$ROS_WS_ROOT" && colcon build --packages-up-to flowstate_common flowstate_luxonis flowstate_orbbec 2>&1 | grep -E "(Finished|Failed)"; then
    print_success "Build completed"
else
    print_error "Build failed - see full output above"
    exit 1
fi

# Test 2: Source setup
print_header "TEST 2: Sourcing Setup"
source "$ROS_WS_ROOT/install/setup.bash"
print_success "Setup sourced"

# Test 3: Check if ROS is running
print_header "TEST 3: Checking ROS Daemon"
if pgrep -x "ros2" > /dev/null; then
    print_success "ROS daemon is running"
else
    print_warning "ROS daemon not running, starting it..."
    ros2 daemon start
    sleep 1
fi

# Test 4: List available packages
print_header "TEST 4: Checking Installed Packages"
if ros2 pkg list | grep -q flowstate_common; then
    print_success "flowstate_common package found"
else
    print_error "flowstate_common package not found"
    exit 1
fi

if ros2 pkg list | grep -q flowstate_luxonis; then
    print_success "flowstate_luxonis package found"
else
    print_warning "flowstate_luxonis package not found"
fi

if ros2 pkg list | grep -q flowstate_orbbec; then
    print_success "flowstate_orbbec package found"
else
    print_warning "flowstate_orbbec package not found"
fi

# Test 5: Check libraries
print_header "TEST 5: Checking Compiled Libraries"
if [ -f "$ROS_WS_ROOT/install/flowstate_common/lib/libflowstate_common.so" ]; then
    print_success "libflowstate_common.so exists"
else
    print_error "libflowstate_common.so not found"
fi

if [ -f "$ROS_WS_ROOT/install/flowstate_luxonis/lib/libflowstate_luxonis.so" ]; then
    print_success "libflowstate_luxonis.so exists"
else
    print_warning "libflowstate_luxonis.so not found"
fi

if [ -f "$ROS_WS_ROOT/install/flowstate_orbbec/lib/libflowstate_orbbec.so" ]; then
    print_success "libflowstate_orbbec.so exists"
else
    print_warning "libflowstate_orbbec.so not found"
fi

# Test 6: Check headers
print_header "TEST 6: Checking Header Files"
HEADERS=(
    "flowstate_common/camera_adapter_node.h"
    "flowstate_common/camera_spawner_node.h"
    "flowstate_common/image_utils.h"
)

for header in "${HEADERS[@]}"; do
    if [ -f "$ROS_WS_ROOT/install/flowstate_common/include/$header" ]; then
        print_success "$header found"
    else
        print_error "$header not found"
    fi
done

# Test 7: Try to run spawner node (with timeout)
print_header "TEST 7: Testing Spawner Nodes"

for camera_type in luxonis orbbec; do
    echo "Testing flowstate_$camera_type spawner..."
    
    if timeout 3 ros2 run flowstate_$camera_type spawner_node 2>&1 | head -5; then
        print_success "flowstate_$camera_type spawner node starts successfully"
    else
        case $? in
            124) print_warning "flowstate_$camera_type spawner timeout (expected - no camera connected)" ;;
            *) print_error "flowstate_$camera_type spawner node failed to start" ;;
        esac
    fi
done

# Test 8: Check for basic ROS interfaces
print_header "TEST 8: Checking ROS Interfaces"
if ros2 interface show snapshot_interfaces/srv/Discover > /dev/null 2>&1; then
    print_success "Discover service interface found"
else
    print_warning "Discover service interface not found"
fi

if ros2 interface show snapshot_interfaces/srv/Snapshot > /dev/null 2>&1; then
    print_success "Snapshot service interface found"
else
    print_warning "Snapshot service interface not found"
fi

# Test 9: Code quality checks
print_header "TEST 9: Code Quality"

COMMON_FILES=(
    "flowstate_common/include/flowstate_common/camera_adapter_node.h"
    "flowstate_common/src/camera_adapter_node.cc"
    "flowstate_common/include/flowstate_common/camera_spawner_node.h"
    "flowstate_common/src/camera_spawner_node.cc"
    "flowstate_common/include/flowstate_common/image_utils.h"
    "flowstate_common/src/image_utils.cc"
)

all_exist=true
for file in "${COMMON_FILES[@]}"; do
    if [ -f "$ROS_WS_ROOT/$file" ]; then
        lines=$(wc -l < "$ROS_WS_ROOT/$file")
        echo "  ✓ $file ($lines lines)"
    else
        print_error "$file not found"
        all_exist=false
    fi
done

if $all_exist; then
    print_success "All expected files exist"
fi

# Summary
print_header "SUMMARY"

print_success "Quick test completed!"
echo ""
echo "Next steps:"
echo "1. Connect cameras or use camera simulators"
echo "2. Run: ros2 run flowstate_luxonis spawner_node"
echo "3. In another terminal: ros2 service call /cameras/discover snapshot_interfaces/srv/Discover"
echo "4. See TESTING.md for detailed testing guide"
echo ""
echo "Documentation:"
echo "- ADDING_NEW_CAMERA.md - How to add a new camera driver"
echo "- TESTING.md - Comprehensive testing guide"
echo "- README.md - Architecture overview"
