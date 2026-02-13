#!/bin/bash
# launch_test.sh - Quick launcher for real-world tractor tests
# 
# Usage:
#   ./launch_test.sh [TEST_NAME] [GPS_TOPIC] [CAMERA_X] [CAMERA_Y] [CAMERA_Z] [RADAR_X] [RADAR_Y] [RADAR_Z]
#
# This script assumes GPS (ublox_gps + optional NTRIP) is started separately.
# Recommended: run ./gps_rtk_start.sh in another terminal.
#
# Examples:
#   ./launch_test.sh pedestrian_approach
#   ./launch_test.sh static_detection /gps/fix 1.2 0.0 0.8 1.5 0.0 0.5
#   ./launch_test.sh field_test_1 /fix 1.3 0.0 0.85 1.6 0.0 0.55

set -e  # Exit on error

# Default values
TEST_NAME=${1:-"field_test_S1"}
GPS_TOPIC=${2:-"/gps/fix"}
CAMERA_X=${3:-"1.0"}
CAMERA_Y=${4:-"0.0"}
CAMERA_Z=${5:-"0.0"}
RADAR_X=${6:-"0.0"}
RADAR_Y=${7:-"0.0"}
RADAR_Z=${8:-"1.0"}

echo "========================================"
echo "Real-World Test Launcher"
echo "========================================"
echo "Test name:     $TEST_NAME"
echo "GPS topic:     $GPS_TOPIC"
echo "Camera TF:     x=$CAMERA_X, y=$CAMERA_Y, z=$CAMERA_Z"
echo "Radar TF:      x=$RADAR_X, y=$RADAR_Y, z=$RADAR_Z"
echo "========================================"
echo ""

# Step 1: Setup CAN interface
echo "[1/4] Setting up CAN interface..."
if sudo modprobe kvaser_usb 2>/dev/null; then
    echo "  ✓ Kvaser module loaded"
else
    echo "  ⚠ Warning: Could not load kvaser_usb module (may already be loaded)"
fi

sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

if ip link show can0 | grep -q "UP"; then
    echo "  ✓ CAN interface can0 is UP"
else
    echo "  ✗ ERROR: CAN interface failed to start"
    exit 1
fi
echo ""

# Step 2: Verify GPS is available (non-fatal)
echo "[2/4] Checking GPS availability..."
if timeout 3 ros2 topic echo "$GPS_TOPIC" --once &>/dev/null; then
    echo "  ✓ GPS data available on $GPS_TOPIC"
else
    echo "  ⚠ GPS not detected on $GPS_TOPIC"
    echo "    Start GPS in another terminal, e.g.:"
    echo "      cd testing_tools && ./gps_rtk_start.sh"
fi
echo ""

# Step 3: Check workspace is sourced
echo "[3/4] Verifying ROS 2 workspace..."
if ! ros2 pkg list | grep -q "tractor_safety_system_launch"; then
    echo "  ✗ ERROR: Workspace not sourced"
    echo "  Please run: source install/setup.bash"
    exit 1
fi
echo "  ✓ Workspace sourced"
echo ""

# Step 4: Launch test system
echo "[4/4] Launching real-world test system..."
echo ""
echo "Press Ctrl+C to stop the test and save data"
echo "========================================"
echo ""

ros2 launch tractor_safety_system_launch real_world_test.launch.py \
    test_name:=$TEST_NAME \
    gps_fix_topic:=$GPS_TOPIC \
    camera_tf_x:=$CAMERA_X \
    camera_tf_y:=$CAMERA_Y \
    camera_tf_z:=$CAMERA_Z \
    radar_tf_x:=$RADAR_X \
    radar_tf_y:=$RADAR_Y \
    radar_tf_z:=$RADAR_Z

echo ""
echo "========================================"
echo "Test completed: $TEST_NAME"
echo "Check logs in: real_world_logs/"
echo "========================================"
