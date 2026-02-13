#!/bin/bash
# launch_test.sh - Quick launcher for real-world tractor tests
# 
# Usage:
#   # 0..8 args: positions only (backward-compatible)
#   ./launch_test.sh [TEST_NAME] [GPS_TOPIC] \
#     [CAMERA_X] [CAMERA_Y] [CAMERA_Z] \
#     [RADAR_X] [RADAR_Y] [RADAR_Z]
#
#   # 14 args: full TF (positions + orientation)
#   ./launch_test.sh [TEST_NAME] [GPS_TOPIC] \
#     [CAMERA_X] [CAMERA_Y] [CAMERA_Z] [CAMERA_ROLL] [CAMERA_PITCH] [CAMERA_YAW] \
#     [RADAR_X]  [RADAR_Y]  [RADAR_Z]  [RADAR_ROLL]  [RADAR_PITCH]  [RADAR_YAW]
#
# This script assumes GPS (ublox_gps + optional NTRIP) is started separately.
# Recommended: run ./gps_rtk_start.sh in another terminal.
#
# Examples:
#   # Minimal (positions only)
#   ./launch_test.sh pedestrian_approach
#   ./launch_test.sh static_detection /gps/fix 1.2 0.0 0.8 1.5 0.0 0.5
#
#   # Full TF (recommended after calibration)
#   ./launch_test.sh field_test_S1 /gps/fix \
#     1.189 0.034 0.812 -1.5820 0.0034 -1.5650 \
#     1.523 -0.012 0.487 0.0052 0.0123 -0.0089

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
    echo "Usage:" >&2
    echo "  ./launch_test.sh [TEST_NAME] [GPS_TOPIC] [CAMERA_X] [CAMERA_Y] [CAMERA_Z] [RADAR_X] [RADAR_Y] [RADAR_Z]" >&2
    echo "  ./launch_test.sh [TEST_NAME] [GPS_TOPIC] \\" >&2
    echo "    [CAMERA_X] [CAMERA_Y] [CAMERA_Z] [CAMERA_ROLL] [CAMERA_PITCH] [CAMERA_YAW] \\" >&2
    echo "    [RADAR_X]  [RADAR_Y]  [RADAR_Z]  [RADAR_ROLL]  [RADAR_PITCH]  [RADAR_YAW]" >&2
}

# Default values
TEST_NAME=${1:-"field_test_S1"}
GPS_TOPIC=${2:-"/gps/fix"}

CAMERA_X=${3:-"1.0"}
CAMERA_Y=${4:-"0.0"}
CAMERA_Z=${5:-"0.0"}

# Match the default orientation values used in perception_stack.launch.py
CAMERA_ROLL_DEFAULT="-1.5708"
CAMERA_PITCH_DEFAULT="0.0"
CAMERA_YAW_DEFAULT="-1.5708"

RADAR_ROLL_DEFAULT="0.0"
RADAR_PITCH_DEFAULT="0.0"
RADAR_YAW_DEFAULT="0.0"

ARGC=$#

# Backward-compatible mode: positions only (0..8 args)
if [ "$ARGC" -le 8 ]; then
    RADAR_X=${6:-"0.0"}
    RADAR_Y=${7:-"0.0"}
    RADAR_Z=${8:-"1.0"}

    CAMERA_ROLL="$CAMERA_ROLL_DEFAULT"
    CAMERA_PITCH="$CAMERA_PITCH_DEFAULT"
    CAMERA_YAW="$CAMERA_YAW_DEFAULT"
    RADAR_ROLL="$RADAR_ROLL_DEFAULT"
    RADAR_PITCH="$RADAR_PITCH_DEFAULT"
    RADAR_YAW="$RADAR_YAW_DEFAULT"

    LAUNCH_TEST_MODE="positions_only"

# Full TF mode: positions + orientation (14 args)
elif [ "$ARGC" -ge 14 ]; then
    CAMERA_ROLL=${6:-"$CAMERA_ROLL_DEFAULT"}
    CAMERA_PITCH=${7:-"$CAMERA_PITCH_DEFAULT"}
    CAMERA_YAW=${8:-"$CAMERA_YAW_DEFAULT"}

    RADAR_X=${9:-"0.0"}
    RADAR_Y=${10:-"0.0"}
    RADAR_Z=${11:-"1.0"}
    RADAR_ROLL=${12:-"$RADAR_ROLL_DEFAULT"}
    RADAR_PITCH=${13:-"$RADAR_PITCH_DEFAULT"}
    RADAR_YAW=${14:-"$RADAR_YAW_DEFAULT"}

    LAUNCH_TEST_MODE="full_tf"

else
    echo "ERROR: Invalid number of arguments ($ARGC)." >&2
    echo "  Use 0..8 args (positions only) or 14 args (full TF)." >&2
    usage
    exit 2
fi

# Write a small marker file describing the last launch args.
# This allows validate_system.sh to warn when you’re still using positions-only mode.
(
    umask 077
    cat >"${SCRIPT_DIR}/last_launch_test.env" <<EOF
LAUNCH_TEST_MODE=${LAUNCH_TEST_MODE}
TEST_NAME=${TEST_NAME}
GPS_TOPIC=${GPS_TOPIC}
CAMERA_X=${CAMERA_X}
CAMERA_Y=${CAMERA_Y}
CAMERA_Z=${CAMERA_Z}
CAMERA_ROLL=${CAMERA_ROLL}
CAMERA_PITCH=${CAMERA_PITCH}
CAMERA_YAW=${CAMERA_YAW}
RADAR_X=${RADAR_X}
RADAR_Y=${RADAR_Y}
RADAR_Z=${RADAR_Z}
RADAR_ROLL=${RADAR_ROLL}
RADAR_PITCH=${RADAR_PITCH}
RADAR_YAW=${RADAR_YAW}
EOF
)

echo "========================================"
echo "Real-World Test Launcher"
echo "========================================"
echo "Test name:     $TEST_NAME"
echo "GPS topic:     $GPS_TOPIC"
echo "Camera TF:     x=$CAMERA_X, y=$CAMERA_Y, z=$CAMERA_Z, roll=$CAMERA_ROLL, pitch=$CAMERA_PITCH, yaw=$CAMERA_YAW"
echo "Radar TF:      x=$RADAR_X, y=$RADAR_Y, z=$RADAR_Z, roll=$RADAR_ROLL, pitch=$RADAR_PITCH, yaw=$RADAR_YAW"
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
    camera_tf_roll:=$CAMERA_ROLL \
    camera_tf_pitch:=$CAMERA_PITCH \
    camera_tf_yaw:=$CAMERA_YAW \
    radar_tf_x:=$RADAR_X \
    radar_tf_y:=$RADAR_Y \
    radar_tf_z:=$RADAR_Z \
    radar_tf_roll:=$RADAR_ROLL \
    radar_tf_pitch:=$RADAR_PITCH \
    radar_tf_yaw:=$RADAR_YAW

echo ""
echo "========================================"
echo "Test completed: $TEST_NAME"
echo "Check logs in: real_world_logs/"
echo "========================================"
