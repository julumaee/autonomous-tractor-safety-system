#!/bin/bash
# launch_test.sh - Interactive launcher for real-world tractor tests
# 
# Usage:
#   ./launch_test.sh                          # Interactive mode (recommended)
#   ./launch_test.sh [TEST_NAME]              # Interactive with test name
#   ./launch_test.sh [TEST_NAME] [GPS_TOPIC]  # Interactive with test name and GPS topic
#
# This script assumes GPS (ublox_gps + optional NTRIP) is started separately.
# Recommended: run ./gps_rtk_start.sh in another terminal.
# Also assumes CAN interface is set up and running (e.g. with ./setup_can.sh).

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Default TF values (robot coordinate frame)
# Camera TF values are base_link -> camera_link.
# NOTE: camera_node converts detections from optical -> camera_link online,
# so you should NOT apply the historical optical correction (-π/2, 0, -π/2)
# in this TF anymore.
# Default assumes camera_link axes are aligned with base_link when mounted upright.
# Radar: 0,0,0,0,0,0 (forward-facing at origin)
DEFAULT_CAMERA_X=0.0
DEFAULT_CAMERA_Y=0.0
DEFAULT_CAMERA_Z=0.0
DEFAULT_CAMERA_ROLL=0.0
DEFAULT_CAMERA_PITCH=0.0
DEFAULT_CAMERA_YAW=0.0

DEFAULT_RADAR_X=0.0
DEFAULT_RADAR_Y=0.0
DEFAULT_RADAR_Z=0.0
DEFAULT_RADAR_ROLL=0.0
DEFAULT_RADAR_PITCH=0.0
DEFAULT_RADAR_YAW=0.0

usage() {
    echo "Usage:" >&2
    echo "  ./launch_test.sh                          # Interactive mode" >&2
    echo "  ./launch_test.sh [TEST_NAME]              # Interactive with test name" >&2
    echo "  ./launch_test.sh [TEST_NAME] [GPS_TOPIC]  # Interactive with GPS topic" >&2
}

# Parse command-line arguments
TEST_NAME="${1:-}"
GPS_TOPIC="${2:-/navpvt}"

ensure_ros_domain() {
    local desired_domain="0"
    if [[ "${ROS_DOMAIN_ID:-}" != "${desired_domain}" ]]; then
        export ROS_DOMAIN_ID="${desired_domain}"
    fi
    # Restart the ROS 2 daemon so discovery uses this environment.
    ros2 daemon stop &>/dev/null || true
    ros2 daemon start &>/dev/null || true
}

ensure_ros_domain

# Interactive prompts
echo "========================================"
echo "Real-World Test Launcher - Setup"
echo "========================================"
echo ""

# Prompt for test name if not provided
if [[ -z "$TEST_NAME" ]]; then
    read -r -p "Enter test name [field_test_$(date +%Y%m%d_%H%M%S)]: " TEST_NAME
    TEST_NAME="${TEST_NAME:-field_test_$(date +%Y%m%d_%H%M%S)}"
fi

# Prompt for GPS topic
read -r -p "GPS topic [${GPS_TOPIC}]: " GPS_TOPIC_INPUT
GPS_TOPIC="${GPS_TOPIC_INPUT:-$GPS_TOPIC}"

# Prompt for whether to start the camera driver (depthai-ros-driver) inside the launch.
# Default: yes, since most real-world runs expect the camera stack to be brought up.
START_CAMERA_DRIVER="true"
read -r -p "Start camera driver (OAK-D) inside launch? [Y/n]: " START_CAMERA_DRIVER_IN
START_CAMERA_DRIVER_IN="${START_CAMERA_DRIVER_IN:-Y}"
case "${START_CAMERA_DRIVER_IN}" in
    Y|y|yes|YES)
        START_CAMERA_DRIVER="true"
        ;;
    N|n|no|NO)
        START_CAMERA_DRIVER="false"
        ;;
    *)
        echo "Invalid choice, defaulting to: ${START_CAMERA_DRIVER}"
        ;;
esac

# Prompt for covariance model usage in the tracker.
# This toggles whether kf_tracker uses range/bearing-based measurement covariance models
# vs constant per-source covariance (R_meas_*_xy).
USE_MEASUREMENT_COVARIANCE_MODELS="true"
read -r -p "Use range-dependent measurement covariance models in tracker? [Y/n]: " USE_MEAS_COV_IN
USE_MEAS_COV_IN="${USE_MEAS_COV_IN:-Y}"
case "${USE_MEAS_COV_IN}" in
    Y|y|yes|YES)
        USE_MEASUREMENT_COVARIANCE_MODELS="true"
        ;;
    N|n|no|NO)
        USE_MEASUREMENT_COVARIANCE_MODELS="false"
        ;;
    *)
        echo "Invalid choice, defaulting to: ${USE_MEASUREMENT_COVARIANCE_MODELS}"
        ;;
esac

echo ""
echo "========================================"
echo "Sensor Calibration / Transform Setup"
echo "========================================"
echo ""

# Check if calibration file exists
TF_FILE="${SCRIPT_DIR}/calibration_log/calibration_tf.txt"
CALIBRATION_FILE_EXISTS=false
if [[ -f "${TF_FILE}" ]]; then
    CALIBRATION_FILE_EXISTS=true
    echo "Calibration file found: ${TF_FILE}"
    
    # Read calibration file
    TF_CONTENT="$(tr -d '\r' < "${TF_FILE}")"
    cam_line="$(printf "%s\n" "${TF_CONTENT}" | grep -E '^[[:space:]]*camera_tf:[[:space:]]' | head -n1 || true)"
    rad_line="$(printf "%s\n" "${TF_CONTENT}" | grep -E '^[[:space:]]*radar_tf:[[:space:]]'  | head -n1 || true)"
    
    if [[ -n "${cam_line}" && -n "${rad_line}" ]]; then
        cam_vals="$(printf "%s\n" "${cam_line}" | sed -E 's/^[[:space:]]*camera_tf:[[:space:]]*//')"
        rad_vals="$(printf "%s\n" "${rad_line}" | sed -E 's/^[[:space:]]*radar_tf:[[:space:]]*//')"
        
        read -r -a carr <<< "${cam_vals}"
        read -r -a rarr <<< "${rad_vals}"
        
        if [[ "${#carr[@]}" -ge 6 && "${#rarr[@]}" -ge 6 ]]; then
            CAL_CAMERA_X="${carr[0]}"
            CAL_CAMERA_Y="${carr[1]}"
            CAL_CAMERA_Z="${carr[2]}"
            CAL_CAMERA_ROLL="${carr[3]}"
            CAL_CAMERA_PITCH="${carr[4]}"
            CAL_CAMERA_YAW="${carr[5]}"
            
            CAL_RADAR_X="${rarr[0]}"
            CAL_RADAR_Y="${rarr[1]}"
            CAL_RADAR_Z="${rarr[2]}"
            CAL_RADAR_ROLL="${rarr[3]}"
            CAL_RADAR_PITCH="${rarr[4]}"
            CAL_RADAR_YAW="${rarr[5]}"
            
            echo "  Camera: x=${CAL_CAMERA_X}, y=${CAL_CAMERA_Y}, z=${CAL_CAMERA_Z},"
            echo "          roll=${CAL_CAMERA_ROLL}, pitch=${CAL_CAMERA_PITCH}, yaw=${CAL_CAMERA_YAW}"
            echo "  Radar:  x=${CAL_RADAR_X}, y=${CAL_RADAR_Y}, z=${CAL_RADAR_Z},"
            echo "          roll=${CAL_RADAR_ROLL}, pitch=${CAL_RADAR_PITCH}, yaw=${CAL_RADAR_YAW}"
        else
            CALIBRATION_FILE_EXISTS=false
            echo "  ⚠ Warning: Calibration file format invalid"
        fi
    else
        CALIBRATION_FILE_EXISTS=false
        echo "  ⚠ Warning: Calibration file incomplete"
    fi
else
    echo "No calibration file found."
fi

echo ""
echo "Transform options:"
echo "  1) Use calibrated values from file [DEFAULT - just press Enter]"
echo "  2) Use default/nominal transforms (camera: -π/2,0,-π/2, radar: 0,0,0)"
echo ""

USE_CALIBRATED=true
if [[ "$CALIBRATION_FILE_EXISTS" == true ]]; then
    read -r -p "Select option [1]: " TF_CHOICE
    TF_CHOICE="${TF_CHOICE:-1}"
    
    case "$TF_CHOICE" in
        1)
            USE_CALIBRATED=true
            echo "Using calibrated transforms from file"
            CAMERA_X="$CAL_CAMERA_X"
            CAMERA_Y="$CAL_CAMERA_Y"
            CAMERA_Z="$CAL_CAMERA_Z"
            CAMERA_ROLL="$CAL_CAMERA_ROLL"
            CAMERA_PITCH="$CAL_CAMERA_PITCH"
            CAMERA_YAW="$CAL_CAMERA_YAW"
            
            RADAR_X="$CAL_RADAR_X"
            RADAR_Y="$CAL_RADAR_Y"
            RADAR_Z="$CAL_RADAR_Z"
            RADAR_ROLL="$CAL_RADAR_ROLL"
            RADAR_PITCH="$CAL_RADAR_PITCH"
            RADAR_YAW="$CAL_RADAR_YAW"
            ;;
        2)
            USE_CALIBRATED=false
            echo "Using default transforms"
            CAMERA_X="$DEFAULT_CAMERA_X"
            CAMERA_Y="$DEFAULT_CAMERA_Y"
            CAMERA_Z="$DEFAULT_CAMERA_Z"
            CAMERA_ROLL="$DEFAULT_CAMERA_ROLL"
            CAMERA_PITCH="$DEFAULT_CAMERA_PITCH"
            CAMERA_YAW="$DEFAULT_CAMERA_YAW"
            
            RADAR_X="$DEFAULT_RADAR_X"
            RADAR_Y="$DEFAULT_RADAR_Y"
            RADAR_Z="$DEFAULT_RADAR_Z"
            RADAR_ROLL="$DEFAULT_RADAR_ROLL"
            RADAR_PITCH="$DEFAULT_RADAR_PITCH"
            RADAR_YAW="$DEFAULT_RADAR_YAW"
            ;;
        *)
            echo "Invalid choice, using calibrated transforms"
            USE_CALIBRATED=true
            CAMERA_X="$CAL_CAMERA_X"
            CAMERA_Y="$CAL_CAMERA_Y"
            CAMERA_Z="$CAL_CAMERA_Z"
            CAMERA_ROLL="$CAL_CAMERA_ROLL"
            CAMERA_PITCH="$CAL_CAMERA_PITCH"
            CAMERA_YAW="$CAL_CAMERA_YAW"
            
            RADAR_X="$CAL_RADAR_X"
            RADAR_Y="$CAL_RADAR_Y"
            RADAR_Z="$CAL_RADAR_Z"
            RADAR_ROLL="$CAL_RADAR_ROLL"
            RADAR_PITCH="$CAL_RADAR_PITCH"
            RADAR_YAW="$CAL_RADAR_YAW"
            ;;
    esac
else
    echo "No calibration file available. Using default transforms."
    CAMERA_X="$DEFAULT_CAMERA_X"
    CAMERA_Y="$DEFAULT_CAMERA_Y"
    CAMERA_Z="$DEFAULT_CAMERA_Z"
    CAMERA_ROLL="$DEFAULT_CAMERA_ROLL"
    CAMERA_PITCH="$DEFAULT_CAMERA_PITCH"
    CAMERA_YAW="$DEFAULT_CAMERA_YAW"
    
    RADAR_X="$DEFAULT_RADAR_X"
    RADAR_Y="$DEFAULT_RADAR_Y"
    RADAR_Z="$DEFAULT_RADAR_Z"
    RADAR_ROLL="$DEFAULT_RADAR_ROLL"
    RADAR_PITCH="$DEFAULT_RADAR_PITCH"
    RADAR_YAW="$DEFAULT_RADAR_YAW"
fi

echo ""
echo "========================================"
echo "Real-World Test Launcher"
echo "========================================"
echo "Test name:     $TEST_NAME"
echo "GPS topic:     $GPS_TOPIC"
echo "Camera driver: start_camera_driver:=$START_CAMERA_DRIVER"
echo "Tracker cov.:  use_measurement_covariance_models:=$USE_MEASUREMENT_COVARIANCE_MODELS"
echo "Camera TF:     x=$CAMERA_X, y=$CAMERA_Y, z=$CAMERA_Z, roll=$CAMERA_ROLL, pitch=$CAMERA_PITCH, yaw=$CAMERA_YAW"
echo "Radar TF:      x=$RADAR_X, y=$RADAR_Y, z=$RADAR_Z, roll=$RADAR_ROLL, pitch=$RADAR_PITCH, yaw=$RADAR_YAW"
echo "========================================"
echo ""

# Step 1: Check CAN interface is up (fatal)
echo "[1/4] Checking CAN interface..."
if ip link show can0 | grep -q "UP"; then
    echo "  ✓ CAN interface can0 is UP"
else
    echo "  ✗ CAN interface can0 is not UP" >&2
    echo "    Please set up CAN interface before running the test, e.g.:" >&2
    echo "    ./setup_can.sh" >&2
    exit 1
fi
echo ""

# Step 2: Verify GPS is available and check RTK status
echo "[2/4] Checking GPS availability and RTK status..."
GPS_FOUND=false
if timeout 3 ros2 topic echo "$GPS_TOPIC" --once &>/dev/null; then
    echo "  ✓ GPS data available on $GPS_TOPIC"
    GPS_FOUND=true
    
    # Check GPS status for RTK
    GPS_FIX_OUT="$(timeout 3 ros2 topic echo "$GPS_TOPIC" --once 2>/dev/null || true)"
    if [[ -n "$GPS_FIX_OUT" ]]; then
        GPS_STATUS="$(printf '%s\n' "$GPS_FIX_OUT" | awk -F': ' '/^  status:/{print $2}' | head -n1 | tr -d ' ')"
        
        case "$GPS_STATUS" in
            4)
                echo "  ✓ RTK FIXED solution (cm-level accuracy)"
                ;;
            3)
                echo "  ⚠ RTK FLOAT solution (waiting for fixed)"
                echo "    Consider waiting for RTK fixed if cm-level accuracy needed"
                ;;
            2)
                echo "  ⚠ SBAS fix (meter-level accuracy)"
                echo "    RTK corrections may not be active"
                ;;
            1)
                echo "  ⚠ Standard GPS fix (no RTK)"
                echo "    Check RTCM stream: ros2 topic hz /rtcm"
                ;;
            0)
                echo "  ✗ NO GPS FIX"
                echo "    Wait for GPS fix before starting test"
                ;;
        esac
    fi
    
    # Check RTCM stream
    if timeout 2 ros2 topic list 2>/dev/null | grep -q "^/rtcm$"; then
        if timeout 3 ros2 topic echo /rtcm --once &>/dev/null; then
            echo "  ✓ RTCM corrections stream active"
        else
            echo "  ⚠ RTCM topic exists but no messages received"
            echo "    NTRIP client may be connecting or caster unavailable"
        fi
    else
        echo "  ⚠ RTCM topic not found (RTK corrections not available)"
        echo "    For RTK operation, start with: cd testing_tools && ./gps_rtk_start.sh"
    fi
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
    start_camera_driver:=$START_CAMERA_DRIVER \
    use_measurement_covariance_models:=$USE_MEASUREMENT_COVARIANCE_MODELS \
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
