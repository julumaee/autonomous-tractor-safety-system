#!/bin/bash
# validate_system.sh - Pre-test system validation
# 
# Checks all hardware and software components before starting a test
# Run this before each test session to catch issues early

echo "========================================"
echo "System Validation - Pre-Test Checks"
echo "========================================"
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

FAIL_COUNT=0
WARN_COUNT=0

TOPIC_LIST=""

topic_exists() {
    local topic="$1"
    printf '%s\n' "$TOPIC_LIST" | grep -qx "$topic"
}

topic_publisher_count() {
    local topic="$1"
    timeout 2 ros2 topic info -v "$topic" 2>/dev/null | awk -F': ' '/^Publisher count:/{print $2}' | head -n 1
}

topic_has_publisher() {
    local topic="$1"
    local count
    count="$(topic_publisher_count "$topic")"
    if [[ -z "$count" ]]; then
        count=0
    fi
    [[ "$count" -gt 0 ]]
}

check_topic_once() {
    local topic="$1"
    if timeout 3 ros2 topic echo "$topic" --once &>/dev/null; then
        return 0
    fi
    return 1
}

check_topic_rate_warn() {
    # Warn-only topic rate check: if a publisher exists, try to get an average rate.
    local topic="$1"
    local expected_min_hz="$2"

    local hz
    hz=$(timeout 5 ros2 topic hz "$topic" 2>&1 | grep "average rate" | awk '{print $3}' | head -n 1 || true)
    if [[ -n "$hz" ]] && command -v bc &>/dev/null && (( $(echo "$hz > $expected_min_hz" | bc -l 2>/dev/null || echo 0) )); then
        echo "  ✓ PASS: ${topic} publishing at ${hz} Hz"
        return 0
    fi
    if check_topic_once "$topic"; then
        echo "  ✓ PASS: ${topic} data received"
        return 0
    fi

    echo "  ⚠ WARN: ${topic} publisher exists but no data observed"
    ((WARN_COUNT++))
    return 1
}

# Check 1: ROS 2 workspace
echo "[1/12] Checking ROS 2 workspace..."
if ! command -v ros2 &> /dev/null; then
    echo "  ✗ FAIL: ROS 2 not found in PATH"
    ((FAIL_COUNT++))
elif ! ros2 pkg list | grep -q "tractor_safety_system_launch"; then
    echo "  ✗ FAIL: Workspace not sourced"
    echo "         Run: source install/setup.bash"
    ((FAIL_COUNT++))
else
    echo "  ✓ PASS: Workspace sourced correctly"
    TOPIC_LIST="$(ros2 topic list 2>/dev/null || true)"
fi
echo ""

# Check 2: CAN interface
echo "[2/12] Checking CAN interface..."
if ! ip link show can0 &>/dev/null; then
    echo "  ✗ FAIL: CAN interface can0 not found"
    echo "         Attach Kvaser device and run setup_can.sh"
    ((FAIL_COUNT++))
elif ! ip link show can0 | grep -q "UP"; then
    echo "  ⚠ WARN: CAN interface exists but not UP"
    echo "         Run: sudo ip link set can0 up type can bitrate 1000000"
    ((WARN_COUNT++))
else
    echo "  ✓ PASS: CAN interface can0 is UP"
    
    # Test for CAN traffic
    if timeout 2 candump can0 -n 1 &>/dev/null; then
        echo "  ✓ PASS: CAN traffic detected"
    else
        echo "  ⚠ WARN: No CAN traffic (radar may be off or no detections)"
        ((WARN_COUNT++))
    fi
fi
echo ""

# Check 3: GPS availability
echo "[3/12] Checking GPS data..."
GPS_TOPICS=$(printf '%s\n' "$TOPIC_LIST" | grep -E "(gps|fix|navsat)" || echo "")
if [ -z "$GPS_TOPICS" ]; then
    echo "  ✗ FAIL: No GPS topics found"
    echo "         Start GPS using the recommended script:"
    echo "           cd testing_tools && ./gps_rtk_start.sh"
    ((FAIL_COUNT++))
else
    echo "  ✓ PASS: GPS topics found:"
    for topic in $GPS_TOPICS; do
        echo "         - $topic"
    done
    
    # Prefer the standard topic used by this repo.
    if topic_exists "/gps/fix"; then
        GPS_TOPIC="/gps/fix"
    else
        GPS_TOPIC=$(echo "$GPS_TOPICS" | head -1)
    fi

    echo "  Checking GPS data on $GPS_TOPIC..."
    if check_topic_once "$GPS_TOPIC"; then
        echo "  ✓ PASS: GPS data received on $GPS_TOPIC"
    else
        echo "  ⚠ WARN: GPS topic exists but no data observed on $GPS_TOPIC"
        ((WARN_COUNT++))
    fi

    echo "  Checking GPS rate on $GPS_TOPIC..."
    GPS_HZ=$(timeout 5 ros2 topic hz "$GPS_TOPIC" 2>&1 | grep "average rate" | awk '{print $3}' | head -n 1 || echo "")
    if [[ -n "$GPS_HZ" ]] && command -v bc &>/dev/null && (( $(echo "$GPS_HZ > 5" | bc -l 2>/dev/null || echo 0) )); then
        echo "  ✓ PASS: GPS publishing at ${GPS_HZ} Hz"
    else
        echo "  ⚠ WARN: GPS rate low or not detected"
        ((WARN_COUNT++))
    fi
fi
echo ""

# Check 4: Sensor topics (optional live check)
# These checks become meaningful if you already launched the perception/test stack.
echo "[4/12] Checking sensor topics (if running)..."
if topic_exists "/radar_detections"; then
    if topic_has_publisher "/radar_detections"; then
        check_topic_rate_warn "/radar_detections" 1
    else
        echo "  ⚠ WARN: /radar_detections exists but has no publisher"
        ((WARN_COUNT++))
    fi
else
    echo "  - INFO: /radar_detections not present (launch stack to validate radar pipeline)"
fi

if topic_exists "/camera_detections"; then
    if topic_has_publisher "/camera_detections"; then
        check_topic_rate_warn "/camera_detections" 1
    else
        echo "  ⚠ WARN: /camera_detections exists but has no publisher"
        ((WARN_COUNT++))
    fi
else
    echo "  - INFO: /camera_detections not present (launch stack to validate camera pipeline)"
fi

if topic_exists "/fused_detections"; then
    if topic_has_publisher "/fused_detections"; then
        check_topic_rate_warn "/fused_detections" 1
    else
        echo "  ⚠ WARN: /fused_detections exists but has no publisher"
        ((WARN_COUNT++))
    fi
else
    echo "  - INFO: /fused_detections not present (launch stack to validate fusion pipeline)"
fi

if topic_exists "/tracked_detections"; then
    if topic_has_publisher "/tracked_detections"; then
        check_topic_rate_warn "/tracked_detections" 1
    else
        echo "  ⚠ WARN: /tracked_detections exists but has no publisher"
        ((WARN_COUNT++))
    fi
else
    echo "  - INFO: /tracked_detections not present (launch stack to validate tracking pipeline)"
fi

if topic_exists "/ego_motion"; then
    if topic_has_publisher "/ego_motion"; then
        check_topic_rate_warn "/ego_motion" 1
    else
        echo "  ⚠ WARN: /ego_motion exists but has no publisher"
        ((WARN_COUNT++))
    fi
else
    echo "  - INFO: /ego_motion not present (launch stack to validate ego motion)"
fi
echo ""

# Check 5: Camera (OAK-D)
echo "[5/12] Checking OAK-D camera..."
if ! lsusb | grep -i "movidius\|myriadx\|luxonis\|oak" &>/dev/null; then
    echo "  ✗ FAIL: OAK-D camera not detected via USB"
    echo "         Check USB connection"
    ((FAIL_COUNT++))
else
    echo "  ✓ PASS: OAK-D camera detected via USB"
fi
echo ""

# Check 6: Required executables
echo "[6/12] Checking ROS 2 executables..."
EXECUTABLES=(
    "simulations:gps_ego_motion"
    "simulations:real_world_logger"
    "radar_interface:radar_node"
    "camera_interface:camera_node"
    "sensor_fusion:fusion_node"
    "sensor_fusion:kf_tracker"
)

for exec in "${EXECUTABLES[@]}"; do
    PKG=$(echo $exec | cut -d: -f1)
    NODE=$(echo $exec | cut -d: -f2)
    
    if ros2 pkg executables $PKG 2>/dev/null | grep -q $NODE; then
        echo "  ✓ PASS: $PKG/$NODE found"
    else
        echo "  ✗ FAIL: $PKG/$NODE not found"
        echo "         Rebuild: colcon build --packages-select $PKG"
        ((FAIL_COUNT++))
    fi
done
echo ""

# Check 7: Log directory
echo "[7/12] Checking log directory..."
if [ ! -d "real_world_logs" ]; then
    echo "  ⚠ WARN: real_world_logs directory doesn't exist (will be created)"
    ((WARN_COUNT++))
else
    echo "  ✓ PASS: real_world_logs directory exists"
    
    # Check permissions
    if [ -w "real_world_logs" ]; then
        echo "  ✓ PASS: Directory is writable"
    else
        echo "  ✗ FAIL: Directory not writable"
        ((FAIL_COUNT++))
    fi
fi
echo ""

# Check 8: Disk space
echo "[8/12] Checking disk space..."
DISK_AVAIL=$(df -h . | tail -1 | awk '{print $4}')
DISK_AVAIL_GB=$(df -BG . | tail -1 | awk '{print $4}' | sed 's/G//')
if [ "$DISK_AVAIL_GB" -lt 1 ]; then
    echo "  ✗ FAIL: Low disk space: $DISK_AVAIL available"
    ((FAIL_COUNT++))
elif [ "$DISK_AVAIL_GB" -lt 5 ]; then
    echo "  ⚠ WARN: Disk space low: $DISK_AVAIL available"
    ((WARN_COUNT++))
else
    echo "  ✓ PASS: Sufficient disk space: $DISK_AVAIL available"
fi
echo ""

# Check 9: CAN utilities
echo "[9/12] Checking CAN utilities..."
if ! command -v candump &> /dev/null; then
    echo "  ✗ FAIL: candump not found"
    echo "         Install: sudo apt install can-utils"
    ((FAIL_COUNT++))
else
    echo "  ✓ PASS: CAN utilities installed"
fi
echo ""

# Check 10: Launch files
echo "[10/12] Checking required launch files..."
LAUNCH_FILES_OK=true

# Check perception stack launch file using ros2 launch --show-args
if ros2 launch tractor_safety_system_launch perception_stack.launch.py --show-args &>/dev/null; then
    echo "  ✓ PASS: perception_stack.launch.py found"
else
    echo "  ✗ FAIL: perception_stack.launch.py not found"
    echo "         Rebuild: colcon build --packages-select tractor_safety_system_launch"
    ((FAIL_COUNT++))
    LAUNCH_FILES_OK=false
fi

# Check real-world test launch file
if ros2 launch tractor_safety_system_launch real_world_test.launch.py --show-args &>/dev/null; then
    echo "  ✓ PASS: real_world_test.launch.py found"
else
    echo "  ✗ FAIL: real_world_test.launch.py not found"
    echo "         Rebuild: colcon build --packages-select tractor_safety_system_launch"
    ((FAIL_COUNT++))
    LAUNCH_FILES_OK=false
fi
echo ""

# Check 11: Calibration / TF usage
echo "[11/12] Checking TF configuration usage..."

LAST_LAUNCH_FILE="${SCRIPT_DIR}/last_launch_test.env"
if [ -f "$LAST_LAUNCH_FILE" ]; then
    # shellcheck disable=SC1090
    source "$LAST_LAUNCH_FILE"

    if [ "${LAUNCH_TEST_MODE:-}" = "positions_only" ]; then
        echo "  ⚠ WARN: Last launch_test.sh run used positions-only mode"
        echo "         This uses default roll/pitch/yaw values."
        echo "         If you have calibration results, use full TF mode (14 args) from REAL_WORLD_TESTING_GUIDE.md."
        ((WARN_COUNT++))
    elif [ "${LAUNCH_TEST_MODE:-}" = "full_tf" ]; then
        echo "  ✓ PASS: Last launch_test.sh run used full TF mode"
    else
        echo "  ⚠ WARN: last_launch_test.env found but LAUNCH_TEST_MODE is missing/unknown"
        ((WARN_COUNT++))
    fi
else
    echo "  ⚠ WARN: No last_launch_test.env found"
    echo "         After calibration, run launch_test.sh in full TF mode (14 args) to apply calibrated roll/pitch/yaw values."
    ((WARN_COUNT++))
fi
echo ""

# Check 12: System resources
echo "[12/12] Checking system resources..."
CPU_COUNT=$(nproc)
MEM_TOTAL=$(free -g | awk '/^Mem:/{print $2}')

if [ "$CPU_COUNT" -lt 2 ]; then
    echo "  ⚠ WARN: Low CPU count: $CPU_COUNT cores"
    ((WARN_COUNT++))
else
    echo "  ✓ PASS: CPU cores: $CPU_COUNT"
fi

if [ "$MEM_TOTAL" -lt 2 ]; then
    echo "  ⚠ WARN: Low RAM: ${MEM_TOTAL}GB"
    ((WARN_COUNT++))
else
    echo "  ✓ PASS: RAM: ${MEM_TOTAL}GB"
fi
echo ""

# Summary
echo "========================================"
echo "Validation Summary"
echo "========================================"
if [ $FAIL_COUNT -eq 0 ] && [ $WARN_COUNT -eq 0 ]; then
    echo "✓ All checks passed - System ready for testing!"
    exit 0
elif [ $FAIL_COUNT -eq 0 ]; then
    echo "⚠ $WARN_COUNT warnings - System may work but check warnings"
    exit 0
else
    echo "✗ $FAIL_COUNT failures, $WARN_COUNT warnings"
    echo "Fix failures before testing"
    exit 1
fi
