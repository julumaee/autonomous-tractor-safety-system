#!/bin/bash
# validate_system.sh - Pre-test system validation
#
# Checks all hardware and software components before starting a test
# Run this before each test session to catch issues early
#
# Changes vs previous version:
# - Color-coded output (PASS=green, WARN=yellow, FAIL=red, INFO=cyan)
# - GPS check uses /navpvt (ublox_msgs/msg/NavPVT), NOT /gps/fix
# - "Accurate GPS available" check based on NavPVT fields (fix_type, num_sv, p_dop, h_acc)
# - More robust topic discovery and graceful behavior when ROS graph isn't reachable

set -euo pipefail

# ----------------------------
# Colors + helpers
# ----------------------------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

PASS() { echo -e "  ${GREEN}✓ PASS:${NC} $*"; }
WARN() { echo -e "  ${YELLOW}⚠ WARN:${NC} $*"; }
FAIL() { echo -e "  ${RED}✗ FAIL:${NC} $*"; }
INFO() { echo -e "  ${CYAN}- INFO:${NC} $*"; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

FAIL_COUNT=0
WARN_COUNT=0

TOPIC_LIST=""
NODE_LIST=""

inc_fail() { ((FAIL_COUNT++)) || true; }
inc_warn() { ((WARN_COUNT++)) || true; }

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

# ----------------------------
# ROS graph helpers
# ----------------------------
refresh_ros_graph() {
  TOPIC_LIST="$(timeout 2 ros2 topic list 2>/dev/null || true)"
  NODE_LIST="$(timeout 2 ros2 node list 2>/dev/null || true)"
}

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
  [[ -n "$count" ]] || count=0
  [[ "$count" -gt 0 ]]
}

check_topic_once() {
  local topic="$1"
  timeout 3 ros2 topic echo "$topic" --once &>/dev/null
}

check_topic_rate_warn() {
  # Warn-only topic rate check: if a publisher exists, try to get an average rate.
  local topic="$1"
  local expected_min_hz="$2"

  if ! topic_exists "$topic"; then
    INFO "$topic not present"
    return 1
  fi

  if ! topic_has_publisher "$topic"; then
    WARN "$topic exists but has no publisher"
    inc_warn
    return 1
  fi

  local hz=""
  hz="$(timeout 5 ros2 topic hz "$topic" 2>&1 | grep "average rate" | awk '{print $3}' | head -n 1 || true)"

  if [[ -n "$hz" ]] && command -v bc &>/dev/null && (( $(echo "$hz > $expected_min_hz" | bc -l 2>/dev/null || echo 0) )); then
    PASS "$topic publishing at ${hz} Hz"
    return 0
  fi

  if check_topic_once "$topic"; then
    PASS "$topic data received (rate not computed)"
    return 0
  fi

  WARN "$topic publisher exists but no data observed"
  inc_warn
  return 1
}

# ----------------------------
# NavPVT accuracy check
# ----------------------------
# We'll consider "accurate enough for tests" if:
#   - fix_type >= 3 (3D fix)  OR ideally 4/5 depending on receiver output (not assumed)
#   - num_sv >= 8 (configurable)
#   - p_dop <= 3.0 (configurable)
#   - h_acc <= 1.0 m (configurable)
#
# Note: ublox_msgs/NavPVT h_acc is in millimeters for UBX-NAV-PVT (u-blox spec).
# Many ROS drivers expose it raw. We convert mm -> m here.
NAVPVT_MIN_FIX_TYPE=3
NAVPVT_MIN_SV=8
NAVPVT_MAX_PDOP=3.0
NAVPVT_MAX_HACC_M=1.0

read_navpvt_snapshot() {
  # Prints: fix_type num_sv p_dop h_acc_m
  # Returns non-zero if cannot read.
  local out
  out="$(timeout 3 ros2 topic echo /navpvt --once 2>/dev/null || true)"
  [[ -n "$out" ]] || return 1

  local fix_type num_sv p_dop h_acc
  fix_type="$(printf '%s\n' "$out" | awk -F': ' '/^fix_type:/{print $2}' | head -n1)"
  num_sv="$(printf '%s\n' "$out" | awk -F': ' '/^num_sv:/{print $2}' | head -n1)"
  p_dop="$(printf '%s\n' "$out" | awk -F': ' '/^p_dop:/{print $2}' | head -n1)"
  h_acc="$(printf '%s\n' "$out" | awk -F': ' '/^h_acc:/{print $2}' | head -n1)"

  # Basic sanity
  [[ -n "$fix_type" && -n "$num_sv" && -n "$p_dop" && -n "$h_acc" ]] || return 1

  # Convert h_acc to meters assuming mm (common for NavPVT UBX)
  # If your driver already publishes meters, this will look too small.
  # We'll detect that: if h_acc < 50 and looks like "meters", keep as-is.
  local h_acc_m
  if command -v python3 &>/dev/null; then
    h_acc_m="$(python3 - <<PY
h=float("$h_acc")
# heuristic: if it's clearly huge (like 3000..50000), it's mm -> m
# if it's already small (<50), treat as meters
if h >= 50.0:
    print(h/1000.0)
else:
    print(h)
PY
)"
  else
    # fallback: assume mm
    h_acc_m="$(echo "$h_acc" | awk '{print $1/1000.0}')"
  fi

  echo "$fix_type $num_sv $p_dop $h_acc_m"
  return 0
}

navpvt_is_accurate_enough() {
  # Returns 0 if passes thresholds, 1 otherwise.
  local fix_type num_sv p_dop h_acc_m
  read -r fix_type num_sv p_dop h_acc_m <<< "$1"

  # fix_type + num_sv are ints
  if [[ "$fix_type" -lt "$NAVPVT_MIN_FIX_TYPE" ]]; then
    return 1
  fi
  if [[ "$num_sv" -lt "$NAVPVT_MIN_SV" ]]; then
    return 1
  fi

  # p_dop and h_acc_m floats
  if command -v bc &>/dev/null; then
    if ! (( $(echo "$p_dop <= $NAVPVT_MAX_PDOP" | bc -l 2>/dev/null || echo 0) )); then
      return 1
    fi
    if ! (( $(echo "$h_acc_m <= $NAVPVT_MAX_HACC_M" | bc -l 2>/dev/null || echo 0) )); then
      return 1
    fi
  else
    # Without bc, be conservative: fail the strict float checks
    return 1
  fi

  return 0
}

# ----------------------------
# Header
# ----------------------------
echo "========================================"
echo "System Validation - Pre-Test Checks"
echo "========================================"
echo ""

# ----------------------------
# [1] ROS 2 workspace
# ----------------------------
echo "[1/11] Checking ROS 2 workspace..."
if ! command -v ros2 &> /dev/null; then
  FAIL "ROS 2 not found in PATH"
  inc_fail
else
  if ! ros2 pkg list 2>/dev/null | grep -q "^tractor_safety_system_launch$"; then
    FAIL "Workspace not sourced (missing package tractor_safety_system_launch)"
    INFO "Run: source install/setup.bash"
    inc_fail
  else
    PASS "Workspace sourced correctly"
    refresh_ros_graph
  fi
fi
echo ""

# ----------------------------
# [2] CAN interface
# ----------------------------
echo "[2/11] Checking CAN interface..."
if ! ip link show can0 &>/dev/null; then
  FAIL "CAN interface can0 not found"
  INFO "Attach Kvaser device and run setup_can.sh"
  inc_fail
elif ! ip link show can0 | grep -q "UP"; then
  WARN "CAN interface exists but not UP"
  INFO "Run: sudo ip link set can0 up type can bitrate 1000000"
  inc_warn
else
  PASS "CAN interface can0 is UP"
  if command -v candump &>/dev/null; then
    if timeout 2 candump can0 -n 1 &>/dev/null; then
      PASS "CAN traffic detected"
    else
      WARN "No CAN traffic (radar may be off, or no detections yet)"
      inc_warn
    fi
  else
    WARN "candump not found (can-utils not installed) — skipping traffic check"
    inc_warn
  fi
fi
echo ""

# ----------------------------
# [3] GPS availability via NavPVT
# ----------------------------
echo "[3/11] Checking GPS data (NavPVT)..."

if ! topic_exists "/navpvt"; then
  FAIL "Topic /navpvt not found"
  INFO "Start GPS: cd testing_tools && ./gps_rtk_start.sh"
  inc_fail
else
  PASS "Topic /navpvt found"

  if ! topic_has_publisher "/navpvt"; then
    FAIL "/navpvt has no publishers"
    inc_fail
  else
    # Read one message
    SNAPSHOT="$(read_navpvt_snapshot || true)"
    if [[ -z "$SNAPSHOT" ]]; then
      WARN "Could not read a /navpvt message (timeout)"
      inc_warn
    else
      read -r fix_type num_sv p_dop h_acc_m <<< "$SNAPSHOT"
      INFO "NavPVT snapshot: fix_type=${fix_type}, num_sv=${num_sv}, p_dop=${p_dop}, h_acc≈${h_acc_m} m"

      # Rate check (warn)
      check_topic_rate_warn "/navpvt" 1 || true

      # Accuracy check (warn vs fail)
      if navpvt_is_accurate_enough "$SNAPSHOT"; then
        PASS "GPS accuracy looks good for testing (thresholds: fix_type>=${NAVPVT_MIN_FIX_TYPE}, sv>=${NAVPVT_MIN_SV}, p_dop<=${NAVPVT_MAX_PDOP}, h_acc<=${NAVPVT_MAX_HACC_M}m)"
      else
        WARN "GPS does NOT meet 'accurate' thresholds (may be indoors / multipath / too few sats)"
        INFO "Current: fix_type=${fix_type} (need >=${NAVPVT_MIN_FIX_TYPE}), sv=${num_sv} (need >=${NAVPVT_MIN_SV}), p_dop=${p_dop} (need <=${NAVPVT_MAX_PDOP}), h_acc=${h_acc_m}m (need <=${NAVPVT_MAX_HACC_M}m)"
        inc_warn
      fi
    fi
  fi
fi
echo ""

# ----------------------------
# [3.5] RTK Fix Status Check (NavSatFix)
# ----------------------------
echo "[3.5/11] Checking RTK fix status..."

if ! topic_exists "/gps/fix"; then
  WARN "Topic /gps/fix not found (may use different topic)"
  inc_warn
else
  GPS_FIX_OUT="$(timeout 3 ros2 topic echo /gps/fix --once 2>/dev/null || true)"
  if [[ -n "$GPS_FIX_OUT" ]]; then
    GPS_STATUS="$(printf '%s\n' "$GPS_FIX_OUT" | awk -F': ' '/^  status:/{print $2}' | head -n1 | tr -d ' ')"
    GPS_COVAR="$(printf '%s\n' "$GPS_FIX_OUT" | awk '/position_covariance:/{getline; gsub(/^[[:space:]]*-[[:space:]]*/, ""); print; exit}')"
    
    INFO "NavSatFix status: $GPS_STATUS (0=no_fix, 1=fix, 2=sbas_fix, 3=gbas_fix/rtk_float, 4=rtk_fixed)"
    INFO "Position covariance[0]: ${GPS_COVAR} m² (lower is better)"
    
    case "$GPS_STATUS" in
      4)
        PASS "RTK FIXED solution achieved! (cm-level accuracy)"
        ;;
      3)
        WARN "RTK FLOAT solution (decimeter accuracy, waiting for fixed)"
        inc_warn
        ;;
      2)
        WARN "SBAS fix only (meter-level accuracy)"
        inc_warn
        ;;
      1)
        WARN "Standard GPS fix (no RTK corrections being applied)"
        INFO "Check RTCM stream below"
        inc_warn
        ;;
      0)
        FAIL "No GPS fix"
        inc_fail
        ;;
      *)
        WARN "Unknown GPS status: $GPS_STATUS"
        inc_warn
        ;;
    esac
  else
    WARN "Could not read /gps/fix message"
    inc_warn
  fi
fi
echo ""

# ----------------------------
# [3.6] RTCM Corrections Check
# ----------------------------
echo "[3.6/11] Checking RTCM corrections stream..."

if ! topic_exists "/rtcm"; then
  FAIL "Topic /rtcm not found"
  INFO "NTRIP client not running or incorrect topic name"
  INFO "Start with: cd testing_tools && ./gps_rtk_start.sh"
  inc_fail
else
  PASS "Topic /rtcm found"
  
  if ! topic_has_publisher "/rtcm"; then
    FAIL "/rtcm has no publishers"
    INFO "NTRIP client may have crashed or failed to connect"
    inc_fail
  else
    RTCM_RATE="$(timeout 8 ros2 topic hz /rtcm 2>&1 | grep "average rate" | awk '{print $3}' | head -n 1 || true)"
    
    if [[ -n "$RTCM_RATE" ]] && command -v bc &>/dev/null && (( $(echo "$RTCM_RATE > 1.0" | bc -l 2>/dev/null || echo 0) )); then
      PASS "RTCM stream active at ${RTCM_RATE} Hz (good for RTK)"
      
      if (( $(echo "$RTCM_RATE < 0.5" | bc -l 2>/dev/null || echo 0) )); then
        WARN "RTCM rate low (< 0.5 Hz) - may be unstable"
        inc_warn
      fi
    else
      WARN "RTCM stream rate could not be determined (may be too slow or no messages yet)"
      INFO "Wait a few seconds for NTRIP connection to stabilize"
      inc_warn
    fi
    
    # Check if we can read at least one RTCM message
    if check_topic_once "/rtcm"; then
      PASS "RTCM message received successfully"
    else
      WARN "No RTCM message received in 3 seconds"
      INFO "Check NTRIP mountpoint/credentials, or caster may be down"
      inc_warn
    fi
  fi
fi
echo ""

# ----------------------------
# [3.7] NTRIP Client Status
# ----------------------------
echo "[3.7/11] Checking NTRIP client node..."

if printf '%s\n' "$NODE_LIST" | grep -q "ntrip_client"; then
  PASS "NTRIP client node is running"
  
  # Try to check for common NTRIP error patterns in logs
  if command -v journalctl &>/dev/null; then
    NTRIP_ERRORS="$(journalctl --user -u "ros*" --since "5 minutes ago" 2>/dev/null | grep -i "ntrip.*\(sourcetable\|error\|failed\|invalid\)" | tail -n 3 || true)"
    if [[ -n "$NTRIP_ERRORS" ]]; then
      WARN "Recent NTRIP errors detected in logs:"
      printf '%s\n' "$NTRIP_ERRORS" | while IFS= read -r line; do
        INFO "  $line"
      done
      inc_warn
    fi
  fi
else
  WARN "NTRIP client node not detected"
  INFO "If using NTRIP, launch with: cd testing_tools && ./gps_rtk_start.sh"
  inc_warn
fi
echo ""

# ----------------------------
# [4] Sensor topics (meaningful if stack already running)
# ----------------------------
echo "[4/15] Checking sensor topics (if running)..."
check_topic_rate_warn "/radar_detections" 1 || true
check_topic_rate_warn "/camera_detections" 1 || true
check_topic_rate_warn "/fused_detections" 1 || true
check_topic_rate_warn "/tracked_detections" 1 || true
check_topic_rate_warn "/ego_motion" 1 || true
echo ""

# ----------------------------
# [5] GPS USB device check
# ----------------------------
echo "[5/15] Checking GPS USB device..."
if [[ -e "/dev/ttyACM0" ]]; then
  PASS "GPS device /dev/ttyACM0 exists"
  if [[ -r "/dev/ttyACM0" && -w "/dev/ttyACM0" ]]; then
    PASS "GPS device is readable/writable"
  else
    WARN "GPS device exists but may have permission issues"
    INFO "Run: sudo chmod 666 /dev/ttyACM0 (or add user to dialout group)"
    inc_warn
  fi
else
  FAIL "GPS device /dev/ttyACM0 not found"
  INFO "Check USB connection or try: ls /dev/ttyACM* /dev/ttyUSB*"
  inc_fail
fi
echo ""

# ----------------------------
# [6] OAK-D camera presence (USB)
# ----------------------------
echo "[6/15] Checking OAK-D camera..."
if ! command -v lsusb &>/dev/null; then
  WARN "lsusb not found — cannot check USB camera presence"
  inc_warn
elif ! lsusb | grep -iE "movidius|myriadx|luxonis|oak" &>/dev/null; then
  FAIL "OAK-D camera not detected via USB"
  INFO "Check USB connection / cable / power"
  inc_fail
else
  PASS "OAK-D camera detected via USB"
fi
echo ""

# ----------------------------
# [7] Required executables
# ----------------------------
echo "[7/15] Checking ROS 2 executables..."
EXECUTABLES=(
  "simulations:gps_ego_motion"
  "simulations:real_world_logger"
  "radar_interface:radar_node"
  "camera_interface:camera_node"
  "sensor_fusion:fusion_node"
  "sensor_fusion:kf_tracker"
)

for exec in "${EXECUTABLES[@]}"; do
  PKG="${exec%%:*}"
  NODE="${exec##*:}"
  if ros2 pkg executables "$PKG" 2>/dev/null | awk '{print $2}' | grep -qx "$NODE"; then
    PASS "$PKG/$NODE found"
  else
    FAIL "$PKG/$NODE not found"
    INFO "Rebuild: colcon build --packages-select $PKG"
    inc_fail
  fi
done
echo ""

# ----------------------------
# [8] Log directory
# ----------------------------
echo "[8/15] Checking log directory..."
if [[ ! -d "real_world_logs" ]]; then
  WARN "real_world_logs directory doesn't exist (will be created by logger)"
  inc_warn
else
  PASS "real_world_logs directory exists"
  if [[ -w "real_world_logs" ]]; then
    PASS "Directory is writable"
  else
    FAIL "Directory not writable"
    inc_fail
  fi
fi
echo ""

# ----------------------------
# [9] Disk space
# ----------------------------
echo "[9/15] Checking disk space..."
DISK_AVAIL="$(df -h . | tail -1 | awk '{print $4}')"
DISK_AVAIL_GB="$(df -BG . | tail -1 | awk '{print $4}' | sed 's/G//')"
if [[ "${DISK_AVAIL_GB}" -lt 1 ]]; then
  FAIL "Low disk space: $DISK_AVAIL available"
  inc_fail
elif [[ "${DISK_AVAIL_GB}" -lt 5 ]]; then
  WARN "Disk space low: $DISK_AVAIL available"
  inc_warn
else
  PASS "Sufficient disk space: $DISK_AVAIL available"
fi
echo ""

# ----------------------------
# [10] CAN utilities
# ----------------------------
echo "[10/15] Checking CAN utilities..."
if ! command -v candump &> /dev/null; then
  FAIL "candump not found"
  INFO "Install: sudo apt install can-utils"
  inc_fail
else
  PASS "CAN utilities installed"
fi
echo ""

# ----------------------------
# [11] Launch files
# ----------------------------
echo "[11/15] Checking required launch files..."
if ros2 launch tractor_safety_system_launch perception_stack.launch.py --show-args &>/dev/null; then
  PASS "perception_stack.launch.py found"
else
  FAIL "perception_stack.launch.py not found"
  INFO "Rebuild: colcon build --packages-select tractor_safety_system_launch"
  inc_fail
fi

if ros2 launch tractor_safety_system_launch real_world_test.launch.py --show-args &>/dev/null; then
  PASS "real_world_test.launch.py found"
else
  FAIL "real_world_test.launch.py not found"
  INFO "Rebuild: colcon build --packages-select tractor_safety_system_launch"
  inc_fail
fi
echo ""

# ----------------------------
# [12] System resources
# ----------------------------
echo "[12/15] Checking system resources..."
CPU_COUNT="$(nproc)"
MEM_TOTAL="$(free -g | awk '/^Mem:/{print $2}')"

if [[ "$CPU_COUNT" -lt 2 ]]; then
  WARN "Low CPU count: $CPU_COUNT cores"
  inc_warn
else
  PASS "CPU cores: $CPU_COUNT"
fi

if [[ "$MEM_TOTAL" -lt 2 ]]; then
  WARN "Low RAM: ${MEM_TOTAL}GB"
  inc_warn
else
  PASS "RAM: ${MEM_TOTAL}GB"
fi
echo ""

# ----------------------------
# [13] System temperature (cold weather check)
# ----------------------------
echo "[13/15] Checking system temperature..."
if command -v sensors &>/dev/null; then
  TEMP_OUTPUT="$(sensors 2>/dev/null | grep -E "Core|temp1" | head -n 1 || true)"
  if [[ -n "$TEMP_OUTPUT" ]]; then
    INFO "Temperature: $TEMP_OUTPUT"
    PASS "Temperature sensors accessible"
  else
    INFO "Temperature data not available from sensors"
  fi
else
  INFO "lm-sensors not installed (optional for temperature monitoring)"
fi

# Check for throttling (Raspberry Pi/similar)
if [[ -f "/sys/class/thermal/thermal_zone0/temp" ]]; then
  THERMAL_TEMP="$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo "0")"
  THERMAL_TEMP_C="$((THERMAL_TEMP / 1000))"
  INFO "SoC temperature: ${THERMAL_TEMP_C}°C"
  
  if [[ "$THERMAL_TEMP_C" -gt 80 ]]; then
    WARN "System temperature high (${THERMAL_TEMP_C}°C) - may throttle"
    inc_warn
  elif [[ "$THERMAL_TEMP_C" -lt 5 ]]; then
    WARN "System temperature very low (${THERMAL_TEMP_C}°C) - cold weather operation"
    INFO "Allow system to warm up if possible, monitor for sensor freezing"
    inc_warn
  fi
fi

# Battery check for laptops
if command -v acpi &>/dev/null; then
  BATTERY_INFO="$(acpi -b 2>/dev/null | head -n 1 || true)"
  if [[ -n "$BATTERY_INFO" ]]; then
    INFO "Battery: $BATTERY_INFO"
    
    if echo "$BATTERY_INFO" | grep -qE "([0-9]+)%" && ! echo "$BATTERY_INFO" | grep -q "Charging"; then
      BATTERY_PCT="$(echo "$BATTERY_INFO" | grep -oE "[0-9]+%" | head -n1 | tr -d '%')"
      if [[ "$BATTERY_PCT" -lt 20 ]]; then
        WARN "Battery low (${BATTERY_PCT}%) - connect to power for outdoor testing"
        inc_warn
      elif [[ "$BATTERY_PCT" -lt 50 ]]; then
        INFO "Battery at ${BATTERY_PCT}% - consider connecting to power for long tests"
      else
        PASS "Battery sufficient (${BATTERY_PCT}%)"
      fi
    fi
  fi
fi
echo ""

# ----------------------------
# [14] USB device stability
# ----------------------------
echo "[14/15] Checking USB device stability..."
if command -v dmesg &>/dev/null; then
  USB_ERRORS="$(dmesg -T 2>/dev/null | tail -n 100 | grep -iE "usb.*disconnect|usb.*reset|usb.*error" | tail -n 3 || true)"
  if [[ -n "$USB_ERRORS" ]]; then
    WARN "Recent USB errors detected:"
    printf '%s\n' "$USB_ERRORS" | while IFS= read -r line; do
      INFO "  $line"
    done
    INFO "Cold weather can cause USB connection issues - ensure cables are secure"
    inc_warn
  else
    PASS "No recent USB errors detected"
  fi
else
  INFO "dmesg not accessible (requires permissions)"
fi
echo ""

# ----------------------------
# [15] Time synchronization
# ----------------------------
echo "[15/15] Checking time synchronization..."
if command -v timedatectl &>/dev/null; then
  TIME_SYNC="$(timedatectl status 2>/dev/null | grep -E "System clock synchronized|NTP service" || true)"
  if echo "$TIME_SYNC" | grep -q "yes"; then
    PASS "System clock synchronized"
  else
    WARN "System clock may not be synchronized"
    INFO "Time sync important for GPS timestamping: sudo timedatectl set-ntp true"
    inc_warn
  fi
else
  INFO "timedatectl not available"
fi
echo ""

# ----------------------------
# Summary
# ----------------------------
echo "========================================"
echo "Validation Summary"
echo "========================================"

if [[ $FAIL_COUNT -eq 0 && $WARN_COUNT -eq 0 ]]; then
  echo -e "${GREEN}✓ All checks passed — System ready for testing!${NC}"
  exit 0
elif [[ $FAIL_COUNT -eq 0 ]]; then
  echo -e "${YELLOW}⚠ ${WARN_COUNT} warnings — System may work but check warnings.${NC}"
  exit 0
else
  echo -e "${RED}✗ ${FAIL_COUNT} failures, ${WARN_COUNT} warnings${NC}"
  echo -e "${RED}Fix failures before testing.${NC}"
  exit 1
fi