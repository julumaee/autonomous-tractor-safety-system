#!/bin/bash
# monitor_test.sh - Real-time monitoring dashboard for test execution
# 
# Displays system health and topic rates during testing
# Run this in a separate terminal while the test is running

echo "========================================"
echo "Real-World Test Monitor"
echo "========================================"
echo "Press Ctrl+C to exit"
echo "========================================"
echo ""

if [[ "${1:-}" == "--domain-id" ]] && [[ -n "${2:-}" ]]; then
    export ROS_DOMAIN_ID="$2"
    shift 2
fi

if [[ "${1:-}" == "--once" ]]; then
    RUN_ONCE=true
    shift
else
    RUN_ONCE=false
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Check if ROS is available
if ! command -v ros2 &> /dev/null; then
    echo "✗ ERROR: ROS 2 not found"
    exit 1
fi

# Best-effort: source workspace if it looks unsourced.
# (This is mainly for convenience; discovery itself doesn't require workspace sourcing.)
if ! ros2 pkg list 2>/dev/null | grep -q "^tractor_safety_system_launch$"; then
    if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
        # shellcheck disable=SC1091
        source "${WORKSPACE_ROOT}/install/setup.bash"
    fi
fi

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to check topic rate
check_topic() {
    local topic=$1
    local expected_min=$2
    local topic_list=$3
    local hz_timeout=4

    if ! printf '%s\n' "$topic_list" | grep -qx "$topic"; then
        echo -e "${RED}✗ Missing${NC}"
        return 1
    fi

    local pub_count
    pub_count="$(timeout 2 ros2 topic info -v "$topic" 2>/dev/null | awk -F': ' '/^Publisher count:/{print $2}' | head -n 1)"
    if [[ -z "$pub_count" ]]; then
        pub_count=0
    fi
    if [[ "$pub_count" -eq 0 ]]; then
        echo -e "${YELLOW}⚠ No publisher${NC}"
        return 1
    fi

    # Prefer hz (gives a useful number), but it may not print an average before timeout.
    local rate_output
    rate_output="$(timeout ${hz_timeout} ros2 topic hz "$topic" 2>&1 || true)"
    local rate
    rate="$(echo "$rate_output" | grep -E "average rate" | awk '{print $3}' | head -n 1)"
    if [[ -n "$rate" ]]; then
        if command -v bc &>/dev/null && (( $(echo "$rate > $expected_min" | bc -l 2>/dev/null || echo 0) )); then
            echo -e "${GREEN}✓ ${rate} Hz${NC}"
            return 0
        fi
        echo -e "${YELLOW}⚠ ${rate} Hz (low)${NC}"
        return 1
    fi

    # Fallback: if we can receive at least one message quickly, report data present.
    if timeout 2 ros2 topic echo "$topic" --once &>/dev/null; then
        echo -e "${GREEN}✓ Data${NC}"
        return 0
    fi

    echo -e "${RED}✗ No data${NC}"
    return 1
}

# Function to check node status
check_node() {
    local node=$1
    local node_list=$2
    # Match either exact node name or the same basename under a namespace.
    # E.g. both '/radar_node' and '/field_test/radar_node' should count as running.
    if printf '%s\n' "$node_list" | grep -qE "${node}$"; then
        echo -e "${GREEN}✓ Running${NC}"
        return 0
    else
        echo -e "${RED}✗ Not found${NC}"
        return 1
    fi
}

# Function to get latest log file size
check_log_size() {
    local pattern=$1
    local latest_file=$(ls -t real_world_logs/${pattern}_*.csv 2>/dev/null | head -1)
    
    if [ -z "$latest_file" ]; then
        echo -e "${RED}✗ No file${NC}"
        return 1
    fi
    
    local size=$(du -h "$latest_file" 2>/dev/null | cut -f1)
    echo -e "${GREEN}$size${NC}"
    return 0
}

# Main monitoring loop
while true; do
    clear
    NODE_LIST="$(timeout 2 ros2 node list 2>/dev/null || true)"
    TOPIC_LIST="$(timeout 2 ros2 topic list 2>/dev/null || true)"
    echo "========================================"
    echo "Real-World Test Monitor"
    echo "$(date '+%Y-%m-%d %H:%M:%S')"
    echo "========================================"
    echo ""

    echo "ROS Context:"
    echo "----------------------------------------"
    echo "  ROS_DOMAIN_ID:         ${ROS_DOMAIN_ID:-<unset>}"
    echo "  RMW_IMPLEMENTATION:    ${RMW_IMPLEMENTATION:-<unset>}"
    echo "  ROS_LOCALHOST_ONLY:    ${ROS_LOCALHOST_ONLY:-<unset>}"
    echo ""

    if [[ -z "$NODE_LIST" ]]; then
        echo -e "${YELLOW}⚠ No nodes discovered${NC}"
        echo "  Tips:"
        echo "   - Ensure this terminal uses the same ROS_DOMAIN_ID as the launch terminal"
        echo "   - Try: ros2 daemon stop && ros2 daemon start"
        echo ""
    else
        NODE_COUNT="$(printf '%s\n' "$NODE_LIST" | wc -l | tr -d ' ')"
        echo "Discovered Nodes: $NODE_COUNT"
        printf '%s\n' "$NODE_LIST" | head -n 8 | sed 's/^/  - /'
        echo ""
    fi
    
    # Node Status
    echo "Node Status:"
    echo "----------------------------------------"
    printf "  Radar Node:           "
    check_node "/radar_node" "$NODE_LIST"
    printf "  Camera Node:          "
    check_node "/camera_node" "$NODE_LIST"
    printf "  GPS Ego Motion:       "
    check_node "/gps_ego_motion" "$NODE_LIST"
    printf "  Fusion Node:          "
    check_node "/fusion_node" "$NODE_LIST"
    printf "  KF Tracker:           "
    check_node "/kf_tracker" "$NODE_LIST"
    printf "  Logger:               "
    check_node "/real_world_logger" "$NODE_LIST"
    echo ""
    
    # Topic Rates
    echo "Topic Rates:"
    echo "----------------------------------------"
    printf "  /ego_motion:          "
    check_topic "/ego_motion" 5 "$TOPIC_LIST"
    printf "  /radar_detections:    "
    check_topic "/radar_detections" 1 "$TOPIC_LIST"

    printf "  /camera_detections:   "
    if printf '%s\n' "$NODE_LIST" | grep -qE "/camera_node$"; then
        check_topic "/camera_detections" 1 "$TOPIC_LIST"
    else
        echo -e "${YELLOW}⚠ Camera disabled${NC}"
    fi

    printf "  /fused_detections:    "
    check_topic "/fused_detections" 1 "$TOPIC_LIST"
    printf "  /tracked_detections:  "
    if printf '%s\n' "$NODE_LIST" | grep -qE "/kf_tracker$"; then
        check_topic "/tracked_detections" 5 "$TOPIC_LIST"
    else
        echo -e "${YELLOW}⚠ Tracker disabled${NC}"
    fi
    echo ""
    
    # Log File Sizes
    echo "Log Files (latest):"
    echo "----------------------------------------"
    printf "  Raw detections:       "
    check_log_size "raw_detections"
    printf "  Fused detections:     "
    check_log_size "fused_detections"
    printf "  Tracks:               "
    check_log_size "tracks"
    printf "  Ego motion:           "
    check_log_size "ego_motion"
    echo ""
    
    # CAN Interface Status
    echo "CAN Interface:"
    echo "----------------------------------------"
    CAN_IFACE="$(ros2 param get /radar_node can_channel 2>/dev/null | awk -F': ' '/String value is:/{print $2}' | tr -d '\r' | head -n 1)"
    if [[ -z "$CAN_IFACE" ]]; then
        CAN_IFACE="can0"
    fi

    if ip link show "$CAN_IFACE" 2>/dev/null | grep -q "UP"; then
        printf "  Status:               "
        echo -e "${GREEN}✓ UP (${CAN_IFACE})${NC}"
        
        # Check for CAN traffic
        if timeout 1 candump "$CAN_IFACE" -n 1 &>/dev/null; then
            printf "  Traffic:              "
            echo -e "${GREEN}✓ Active${NC}"
        else
            printf "  Traffic:              "
            echo -e "${YELLOW}⚠ No data${NC}"
        fi
    else
        printf "  Status:               "
        echo -e "${RED}✗ DOWN (${CAN_IFACE})${NC}"
    fi
    echo ""
    
    # System Resources
    echo "System Resources:"
    echo "----------------------------------------"
    
    # CPU usage
    CPU_USAGE=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)
    printf "  CPU Usage:            "
    if (( $(echo "$CPU_USAGE > 80" | bc -l 2>/dev/null || echo 0) )); then
        echo -e "${RED}${CPU_USAGE}%${NC}"
    elif (( $(echo "$CPU_USAGE > 60" | bc -l 2>/dev/null || echo 0) )); then
        echo -e "${YELLOW}${CPU_USAGE}%${NC}"
    else
        echo -e "${GREEN}${CPU_USAGE}%${NC}"
    fi
    
    # Memory usage
    MEM_USAGE=$(free | grep Mem | awk '{printf("%.1f", $3/$2 * 100.0)}')
    printf "  Memory Usage:         "
    if (( $(echo "$MEM_USAGE > 80" | bc -l 2>/dev/null || echo 0) )); then
        echo -e "${RED}${MEM_USAGE}%${NC}"
    elif (( $(echo "$MEM_USAGE > 60" | bc -l 2>/dev/null || echo 0) )); then
        echo -e "${YELLOW}${MEM_USAGE}%${NC}"
    else
        echo -e "${GREEN}${MEM_USAGE}%${NC}"
    fi
    
    # Disk space
    DISK_AVAIL=$(df -h . | tail -1 | awk '{print $4}')
    printf "  Disk Available:       "
    echo -e "${GREEN}${DISK_AVAIL}${NC}"
    
    echo ""
    echo "========================================"
    echo "Refreshing in 3 seconds... (Ctrl+C to exit)"

    if [[ "$RUN_ONCE" == "true" ]]; then
        exit 0
    fi

    sleep 3
done
