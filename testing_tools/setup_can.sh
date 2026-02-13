#!/bin/bash
# setup_can.sh - Configure CAN interface for Nanoradar SR75
# 
# Usage:
#   ./setup_can.sh [INTERFACE] [BITRATE]
#
# Examples:
#   ./setup_can.sh              (uses defaults: can0, 1000000)
#   ./setup_can.sh can0 1000000
#   ./setup_can.sh can1 500000

CAN_INTERFACE=${1:-"can0"}
BITRATE=${2:-"1000000"}

echo "========================================"
echo "CAN Interface Setup"
echo "========================================"
echo "Interface:     $CAN_INTERFACE"
echo "Bitrate:       $BITRATE bps"
echo "========================================"
echo ""

# Check if running as root (needed for ip commands)
if [ "$EUID" -ne 0 ]; then 
    echo "This script needs sudo privileges for 'ip link' commands"
    echo "Re-running with sudo..."
    sudo "$0" "$@"
    exit $?
fi

# Step 1: Load Kvaser kernel module
echo "[1/4] Loading Kvaser USB kernel module..."
if modprobe kvaser_usb 2>/dev/null; then
    echo "  ✓ kvaser_usb module loaded"
else
    echo "  ⚠ Warning: Could not load kvaser_usb (may already be loaded)"
fi
echo ""

# Step 2: Check if interface exists
echo "[2/4] Checking for CAN interface..."
if ip link show $CAN_INTERFACE &>/dev/null; then
    echo "  ✓ Interface $CAN_INTERFACE found"
else
    echo "  ✗ ERROR: Interface $CAN_INTERFACE not found"
    echo ""
    echo "Available CAN interfaces:"
    ip link show | grep -E "can[0-9]" || echo "  None found"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Check USB connection: lsusb | grep -i kvaser"
    echo "  2. Check dmesg: dmesg | grep -i kvaser"
    echo "  3. Try: modprobe -r kvaser_usb && modprobe kvaser_usb"
    exit 1
fi
echo ""

# Step 3: Bring down interface if already up
echo "[3/4] Configuring CAN interface..."
ip link set $CAN_INTERFACE down 2>/dev/null || true
echo "  ✓ Interface brought down"

# Set bitrate and bring up
ip link set $CAN_INTERFACE type can bitrate $BITRATE
echo "  ✓ Bitrate set to $BITRATE bps"

ip link set $CAN_INTERFACE up
echo "  ✓ Interface brought up"
echo ""

# Step 4: Verify interface is up
echo "[4/4] Verifying interface status..."
if ip link show $CAN_INTERFACE | grep -q "UP"; then
    echo "  ✓ Interface $CAN_INTERFACE is UP and ready"
    echo ""
    
    # Show interface details
    echo "Interface details:"
    ip -details link show $CAN_INTERFACE | grep -E "can|bitrate"
    echo ""
    
    # Test for CAN traffic
    echo "Testing for CAN traffic (5 second timeout)..."
    echo "Waiting for CAN messages..."
    if timeout 5 candump $CAN_INTERFACE -n 1 &>/dev/null; then
        echo "  ✓ CAN traffic detected!"
        echo ""
        echo "First few CAN messages:"
        timeout 2 candump $CAN_INTERFACE 2>&1 | head -5
    else
        echo "  ⚠ No CAN traffic detected"
        echo "    This is normal if:"
        echo "    - Radar is not powered on"
        echo "    - No objects are detected by radar"
        echo "    - Radar is not connected to CAN bus"
    fi
else
    echo "  ✗ ERROR: Failed to bring up interface"
    exit 1
fi

echo ""
echo "========================================"
echo "CAN Setup Complete!"
echo "========================================"
echo ""
echo "Quick test commands:"
echo "  candump $CAN_INTERFACE              # Monitor CAN traffic"
echo "  cansniffer $CAN_INTERFACE           # Interactive CAN monitor"
echo "  ip -s link show $CAN_INTERFACE      # Interface statistics"
echo ""
echo "To bring down the interface:"
echo "  sudo ip link set $CAN_INTERFACE down"
echo ""
