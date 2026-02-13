#!/bin/bash
# gps_rtk_start.sh - interactive helper to start u-blox + (optional) NTRIP for ArduSimple RTK2B
#
# Usage:
#   ./gps_rtk_start.sh            # start u-blox + NTRIP (prompts for NTRIP creds)
#   ./gps_rtk_start.sh --no-ntrip # start u-blox only
#
# Notes:
# - This script prompts for credentials so you don't need to export them or put them in shell history.
# - If your `ublox_gps`/`ntrip_client` executables differ, override via launch args (see launch file help).

set -euo pipefail

NO_NTRIP=false
GPS_DEVICE_ARG=""
GPS_BAUDRATE_ARG=""
NTRIP_VERSION_ARG=""

usage() {
  echo "Usage:" >&2
  echo "  ./gps_rtk_start.sh [--no-ntrip] [--ntrip-version <Ntrip/2.0>] [--device <path>] [--baudrate <baud>]" >&2
  echo "" >&2
  echo "Non-interactive (optional): set env vars NTRIP_HOST/NTRIP_PORT/NTRIP_MOUNTPOINT/NTRIP_USERNAME/NTRIP_PASSWORD/NTRIP_VERSION." >&2
}

source_setup_file() {
  local setup_file="$1"
  if [[ -f "$setup_file" ]]; then
    # Colcon/ament setup scripts are not guaranteed to be safe under `set -u`.
    # Disable nounset just for sourcing, then restore strict mode.
    set +u
    # shellcheck disable=SC1090
    source "$setup_file"
    set -u
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-ntrip)
      NO_NTRIP=true
      shift
      ;;
    --ntrip-version)
      NTRIP_VERSION_ARG="${2:-}"
      shift 2
      ;;
    --device)
      GPS_DEVICE_ARG="${2:-}"
      shift 2
      ;;
    --baudrate)
      GPS_BAUDRATE_ARG="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: Unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Best-effort: source a ROS distro so `ros2` becomes available in fresh shells.
if ! command -v ros2 &>/dev/null; then
  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    source_setup_file "/opt/ros/${ROS_DISTRO}/setup.bash"
  elif [[ -f "/opt/ros/jazzy/setup.bash" ]]; then
    source_setup_file "/opt/ros/jazzy/setup.bash"
  fi
fi

if ! command -v ros2 &>/dev/null; then
  echo "ERROR: ros2 not found in PATH. Source your ROS 2 environment first." >&2
  exit 1
fi

ROS2_PKG_LIST="$(ros2 pkg list 2>/dev/null || true)"

# Prefer this workspace overlay when available.
if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  source_setup_file "${WORKSPACE_ROOT}/install/setup.bash"
  ROS2_PKG_LIST="$(ros2 pkg list 2>/dev/null || true)"
fi

if ! grep -q "^ublox_gps$" <<<"$ROS2_PKG_LIST"; then
  echo "ERROR: ublox_gps package not installed." >&2
  echo "  Install: sudo apt install ros-${ROS_DISTRO:-jazzy}-ublox-gps" >&2
  exit 1
fi

if ! $NO_NTRIP && ! grep -q "^ntrip_client$" <<<"$ROS2_PKG_LIST"; then
  echo "ERROR: ntrip_client package not installed." >&2
  echo "  Install: sudo apt install ros-${ROS_DISTRO:-jazzy}-ntrip-client" >&2
  exit 1
fi

DEFAULT_GPS_DEVICE="/dev/ttyACM0"
if [[ -d /dev/serial/by-id ]]; then
  UBLOX_CANDIDATES=(/dev/serial/by-id/usb-u-blox*)
  if [[ -e "${UBLOX_CANDIDATES[0]}" ]]; then
    DEFAULT_GPS_DEVICE="${UBLOX_CANDIDATES[0]}"
  fi
fi

if [[ -d /dev/serial/by-id ]]; then
  echo "Detected serial devices (/dev/serial/by-id):"
  ls -l /dev/serial/by-id/ 2>/dev/null || true
  echo ""
fi

GPS_DEVICE="${GPS_DEVICE_ARG:-${GPS_DEVICE:-}}"
if [[ -z "${GPS_DEVICE}" ]]; then
  if [[ -t 0 ]]; then
    read -r -p "GPS serial device [${DEFAULT_GPS_DEVICE}]: " GPS_DEVICE
    GPS_DEVICE=${GPS_DEVICE:-$DEFAULT_GPS_DEVICE}
  else
    GPS_DEVICE="$DEFAULT_GPS_DEVICE"
  fi
fi

if [[ ! -e "$GPS_DEVICE" ]]; then
  echo "ERROR: GPS device does not exist: $GPS_DEVICE" >&2
  exit 1
fi

GPS_BAUDRATE="${GPS_BAUDRATE_ARG:-${GPS_BAUDRATE:-}}"
if [[ -z "${GPS_BAUDRATE}" ]]; then
  if [[ -t 0 ]]; then
    read -r -p "GPS baudrate [115200]: " GPS_BAUDRATE
    GPS_BAUDRATE=${GPS_BAUDRATE:-115200}
  else
    GPS_BAUDRATE=115200
  fi
fi

GPS_FIX_TOPIC=/gps/fix
RTCM_TOPIC=/rtcm

START_NTRIP=true
NTRIP_AUTHENTICATE=false
NTRIP_HOST="rtk2go.com"
NTRIP_PORT=2101
NTRIP_MOUNTPOINT="Ranta"
NTRIP_USERNAME=""
NTRIP_PASSWORD=""
NTRIP_VERSION="${NTRIP_VERSION_ARG:-${NTRIP_VERSION:-}}"

if $NO_NTRIP; then
  START_NTRIP=false
else
  if [[ -t 0 ]]; then
    read -r -p "NTRIP host [${NTRIP_HOST}]: " NTRIP_HOST_IN
    NTRIP_HOST=${NTRIP_HOST_IN:-$NTRIP_HOST}
    read -r -p "NTRIP port [2101]: " NTRIP_PORT_IN
    NTRIP_PORT=${NTRIP_PORT_IN:-2101}
    read -r -p "NTRIP mountpoint [${NTRIP_MOUNTPOINT}]: " NTRIP_MOUNTPOINT_IN
    NTRIP_MOUNTPOINT=${NTRIP_MOUNTPOINT_IN:-$NTRIP_MOUNTPOINT}
    read -r -p "NTRIP username (email for rtk2go.com; optional for some public casters): " NTRIP_USERNAME
    read -r -s -p "NTRIP password (required by this ntrip_client when authenticating; can be any non-empty string for some casters): " NTRIP_PASSWORD
    echo ""
  else
    NTRIP_HOST="${NTRIP_HOST:-rtk2go.com}"
    NTRIP_PORT="${NTRIP_PORT:-2101}"
    NTRIP_MOUNTPOINT="${NTRIP_MOUNTPOINT:-Ranta}"
    NTRIP_USERNAME="${NTRIP_USERNAME:-}"
    NTRIP_PASSWORD="${NTRIP_PASSWORD:-}"
    NTRIP_VERSION="${NTRIP_VERSION:-}"
  fi

  if [[ -z "$NTRIP_HOST" || -z "$NTRIP_MOUNTPOINT" ]]; then
    echo "ERROR: NTRIP host and mountpoint are required (or run with --no-ntrip)." >&2
    exit 2
  fi

  # Version header: RTK2go is known to require Ntrip/2.0.
  if [[ -z "${NTRIP_VERSION}" ]]; then
    if [[ "$NTRIP_HOST" =~ rtk2go\.com$ ]]; then
      NTRIP_VERSION="Ntrip/2.0"
    else
      NTRIP_VERSION="Ntrip/2.0"
    fi
  fi

  if [[ -n "$NTRIP_USERNAME" ]]; then
    # Some casters (notably rtk2go.com) require a (email) username even if password is empty.
    NTRIP_AUTHENTICATE=true
    if [[ "$NTRIP_HOST" =~ rtk2go\.com$ ]] && [[ ! "$NTRIP_USERNAME" =~ .+@.+\..+ ]]; then
      echo "WARNING: RTK2go typically requires an email-form username (e.g. name@example.com)." >&2
    fi
  elif [[ -n "$NTRIP_PASSWORD" ]]; then
    NTRIP_AUTHENTICATE=false
    echo "WARNING: Password provided but username is empty; using anonymous mode." >&2
  else
    NTRIP_AUTHENTICATE=false
    if [[ "$NTRIP_HOST" =~ rtk2go\.com$ ]]; then
      echo "WARNING: RTK2go usually requires authentication (email username). If you get a SOURCETABLE response, rerun and enter your email as username." >&2
    fi
  fi

  if [[ "${NTRIP_AUTHENTICATE}" == "true" && -z "${NTRIP_PASSWORD}" ]]; then
    if [[ -t 0 ]]; then
      read -r -s -p "NTRIP password is required when authenticating. Enter password (can be dummy for some casters): " NTRIP_PASSWORD
      echo ""
    fi

    if [[ -z "${NTRIP_PASSWORD}" ]]; then
      echo "ERROR: Authentication requested but password is empty. This ntrip_client node requires a non-empty password." >&2
      exit 2
    fi
  fi
fi

echo "Starting GPS stack..."
echo "- device:      $GPS_DEVICE"
echo "- baudrate:    $GPS_BAUDRATE"
echo "- fix topic:   $GPS_FIX_TOPIC"
echo "- start_ntrip: $START_NTRIP"
if [[ "$START_NTRIP" == "true" ]]; then
  echo "- ntrip auth:  $NTRIP_AUTHENTICATE"
fi
echo ""
echo "Tip: In another terminal you can verify with: ros2 topic echo ${GPS_FIX_TOPIC} --once"

LAUNCH_ARGS=(
  "gps_device:=${GPS_DEVICE}"
  "gps_baudrate:=${GPS_BAUDRATE}"
  "gps_fix_topic:=${GPS_FIX_TOPIC}"
  "rtcm_topic:=${RTCM_TOPIC}"
  "start_ntrip:=${START_NTRIP}"
)

if [[ "${START_NTRIP}" == "true" ]]; then
  LAUNCH_ARGS+=(
    "ntrip_host:=${NTRIP_HOST}"
    "ntrip_port:=${NTRIP_PORT}"
    "ntrip_mountpoint:=${NTRIP_MOUNTPOINT}"
    "ntrip_version:=${NTRIP_VERSION}"
    "ntrip_authenticate:=${NTRIP_AUTHENTICATE}"
  )

  if [[ "${NTRIP_AUTHENTICATE}" == "true" ]]; then
    LAUNCH_ARGS+=(
      "ntrip_username:=${NTRIP_USERNAME}"
    )

    # Avoid passing empty launch args like `ntrip_password:=` (malformed). The launch file defaults to empty.
    if [[ -n "${NTRIP_PASSWORD}" ]]; then
      LAUNCH_ARGS+=("ntrip_password:=${NTRIP_PASSWORD}")
    fi
  fi
fi

exec ros2 launch tractor_safety_system_launch gps_rtk_ublox_ntrip.launch.py "${LAUNCH_ARGS[@]}"
