#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

usage() {
  cat >&2 <<EOF
Usage:
  ./record_test_bag.sh [TEST_NAME] [--out-dir DIR] [--no-rtk] [--no-tf] [--extra TOPIC]

Examples:
  ./record_test_bag.sh t1
  ./record_test_bag.sh t1 --out-dir /mnt/ssd/bags
  ./record_test_bag.sh t1 --extra /diagnostics

Default topics recorded:
  /fused_detections /camera_detections /radar_detections /tracks /ego_motion /navpvt /gps/fix /diagnostics
Plus (recommended for debugging):
  /tf /tf_static /navrelposned /rtcm /rosout /parameter_events
EOF
}

ensure_ros_domain() {
  local desired_domain="0"
  if [[ "${ROS_DOMAIN_ID:-}" != "${desired_domain}" ]]; then
    export ROS_DOMAIN_ID="${desired_domain}"
  fi
  ros2 daemon stop &>/dev/null || true
  ros2 daemon start &>/dev/null || true
}

TEST_NAME=""
OUT_DIR="${REPO_ROOT}/test_logs/rosbags"
INCLUDE_TF=true
INCLUDE_RTK=true

EXTRA_TOPICS=()

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ -n "${1:-}" && "${1:-}" != --* ]]; then
  TEST_NAME="$1"
  shift
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --out-dir)
      OUT_DIR="${2:-}"
      if [[ -z "${OUT_DIR}" ]]; then
        echo "ERROR: --out-dir requires a value" >&2
        exit 2
      fi
      shift 2
      ;;
    --no-tf)
      INCLUDE_TF=false
      shift
      ;;
    --no-rtk)
      INCLUDE_RTK=false
      shift
      ;;
    --extra)
      if [[ -z "${2:-}" ]]; then
        echo "ERROR: --extra requires a topic name" >&2
        exit 2
      fi
      EXTRA_TOPICS+=("$2")
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: Unknown arg: $1" >&2
      usage
      exit 2
      ;;
  esac
done

if ! command -v ros2 &>/dev/null; then
  echo "ERROR: ros2 not found in PATH (did you source the ROS environment?)" >&2
  exit 1
fi

ensure_ros_domain

mkdir -p "${OUT_DIR}"

TS="$(date +%Y%m%d_%H%M%S)"
if [[ -z "${TEST_NAME}" ]]; then
  TEST_NAME="test"
fi

BAG_NAME="${TEST_NAME}_${TS}"
BAG_PATH="${OUT_DIR}/${BAG_NAME}"

TOPICS=(
  /fused_detections
  /camera_detections
  /radar_detections
  /tracks
  /ego_motion
  /navpvt
  /gps/fix
  /diagnostics
)

if [[ "${INCLUDE_TF}" == true ]]; then
  TOPICS+=(/tf /tf_static)
fi

if [[ "${INCLUDE_RTK}" == true ]]; then
  TOPICS+=(/navrelposned /rtcm)
fi

# Small but useful for debugging
TOPICS+=(/rosout /parameter_events)

if [[ ${#EXTRA_TOPICS[@]} -gt 0 ]]; then
  TOPICS+=("${EXTRA_TOPICS[@]}")
fi

echo "Recording rosbag (MCAP+zstd) -> ${BAG_PATH}"
echo "Topics: ${TOPICS[*]}"
echo "Stop with Ctrl-C."

# Note: '--compression-mode file' compresses each file as it's finalized.
exec ros2 bag record \
  -o "${BAG_PATH}" \
  --storage mcap \
  --compression-mode file \
  --compression-format zstd \
  "${TOPICS[@]}"
