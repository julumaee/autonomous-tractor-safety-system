#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

usage() {
  cat <<'EOF'
Replay per-run rosbags one at a time, pausing for user confirmation between runs.

This script does NOT start or stop the logger. Run your logger manually in another
terminal with the desired test_name for each round (for example `t1_1_replay`).

Usage:
  ./replay_bags_stepwise.sh [options]

Options:
  --bags-dir DIR         Bag directory (default: <repo>/test_logs/rosbags)
  --glob GLOB            Bag name glob (default: t[1-5]_[1-7]_*)
  --topics "..."         Topics to play
                         (default: /camera_detections /radar_detections /ego_motion /navpvt /gps/fix /navrelposned)
  --name-suffix SUFFIX   Suggested suffix for manual logger test_name (default: _replay)
  --start-from NAME      Start from bag basename prefix, e.g. t2_3
  --dry-run              Print matched bags only
  -h, --help             Show help

Behavior:
  - Lists bag runs in sorted order
  - Before each bag, prints the suggested logger test_name
  - Waits for Enter before playing that bag
  - After completion, waits for Enter before moving to the next bag
  - Type 'q' at a prompt to quit
EOF
}

BAGS_DIR="${REPO_ROOT}/test_logs/rosbags"
BAG_GLOB='t[1-5]_[1-7]_*'
TOPICS='/camera_detections /radar_detections /ego_motion /navpvt /gps/fix /navrelposned'
NAME_SUFFIX='_replay'
START_FROM=''
DRY_RUN=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bags-dir)
      BAGS_DIR="${2:-}"
      shift 2
      ;;
    --glob)
      BAG_GLOB="${2:-}"
      shift 2
      ;;
    --topics)
      TOPICS="${2:-}"
      shift 2
      ;;
    --name-suffix)
      NAME_SUFFIX="${2:-}"
      shift 2
      ;;
    --start-from)
      START_FROM="${2:-}"
      shift 2
      ;;
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 not found in PATH. Source your ROS workspace first." >&2
  exit 1
fi

if [[ ! -d "${BAGS_DIR}" ]]; then
  echo "ERROR: bags dir not found: ${BAGS_DIR}" >&2
  exit 1
fi

extract_test_name() {
  local bag_basename="$1"
  if [[ "${bag_basename}" =~ ^(t[0-9]+_[0-9]+)_ ]]; then
    echo "${BASH_REMATCH[1]}"
  else
    echo "${bag_basename}"
  fi
}

confirm_continue() {
  local prompt="$1"
  local reply
  read -r -p "${prompt}" reply
  if [[ "${reply}" == "q" || "${reply}" == "Q" ]]; then
    echo "Stopping at user request."
    exit 0
  fi
}

mapfile -t BAG_DIRS < <(find "${BAGS_DIR}" -maxdepth 1 -mindepth 1 -type d -name "${BAG_GLOB}" | sort)

if [[ ${#BAG_DIRS[@]} -eq 0 ]]; then
  echo "ERROR: no bag directories matched '${BAG_GLOB}' under ${BAGS_DIR}" >&2
  exit 1
fi

if [[ -n "${START_FROM}" ]]; then
  FILTERED=()
  started=false
  for bag in "${BAG_DIRS[@]}"; do
    bag_name="$(basename -- "${bag}")"
    test_name="$(extract_test_name "${bag_name}")"
    if [[ "${bag_name}" == ${START_FROM}* || "${test_name}" == "${START_FROM}" ]]; then
      started=true
    fi
    if [[ "${started}" == true ]]; then
      FILTERED+=("${bag}")
    fi
  done
  BAG_DIRS=("${FILTERED[@]}")
fi

if [[ ${#BAG_DIRS[@]} -eq 0 ]]; then
  echo "ERROR: start point '${START_FROM}' did not match any bag." >&2
  exit 1
fi

if [[ "${DRY_RUN}" == true ]]; then
  echo "[DRY-RUN] Would process ${#BAG_DIRS[@]} bags from ${BAGS_DIR}"
  for bag in "${BAG_DIRS[@]}"; do
    b="$(basename -- "${bag}")"
    tn="$(extract_test_name "${b}")"
    echo "  - ${b} -> logger test_name suggestion: ${tn}${NAME_SUFFIX}"
  done
  exit 0
fi

echo "Found ${#BAG_DIRS[@]} bag runs."
echo "Bags dir: ${BAGS_DIR}"
echo "Topics: ${TOPICS}"
echo "Suggested logger suffix: ${NAME_SUFFIX}"
echo

total=${#BAG_DIRS[@]}
index=0
for bag in "${BAG_DIRS[@]}"; do
  index=$((index + 1))
  bag_name="$(basename -- "${bag}")"
  test_name="$(extract_test_name "${bag_name}")"
  suggested_name="${test_name}${NAME_SUFFIX}"

  if [[ ! -f "${bag}/metadata.yaml" ]]; then
    echo "[WARN] Skipping ${bag_name}: metadata.yaml not found"
    continue
  fi

  echo "============================================================"
  echo "Run ${index}/${total}: ${bag_name}"
  echo "Suggested logger test_name: ${suggested_name}"
  echo "Make sure your external logger is running and ready."
  confirm_continue "Press Enter to play this bag, or 'q' to quit: "

  echo "Playing bag..."
  ros2 bag play "${bag}" --clock --topics ${TOPICS}

  echo
  echo "Completed ${bag_name}"
  if (( index < total )); then
    confirm_continue "Press Enter for the next bag, or 'q' to quit: "
  fi
  echo

done

echo "All matched bag runs processed."
