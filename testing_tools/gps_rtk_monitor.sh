#!/bin/bash
# gps_rtk_monitor.sh - monitor GPS/RTK status from ROS2 topics
# Usage:
#   chmod +x gps_rtk_monitor.sh
#   ./gps_rtk_monitor.sh

set -euo pipefail

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

STATUS_TOPIC="/gps/fix"
NAVPVT_TOPIC="/navpvt"
RELPOS_TOPIC="/navrelposned"
SLEEP_SEC=1

print_header() {
  echo "========================================"
  echo "GPS / RTK Monitor"
  echo "Uses NavRELPOSNED for RTK state when available"
  echo "Press Ctrl+C to exit"
  echo "========================================"
}

get_status() {
  timeout 2 ros2 topic echo "$STATUS_TOPIC" --once 2>/dev/null \
    | awk -F': ' '/^  status:/{print $2}' | head -n1 | tr -d ' '
}

get_navpvt_line() {
  timeout 2 ros2 topic echo "$NAVPVT_TOPIC" --once 2>/dev/null \
    | awk -F': ' '/^(fix_type|num_sv|p_dop|h_acc|v_acc):/{print}'
}

get_navpvt_fields() {
  local out
  out="$(timeout 2 ros2 topic echo "$NAVPVT_TOPIC" --once 2>/dev/null || true)"
  [[ -n "$out" ]] || return 1

  local fix_type num_sv p_dop h_acc v_acc
  fix_type="$(printf '%s\n' "$out" | awk -F': ' '/^fix_type:/{print $2}' | head -n1)"
  num_sv="$(printf '%s\n' "$out" | awk -F': ' '/^num_sv:/{print $2}' | head -n1)"
  p_dop="$(printf '%s\n' "$out" | awk -F': ' '/^p_dop:/{print $2}' | head -n1)"
  h_acc="$(printf '%s\n' "$out" | awk -F': ' '/^h_acc:/{print $2}' | head -n1)"
  v_acc="$(printf '%s\n' "$out" | awk -F': ' '/^v_acc:/{print $2}' | head -n1)"

  [[ -n "$fix_type" && -n "$num_sv" && -n "$p_dop" && -n "$h_acc" && -n "$v_acc" ]] || return 1
  printf '%s %s %s %s %s\n' "$fix_type" "$num_sv" "$p_dop" "$h_acc" "$v_acc"
}

get_relpos_fields() {
  local out
  out="$(timeout 2 ros2 topic echo "$RELPOS_TOPIC" --once 2>/dev/null || true)"
  [[ -n "$out" ]] || return 1

  local flags ref_station_id
  flags="$(printf '%s\n' "$out" | awk -F': ' '/^flags:/{print $2}' | head -n1)"
  ref_station_id="$(printf '%s\n' "$out" | awk -F': ' '/^ref_station_id:/{print $2}' | head -n1)"
  [[ -n "$flags" && -n "$ref_station_id" ]] || return 1
  printf '%s %s\n' "$flags" "$ref_station_id"
}

decode_relpos_state() {
  local flags="$1"
  # Common u-blox RELPOSNED layout: carrSoln is bits 3..4 (mask 0x18)
  # 0 = NONE, 1 = FLOAT, 2 = FIXED, 3 = reserved/invalid
  local carr_soln=$(( (flags & 0x18) >> 3 ))
  case "$carr_soln" in
    2) echo "RTK FIXED (carrSoln=2)" ;;
    1) echo "RTK FLOAT (carrSoln=1)" ;;
    0) echo "NO RTK (carrSoln=0)" ;;
    *) echo "INVALID/RESERVED (carrSoln=$carr_soln)" ;;
  esac
}

format_navsat_status() {
  local status="$1"
  # sensor_msgs/NavSatStatus.status meanings:
  #  -1 = STATUS_NO_FIX
  #   0 = STATUS_FIX
  #   1 = STATUS_SBAS_FIX
  #   2 = STATUS_GBAS_FIX
  # Some drivers may publish other values; do not treat as RTK.
  case "$status" in
    -1) echo "NO FIX (NavSatStatus=-1)" ;;
    0)  echo "FIX (NavSatStatus=0)" ;;
    1)  echo "SBAS FIX (NavSatStatus=1)" ;;
    2)  echo "GBAS/DGNSS (NavSatStatus=2)" ;;
    *)  echo "NavSatStatus=$status (driver-specific)" ;;
  esac
}

mm_to_m() {
  # Convert integer mm to meters with 3 decimals
  local mm="$1"
  awk -v h="$mm" 'BEGIN{printf "%.3f", h/1000.0}'
}

print_header

while true; do
  now="$(date '+%H:%M:%S')"

  status="$(get_status || true)"
  navpvt_fields="$(get_navpvt_fields || true)"
  relpos_fields="$(get_relpos_fields || true)"

  # Defaults
  nav_fix_type=""
  num_sv=""
  p_dop=""
  h_acc_m=""
  v_acc_m=""

  if [[ -n "$navpvt_fields" ]]; then
    read -r nav_fix_type num_sv p_dop h_acc_raw v_acc_raw <<< "$navpvt_fields"
    # ublox_msgs/NavPVT h_acc and v_acc are in millimeters -> convert to meters
    h_acc_m="$(mm_to_m "$h_acc_raw")"
    v_acc_m="$(mm_to_m "$v_acc_raw")"
  fi

  relpos_state=""
  relpos_flags=""
  ref_station_id=""
  if [[ -n "$relpos_fields" ]]; then
    read -r relpos_flags ref_station_id <<< "$relpos_fields"
    relpos_state="$(decode_relpos_state "$relpos_flags")"
  fi

  acc_suffix=""
  if [[ -n "$h_acc_m" ]]; then
    acc_suffix=" | h_acc≈${h_acc_m} m"
  fi
  if [[ -n "$v_acc_m" ]]; then
    acc_suffix+=" | v_acc≈${v_acc_m} m"
  fi
  if [[ -n "$num_sv" ]]; then
    acc_suffix+=" | sv=${num_sv}"
  fi
  if [[ -n "$p_dop" ]]; then
    acc_suffix+=" | p_dop=${p_dop}"
  fi

  # Primary line: RTK state from NavRELPOSNED if available
  if [[ -n "$relpos_state" ]]; then
    case "$relpos_state" in
      RTK\ FIXED*)
        echo -e "${GREEN}[$now] ${relpos_state}${acc_suffix}${NC}"
        ;;
      RTK\ FLOAT*)
        echo -e "${GREEN}[$now] ${relpos_state}${acc_suffix}${NC}"
        ;;
      *)
        echo -e "${YELLOW}[$now] ${relpos_state}${acc_suffix}${NC}"
        ;;
    esac
    echo -e "${CYAN}  ref_station_id: ${ref_station_id} | flags: ${relpos_flags} (NavRELPOSNED)${NC}"
  else
    if [[ -z "${status:-}" ]]; then
      echo -e "${RED}[$now] No NavSatFix data (${STATUS_TOPIC})${NC}"
    else
      echo -e "${CYAN}[$now] $(format_navsat_status "$status")${acc_suffix}${NC}"
    fi
  fi

  # Secondary line: show NavPVT fix_type and NavSatFix status
  if [[ -n "$nav_fix_type" ]]; then
    echo -e "${CYAN}  NavPVT fix_type: ${nav_fix_type} | NavSatFix status: ${status:-n/a}${NC}"
  elif [[ -n "${status:-}" ]]; then
    echo -e "${CYAN}  NavSatFix status: ${status}${NC}"
  fi

  navpvt_line="$(get_navpvt_line || true)"
  if [[ -n "$navpvt_line" ]]; then
    echo -e "${CYAN}${navpvt_line}${NC}" | sed 's/^/  /'
  fi

  sleep "$SLEEP_SEC"
  echo ""
done