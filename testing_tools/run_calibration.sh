#!/usr/bin/env bash
# Interactive wrapper for sensor_calibration.py

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source_setup_file() {
    local setup_file="$1"
    if [[ -f "$setup_file" ]]; then
        set +u
        # shellcheck disable=SC1090
        source "$setup_file"
        set -u
    fi
}

WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

if ! command -v ros2 &>/dev/null; then
    if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
        source_setup_file "/opt/ros/${ROS_DISTRO}/setup.bash"
    elif [[ -f "/opt/ros/jazzy/setup.bash" ]]; then
        source_setup_file "/opt/ros/jazzy/setup.bash"
    fi
fi

if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
    source_setup_file "${WORKSPACE_ROOT}/install/setup.bash"
fi

if [[ ! -f "sensor_calibration.py" ]]; then
    echo "ERROR: sensor_calibration.py not found in ${SCRIPT_DIR}" >&2
    exit 1
fi

mkdir -p calibration_log

DEFAULT_DURATION=60
DEFAULT_START_DELAY=0
GROUND_TRUTH_FILE="calibration_log/ground_truth.csv"
GROUND_TRUTH_TEMPLATE_FILE="calibration_log/ground_truth_template.csv"
COMBINED_FILE="calibration_log/calibration_raw_combined.csv"

echo "=========================================="
echo "Sensor Calibration Quick Start"
echo "=========================================="
echo ""
echo "Calibration Steps:"
echo "  1. Create ground truth template"
echo "  2. Sequential calibration (one target at a time)"
echo "  3. Collect calibration data (all targets at once)"
echo "  4. Analyze existing data"
echo "  5. Full calibration (collect + analyze)"
echo "  6. Exit"
echo ""

read -r -p "Select option [1-6]: " choice

case "$choice" in
    1)
        echo "Creating ground truth template..."
        python3 sensor_calibration.py --mode template
        echo ""
        echo "Edit: ${GROUND_TRUTH_TEMPLATE_FILE}"
        echo "Then copy it to ${GROUND_TRUTH_FILE}:"
        echo "  cp ${GROUND_TRUTH_TEMPLATE_FILE} ${GROUND_TRUTH_FILE}"
        ;;

    2)
        if [[ ! -f "${GROUND_TRUTH_FILE}" ]]; then
            echo "ERROR: ${GROUND_TRUTH_FILE} not found." >&2
            echo "Create it first (option 1) and fill in your target positions." >&2
            exit 1
        fi

        if [[ -f "${COMBINED_FILE}" ]]; then
            read -r -p "Existing ${COMBINED_FILE} found. Start fresh (delete)? [y/N]: " fresh_start
            if [[ "${fresh_start}" == "y" || "${fresh_start}" == "Y" ]]; then
                rm -f "${COMBINED_FILE}"
                echo "Deleted old combined data."
            else
                echo "Will append to existing combined data."
            fi
        fi

        read -r -p "Number of targets to collect (recommend 3-6): " num_targets
        if [[ -z "${num_targets}" || ! "${num_targets}" =~ ^[0-9]+$ || "${num_targets}" -lt 1 ]]; then
            echo "ERROR: Invalid number of targets." >&2
            exit 2
        fi

        read -r -p "Duration per target in seconds [${DEFAULT_DURATION}]: " per_target_duration
        per_target_duration=${per_target_duration:-$DEFAULT_DURATION}

        read -r -p "Start delay before logging each target (seconds) [${DEFAULT_START_DELAY}]: " start_delay
        start_delay=${start_delay:-$DEFAULT_START_DELAY}

        for i in $(seq 1 "$num_targets"); do
            echo ""
            echo "=========================================="
            echo "TARGET ${i} of ${num_targets}"
            echo "=========================================="
            read -r -p "Enter target name/description (e.g., pos_5m): " target_name
            if [[ -z "${target_name}" ]]; then
                echo "ERROR: target name cannot be empty" >&2
                exit 2
            fi

            if [[ "$i" -eq 1 && ! -f "${COMBINED_FILE}" ]]; then
                python3 sensor_calibration.py --mode collect --duration "$per_target_duration" --start-delay "$start_delay" --target-name "$target_name"
            else
                python3 sensor_calibration.py --mode collect --duration "$per_target_duration" --start-delay "$start_delay" --target-name "$target_name" --append
            fi

            if [[ "$i" -lt "$num_targets" ]]; then
                echo ""
                read -r -p "Move to the next target position, then press Enter..." _
            fi
        done

        echo ""
        echo "All targets collected. Combined file: ${COMBINED_FILE}"
        read -r -p "Analyze now? [Y/n]: " analyze_now
        if [[ "${analyze_now}" != "n" && "${analyze_now}" != "N" ]]; then
            python3 sensor_calibration.py --mode analyze --data "${COMBINED_FILE}" --ground-truth "${GROUND_TRUTH_FILE}"
        fi
        ;;

    3)
        read -r -p "Collection duration seconds [${DEFAULT_DURATION}]: " duration
        duration=${duration:-$DEFAULT_DURATION}

        read -r -p "Start delay before logging (seconds) [${DEFAULT_START_DELAY}]: " start_delay
        start_delay=${start_delay:-$DEFAULT_START_DELAY}

        python3 sensor_calibration.py --mode collect --duration "$duration" --start-delay "$start_delay"
        ;;

    4)
        latest_file=""
        if [[ -f "${COMBINED_FILE}" ]]; then
            latest_file="${COMBINED_FILE}"
        else
            latest_file=$(ls -t calibration_log/calibration_raw_*.csv 2>/dev/null | head -n 1 || true)
        fi

        if [[ -z "${latest_file}" ]]; then
            echo "ERROR: No calibration data files found in calibration_log/." >&2
            exit 1
        fi

        echo "Analyzing: ${latest_file}"
        if [[ -f "${GROUND_TRUTH_FILE}" ]]; then
            python3 sensor_calibration.py --mode analyze --data "${latest_file}" --ground-truth "${GROUND_TRUTH_FILE}"
        else
            python3 sensor_calibration.py --mode analyze --data "${latest_file}"
        fi
        ;;

    5)
        if [[ ! -f "${GROUND_TRUTH_FILE}" ]]; then
            echo "ERROR: ${GROUND_TRUTH_FILE} not found." >&2
            echo "Create it first (option 1) and fill in your target positions." >&2
            exit 1
        fi
        read -r -p "Full calibration duration seconds [${DEFAULT_DURATION}]: " duration
        duration=${duration:-$DEFAULT_DURATION}

        read -r -p "Start delay before logging (seconds) [${DEFAULT_START_DELAY}]: " start_delay
        start_delay=${start_delay:-$DEFAULT_START_DELAY}

        python3 sensor_calibration.py --mode full --duration "$duration" --start-delay "$start_delay" --ground-truth "${GROUND_TRUTH_FILE}"
        ;;

    6)
        echo "Exiting..."
        exit 0
        ;;

    *)
        echo "Invalid option." >&2
        exit 2
        ;;
esac

echo ""
echo "Done."
