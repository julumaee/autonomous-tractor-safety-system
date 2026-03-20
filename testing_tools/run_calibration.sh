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
DEFAULT_RADAR_FIT_MODE="planar"
GROUND_TRUTH_FILE="calibration_log/ground_truth.csv"
COMBINED_FILE="calibration_log/calibration_raw_combined.csv"
TF_FILE="calibration_log/calibration_tf.txt"
GPS_ORIGIN_FILE="calibration_log/gps_origin.json"
GPS_LOG_FILE="calibration_log/navpvt_log.csv"
GPS_TOPIC_DEFAULT="/navpvt"

normalize_tf_file() {
    local tf_file="$1"
    if [[ ! -f "${tf_file}" ]]; then
        return 0
    fi

    # Keep only legacy keys (used by launch_test.sh):
    #   radar_tf:  x y z roll pitch yaw
    #   camera_tf: x y z roll pitch yaw
    local header radar_line camera_line
    header="$(grep -E '^[[:space:]]*#' "${tf_file}" || true)"
    radar_line="$(grep -E '^[[:space:]]*radar_tf:[[:space:]]' "${tf_file}" | head -n1 || true)"
    camera_line="$(grep -E '^[[:space:]]*camera_tf:[[:space:]]' "${tf_file}" | head -n1 || true)"

    if [[ -z "${radar_line}" || -z "${camera_line}" ]]; then
        return 0
    fi

    {
        if [[ -n "${header}" ]]; then
            printf "%s\n" "${header}"
            printf "\n"
        fi
        printf "%s\n" "${radar_line}"
        printf "%s\n" "${camera_line}"
    } > "${tf_file}.tmp"

    mv "${tf_file}.tmp" "${tf_file}"
}

find_latest_data_file() {
    if [[ -f "${COMBINED_FILE}" ]]; then
        echo "${COMBINED_FILE}"
        return 0
    fi
    ls -t calibration_log/calibration_raw_*.csv 2>/dev/null | head -n 1 || true
}

visualize_file() {
    local data_file="$1"

    if [[ ! -f "${GROUND_TRUTH_FILE}" ]]; then
        echo "ERROR: Ground truth file ${GROUND_TRUTH_FILE} not found." >&2
        return 1
    fi

    echo ""
    echo "Visualizing: ${data_file}"
    local show_args=()
    if [[ -z "${DISPLAY:-}" ]]; then
        show_args+=(--no-show)
    fi
    python3 visualize_calibration_2d.py \
        --data "${data_file}" \
        --ground-truth "${GROUND_TRUTH_FILE}" \
        --tf-file "${TF_FILE}" \
        "${show_args[@]}"
}

echo "=========================================="
echo "Sensor Calibration Quick Start"
echo "=========================================="
echo ""
echo "Calibration Steps:"
echo "  1. Sequential calibration (one target at a time)"
echo "  2. Analyze existing data"
echo "  3. Visualize existing data (2D top-down)"
echo "  4. Exit"
echo ""

read -r -p "Select option [1-4]: " choice

case "$choice" in
    1)
        if [[ ! -f "${GROUND_TRUTH_FILE}" ]]; then
            echo "ERROR: ${GROUND_TRUTH_FILE} not found." >&2
            echo "Create it first by editing: ${GROUND_TRUTH_FILE}" >&2
            exit 1
        fi

        write_gps_origin_this_session=true

        if [[ -f "${COMBINED_FILE}" ]]; then
            read -r -p "Existing ${COMBINED_FILE} found. Start fresh (delete)? [y/N]: " fresh_start
            if [[ "${fresh_start}" == "y" || "${fresh_start}" == "Y" ]]; then
                rm -f "${COMBINED_FILE}"
                echo "Deleted old combined data."
                rm -f "${GPS_ORIGIN_FILE}" "${GPS_LOG_FILE}" || true
            else
                echo "Will append to existing combined data."

                if [[ -f "${GPS_ORIGIN_FILE}" ]]; then
                    read -r -p "Existing ${GPS_ORIGIN_FILE} found. Overwrite GPS origin this session? [Y/n]: " overwrite_gps_origin
                    if [[ "${overwrite_gps_origin}" == "n" || "${overwrite_gps_origin}" == "N" ]]; then
                        write_gps_origin_this_session=false
                        echo "Will keep existing GPS origin file unchanged."
                    fi
                fi
            fi
        fi

        echo ""
        echo "GPS logging:"
        echo "  topic: ${GPS_TOPIC_DEFAULT}"
        echo "  origin file: ${GPS_ORIGIN_FILE} (written once when logging starts)"
        echo "  log file: ${GPS_LOG_FILE} (appended while collection runs)"

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

            # Use --append for all targets to build combined file sequentially
            cmd=(python3 sensor_calibration.py \
                --mode collect \
                --duration "$per_target_duration" \
                --start-delay "$start_delay" \
                --target-name "$target_name" \
                --append \
                --gps-topic "${GPS_TOPIC_DEFAULT}" \
                --gps-origin-out "${GPS_ORIGIN_FILE}" \
                --gps-log-out "${GPS_LOG_FILE}")

            # Avoid overwriting origin for each target: write it once per session.
            if [[ "$write_gps_origin_this_session" != "true" || "$i" -gt 1 ]]; then
                cmd+=(--disable-gps-origin)
            fi

            "${cmd[@]}"

            if [[ "$i" -lt "$num_targets" ]]; then
                echo ""
                read -r -p "Move to the next target position, then press Enter..." _
            fi
        done

        echo ""
        echo "All targets collected. Combined file: ${COMBINED_FILE}"
        read -r -p "Analyze now? [Y/n]: " analyze_now
        if [[ "${analyze_now}" != "n" && "${analyze_now}" != "N" ]]; then
            echo ""
            echo "Radar fit mode:"
            echo "  6dof  = fit x,y,z,roll,pitch,yaw (default, more flexible)"
            echo "  planar = fit only x,y,yaw (often more stable; avoids yaw overfit)"
            read -r -p "Radar fit mode [${DEFAULT_RADAR_FIT_MODE}]: " radar_fit_mode
            radar_fit_mode=${radar_fit_mode:-$DEFAULT_RADAR_FIT_MODE}
            if [[ "${radar_fit_mode}" != "6dof" && "${radar_fit_mode}" != "planar" ]]; then
                echo "ERROR: Invalid radar fit mode '${radar_fit_mode}' (use 6dof or planar)." >&2
                exit 2
            fi

            read -r -p "Exclude camera target(s) from analysis (comma-separated, blank for none): " exclude_cam_targets

            cmd=(python3 sensor_calibration.py \
                --mode analyze \
                --data "${COMBINED_FILE}" \
                --ground-truth "${GROUND_TRUTH_FILE}" \
                --radar-fit-mode "${radar_fit_mode}")

            if [[ -n "${exclude_cam_targets}" ]]; then
                IFS=',' read -r -a _cam_excl_arr <<< "${exclude_cam_targets}"
                for t in "${_cam_excl_arr[@]}"; do
                    t_trimmed="$(echo "${t}" | xargs)"
                    if [[ -n "${t_trimmed}" ]]; then
                        cmd+=(--exclude-camera-target "${t_trimmed}")
                    fi
                done
            fi

            cmd+=(--save-tf --tf-out "${TF_FILE}")
            "${cmd[@]}"

            normalize_tf_file "${TF_FILE}"

            if [[ -f "${TF_FILE}" ]]; then
                echo ""
                echo "[INFO] TF file updated: ${SCRIPT_DIR}/${TF_FILE}"
                grep -E '^# generated:' "${TF_FILE}" || true
            else
                echo ""
                echo "[WARN] TF file was not created: ${SCRIPT_DIR}/${TF_FILE}" >&2
            fi
        fi
        ;;

    2)
        latest_file="$(find_latest_data_file)"
        if [[ -z "${latest_file}" ]]; then
            echo "ERROR: No calibration data files found in calibration_log/." >&2
            exit 1
        fi

        echo "Analyzing: ${latest_file}"
        echo ""
        echo "Optional: Provide initial sensor positions to improve calibration."
        echo "(Leave blank to use solver defaults)"
        echo ""
        
        read -r -p "Radar initial position [x,y,z] or [x,y,z,roll,pitch,yaw]: " radar_init
        read -r -p "Camera initial position [x,y,z] or [x,y,z,roll,pitch,yaw]: " camera_init

        echo ""
        echo "Radar fit mode:"
        echo "  6dof  = fit x,y,z,roll,pitch,yaw (default, more flexible)"
        echo "  planar = fit only x,y,yaw (often more stable; avoids yaw overfit)"
        read -r -p "Radar fit mode [${DEFAULT_RADAR_FIT_MODE}]: " radar_fit_mode
        radar_fit_mode=${radar_fit_mode:-$DEFAULT_RADAR_FIT_MODE}
        if [[ "${radar_fit_mode}" != "6dof" && "${radar_fit_mode}" != "planar" ]]; then
            echo "ERROR: Invalid radar fit mode '${radar_fit_mode}' (use 6dof or planar)." >&2
            exit 2
        fi

        # Build command array to avoid quoting issues
        cmd=(python3 sensor_calibration.py --mode analyze --data "${latest_file}")
        if [[ -f "${GROUND_TRUTH_FILE}" ]]; then
            cmd+=(--ground-truth "${GROUND_TRUTH_FILE}")
        fi
        cmd+=(--radar-fit-mode "${radar_fit_mode}")

        read -r -p "Exclude camera target(s) from analysis (comma-separated, blank for none): " exclude_cam_targets
        if [[ -n "${exclude_cam_targets}" ]]; then
            IFS=',' read -r -a _cam_excl_arr <<< "${exclude_cam_targets}"
            for t in "${_cam_excl_arr[@]}"; do
                t_trimmed="$(echo "${t}" | xargs)"
                if [[ -n "${t_trimmed}" ]]; then
                    cmd+=(--exclude-camera-target "${t_trimmed}")
                fi
            done
        fi
        if [[ -n "${radar_init}" ]]; then
            cmd+=(--radar-init="${radar_init}")
        fi
        if [[ -n "${camera_init}" ]]; then
            cmd+=(--camera-init="${camera_init}")
        fi
        cmd+=(--save-tf --tf-out "${TF_FILE}")
        "${cmd[@]}"

        normalize_tf_file "${TF_FILE}"

        if [[ -f "${TF_FILE}" ]]; then
            echo ""
            echo "[INFO] TF file updated: ${SCRIPT_DIR}/${TF_FILE}"
            grep -E '^# generated:' "${TF_FILE}" || true
        else
            echo ""
            echo "[WARN] TF file was not created: ${SCRIPT_DIR}/${TF_FILE}" >&2
        fi

        # Auto-visualize right after analysis
        visualize_file "${latest_file}"
        ;;

    3)
        latest_file="$(find_latest_data_file)"
        if [[ -z "${latest_file}" ]]; then
            echo "ERROR: No calibration data files found in calibration_log/." >&2
            exit 1
        fi

        visualize_file "${latest_file}"
        ;;

    4)
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
