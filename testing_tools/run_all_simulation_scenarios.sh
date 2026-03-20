#!/usr/bin/env bash
set -e

# Source ROS and your workspace
source /opt/ros/jazzy/setup.bash
source ~/playground/install/setup.bash

SCENARIOS=("S1" "S2" "S3" "S4")

# Toggle whether kf_tracker uses range/bearing-based measurement covariance models.
# Default: enabled.
USE_MEASUREMENT_COVARIANCE_MODELS="true"
read -r -p "Use range-dependent measurement covariance models in tracker? [Y/n]: " USE_MEAS_COV_IN
USE_MEAS_COV_IN="${USE_MEAS_COV_IN:-Y}"
case "${USE_MEAS_COV_IN}" in
  Y|y|yes|YES)
    USE_MEASUREMENT_COVARIANCE_MODELS="true"
    ;;
  N|n|no|NO)
    USE_MEASUREMENT_COVARIANCE_MODELS="false"
    ;;
  *)
    echo "Invalid choice, defaulting to: ${USE_MEASUREMENT_COVARIANCE_MODELS}"
    ;;
esac

for S in "${SCENARIOS[@]}"; do
  echo "============================================="
  echo "Running scenario ${S}"
  echo "============================================="

  ros2 launch simulations run_experiment_sim.launch.py \
    scenario:=${S} \
    use_measurement_covariance_models:=${USE_MEASUREMENT_COVARIANCE_MODELS}

  echo "Scenario ${S} finished."
done

echo "All scenarios completed."
