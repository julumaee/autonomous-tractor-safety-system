#!/usr/bin/env bash
set -e

# Source ROS and your workspace
source /opt/ros/jazzy/setup.bash
source ~/playground/install/setup.bash

SCENARIOS=("S1" "S2" "S3" "S4")

for S in "${SCENARIOS[@]}"; do
  echo "============================================="
  echo "Running scenario ${S}"
  echo "============================================="

  ros2 launch simulations run_experiment_sim.launch.py scenario:=${S}

  echo "Scenario ${S} finished."
done

echo "All scenarios completed."
