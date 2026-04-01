# Real-World Tractor Testing - Complete Guide

**Last Updated:** February 13, 2026  \
**System:** Autonomous Tractor Safety System (Radar + Camera)

**Note:** GPS/GNSS (including RTK) is **testing-only** in this repo: it can be used as a substitute ego-motion source when tractor ECU odometry/IMU is unavailable. It is not a design requirement for the safety pipeline.

This guide provides a script-first workflow for:
- Calibrating sensor transforms (radar + camera)
- Running real-world test scenarios (standing pedestrian, moving tractor)
- Logging and basic analysis

All commands assume ROS 2 Jazzy.

---

## Table of Contents

1. [Quick Start](#quick-start)
   - [A) Calibration (scripts)](#a-calibration-scripts)
   - [B) Real-world test (scripts)](#b-real-world-test-scripts)
2. [Hardware Requirements](#hardware-requirements)
3. [Pre-Test Preparation](#pre-test-preparation)
4. [Test Execution Instructions](#test-execution-instructions)
   - [Test cases](#test-cases)
   - [Script-based workflow (recommended)](#script-based-workflow-recommended)
   - [Manual workflow (optional)](#manual-workflow-optional)
5. [Post-Test Analysis](#post-test-analysis)
6. [Command Reference](#command-reference)
7. [Safety Reminders](#safety-reminders)
8. [Additional Resources](#additional-resources)

---

## Quick Start

### A) Calibration (scripts)

Goal: Estimate sensor TF parameters (`radar_tf_*`, `camera_tf_*`) so detections align.

**Terminal 1 (bring up sensors only):**

```bash
cd ~/autonomous-tractor-safety-system
source install/setup.bash

# Start radar + camera; tracker disabled for calibration
ros2 launch tractor_safety_system_launch perception_stack.launch.py \
  start_tracker:=false
```

**Terminal 2 (calibration workflow):**

```bash
cd ~/autonomous-tractor-safety-system/testing_tools
./run_calibration.sh
```

Use the full calibration guide here:
- [CALIBRATION.md](CALIBRATION.md)

Important:
- Sequential calibration uses `target_name` matching; your `--target-name` values must match the `target_name` column in `calibration_log/ground_truth.csv`.
- If you need time to walk into place before logging, use `--start-delay 10` (or set it in the interactive `run_calibration.sh` prompts).
- When calibration prints final TF values (including roll/pitch/yaw), use those same numbers in the real-world test bring-up.

---

### B) Real-world test (scripts)

Goal: Bring up GPS (+ optional NTRIP), perception stack, ego motion, and logging.

**Terminal A (GPS + optional NTRIP, interactive):**

```bash
cd ~/autonomous-tractor-safety-system/testing_tools
./gps_rtk_start.sh

# For GPS without NTRIP corrections:
# ./gps_rtk_start.sh --no-ntrip
```

**Terminal B (pre-test checks, then start test):**

```bash
cd ~/autonomous-tractor-safety-system
source install/setup.bash

cd testing_tools
./validate_system.sh
```

Start the test stack.

Option 1 (positions only; quick start):

```bash
./launch_test.sh field_test_S1 /gps/fix \
  1.2 0.0 0.8 \
  1.5 0.0 0.5
```

Option 2 (recommended after calibration; full TF):

```bash
./launch_test.sh field_test_S1 /gps/fix \
  <camera_x> <camera_y> <camera_z> <camera_roll> <camera_pitch> <camera_yaw> \
  <radar_x>  <radar_y>  <radar_z>  <radar_roll>  <radar_pitch>  <radar_yaw>
```

**Terminal C (live monitoring dashboard):**

```bash
cd ~/autonomous-tractor-safety-system/testing_tools
./monitor_test.sh
```

Logs are written under `testing_tools/real_world_logs/`.

---

## Hardware Requirements

At minimum:
- Radar sensor + CAN interface (CAN @ 1 Mbps)
- Depth camera + USB connection
- RTK-capable GNSS receiver (u-blox ZED-F9P / ArduSimple RTK2B recommended)
- Power supplies for sensors

---

## Pre-Test Preparation

Do these before excecuting the test scenarios.

### 1) Build and source

```bash
cd ~/autonomous-tractor-safety-system
colcon build
source install/setup.bash
```

### 2) Calibration

- Mark calibration target positions relative to your chosen tractor origin (`base_link`).
- Create and fill in ground truth:
  - `testing_tools/calibration_log/ground_truth.csv`
- Run calibration per [CALIBRATION.md](CALIBRATION.md).

### 3) Mark the test area

Before running S1–S3, mark:
- A straight tractor path / centerline
- Start line(s) at 30 m (and optionally 20 m, 10 m)
- Your working-width boundaries

Working width $W$ (tractor + implement) boundaries in the tractor body frame convention:
- Left boundary: $Y = +\frac{W}{2}$
- Right boundary: $Y = -\frac{W}{2}$

### 4) Permissions + dependencies

# Real-World Testing Guide (Current)

This guide reflects the current scripts and launch files in this repository.

## 1) One-time setup

```bash
cd ~/autonomous-tractor-safety-system
colcon build
source install/setup.bash
```

Install required system tools if missing:
```bash
sudo apt install -y can-utils ros-jazzy-ntrip-client
```

## 2) Calibration (recommended before field tests)

Use the calibration workflow in [CALIBRATION.md](CALIBRATION.md). The output file used by test launch scripts is:

- `testing_tools/calibration_log/calibration_tf.txt`

## 3) Live real-world test workflow

### Terminal A: GPS + optional NTRIP

```bash
cd ~/autonomous-tractor-safety-system/testing_tools
./gps_rtk_start.sh
# or: ./gps_rtk_start.sh --no-ntrip
```

### Terminal B: CAN + validation

```bash
cd ~/autonomous-tractor-safety-system/testing_tools
./setup_can.sh
./validate_system.sh
```

### Terminal C: Start test stack (interactive)

```bash
cd ~/autonomous-tractor-safety-system/testing_tools
./launch_test.sh
```

`launch_test.sh` now prompts for:
- test name
- GPS topic (default `/navpvt`)
- start camera driver or not
- tracker covariance model toggle
- TF source (calibration file vs defaults)

The launched stack includes real-world logging automatically (`real_world_logger`), producing files under:
- `testing_tools/real_world_logs/`

## 4) Replay workflow for bag batches

Recommended current replay method is stepwise/manual logger control:

### Terminal A: perception stack for replay

```bash
cd ~/autonomous-tractor-safety-system
source install/setup.bash

ros2 launch tractor_safety_system_launch perception_stack.launch.py \
  start_radar:=false start_camera:=false publish_tf:=true \
  tf_file:=testing_tools/calibration_log/replay_tf.txt
```

### Terminal B: play bags one-by-one

```bash
cd ~/autonomous-tractor-safety-system/testing_tools
./replay_bags_stepwise.sh
```

For each bag, start logger manually in another terminal using suggested test name (for example `t2_3_replay`):

```bash
ros2 launch simulations real_world_logger.launch.py \
  test_name:=t2_3_replay use_sim_time:=true
```

Stop logger after each run, then continue to next bag.

## 5) Analysis

### Distance-only real-world analysis (current main workflow)

```bash
cd ~/autonomous-tractor-safety-system/testing_tools
./analysis_tools/analyze_real_world_distance t1_2_replay
```

For consistent axis across scenarios, use a `t1` GPS file as axis reference:

```bash
./analysis_tools/analyze_real_world_distance t3_4_replay \
  --axis-gps-file real_world_logs/gps_truth_t1_2_replay.csv
```

### Legacy 2D analysis (kept)

```bash
./analysis_tools/analyze_real_world t1_2_replay
```

## 6) Common troubleshooting

### No nodes visible across terminals

```bash
ros2 daemon stop
ros2 daemon start
ros2 node list
```

Also ensure every terminal uses the same ROS environment (`install/setup.bash`, domain, RMW settings).

### Validate logging output quickly

```bash
cd ~/autonomous-tractor-safety-system/testing_tools
ls -lh real_world_logs/
head -n 3 real_world_logs/gps_truth_*.csv
head -n 3 real_world_logs/fused_detections_*.csv
```

## 7) Safety reminder

This is a research prototype, not safety-certified. Always use a controlled area, a human safety operator, and a tested emergency stop procedure.
```
