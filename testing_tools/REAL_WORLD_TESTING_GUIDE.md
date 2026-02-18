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

```bash
sudo apt update
sudo apt install -y can-utils
sudo apt install -y ros-jazzy-ntrip-client

# Serial permissions for GPS (then log out/in)
sudo usermod -a -G dialout $USER
```

---

## Test Execution Instructions

### Test cases

This guide assumes **standing pedestrian only** and **moving tractor approaching**.

Definitions:
- **Centerline:** tractor forward path (+X direction from `base_link`)
- **Working width:** tractor width + implement width (lateral hazard zone)

Repeat each case **7 times**.

S1. **Centerline (standing pedestrian)**
- Pedestrian stands on the centerline, directly ahead.
- Suggested start distance: 30 m.

S2. **Working-width edges (standing pedestrian)**
- Pedestrian stands near the left boundary ($Y = +W/2$).
- Repeat on the right boundary ($Y = -W/2$).

S3. **Outside working width (standing pedestrian)**
- Pedestrian stands outside the working width (left and right).
- Suggested offsets: 1–2 m outside boundary, start distances 30 m and 20 m.

Record for each run:
- Scenario + repeat index
- Pedestrian lateral position (m)
- Tractor speed (m/s)
- Weather/lighting
- Any warnings/errors

---

### Script-based workflow (recommended)

**1) GPS + (optional) NTRIP**

```bash
cd ~/autonomous-tractor-safety-system/testing_tools
./gps_rtk_start.sh
ros2 topic echo /gps/fix --once
```

Optional RTK verification:

```bash
ros2 topic info /rtcm -v
ros2 topic echo /rxmrtcm --once
```

**2) Validate system**

Before validation, bring up can:

```bash
cd ~/autonomous-tractor-safety-system/testing_tools
./setup_can.sh
```
```bash
cd ~/autonomous-tractor-safety-system/testing_tools
./validate_system.sh
```

**3) Launch system + logger**

Use calibrated TF values here whenever available:

```bash
cd ~/autonomous-tractor-safety-system/testing_tools
./launch_test.sh field_test_S1 /gps/fix \
  <camera_x> <camera_y> <camera_z> <camera_roll> <camera_pitch> <camera_yaw> \
  <radar_x>  <radar_y>  <radar_z>  <radar_roll>  <radar_pitch>  <radar_yaw>
```

**4) Monitor**

```bash
cd ~/autonomous-tractor-safety-system/testing_tools
./monitor_test.sh
```

---

### Manual workflow (optional)

Use this when debugging or if your setup differs from the scripts.

**CAN:**

```bash
cd ~/autonomous-tractor-safety-system/testing_tools
sudo ./setup_can.sh can0 1000000
candump can0
```

**GPS + NTRIP (integrated launch):**

```bash
cd ~/autonomous-tractor-safety-system
source install/setup.bash

ros2 launch tractor_safety_system_launch gps_rtk_ublox_ntrip.launch.py \
  gps_device:=/dev/ttyACM0 \
  gps_baudrate:=115200 \
  rtcm_topic:=/rtcm \
  start_ntrip:=true \
  ntrip_host:=rtk2go.com \
  ntrip_port:=2101 \
  ntrip_mountpoint:=Ranta \
  ntrip_authenticate:=true \
  ntrip_username:="YOUR_EMAIL" \
  ntrip_password:="YOUR_PASSWORD"
```

**Main system:**

```bash
ros2 launch tractor_safety_system_launch real_world_test.launch.py \
  test_name:=field_test_S1 \
  gps_fix_topic:=/gps/fix \
  camera_tf_x:=<camera_x> camera_tf_y:=<camera_y> camera_tf_z:=<camera_z> \
  camera_tf_roll:=<camera_roll> camera_tf_pitch:=<camera_pitch> camera_tf_yaw:=<camera_yaw> \
  radar_tf_x:=<radar_x> radar_tf_y:=<radar_y> radar_tf_z:=<radar_z> \
  radar_tf_roll:=<radar_roll> radar_tf_pitch:=<radar_pitch> radar_tf_yaw:=<radar_yaw>
```

**Stopping + backup:**

```bash
cd ~/autonomous-tractor-safety-system/testing_tools
ls -lh real_world_logs/
wc -l real_world_logs/*.csv

backup_dir=~/test_backups/$(date +%Y%m%d_%H%M%S)
mkdir -p "$backup_dir"
cp -r real_world_logs "$backup_dir"/
```

---

## Post-Test Analysis

### Immediate integrity checks

```bash
cd ~/autonomous-tractor-safety-system/testing_tools
ls -lh real_world_logs/
head -n 5 real_world_logs/raw_detections_*.csv
head -n 5 real_world_logs/ego_motion_*.csv
```

### Scenario analysis (optional)

```bash
cd ~/autonomous-tractor-safety-system/testing_tools
mkdir -p plots

TEST_NAME=field_test_S1

python3 analysis_tools/analyze_scenario.py \
  --scenario S1 \
  --raw   "real_world_logs/raw_detections_${TEST_NAME}.csv" \
  --fused "real_world_logs/fused_detections_${TEST_NAME}.csv" \
  --tracks "real_world_logs/tracks_${TEST_NAME}.csv" \
  --ego   "real_world_logs/ego_motion_${TEST_NAME}.csv" \
  --out_prefix "plots/${TEST_NAME}"
```

Note: update the static GT configuration inside `analysis_tools/analyze_scenario.py` to match your marked test geometry before trusting numeric metrics.

---

## Command Reference

### Core scripts

```bash
cd ~/autonomous-tractor-safety-system/testing_tools
./gps_rtk_start.sh
./validate_system.sh

# Positions-only mode
./launch_test.sh field_test_S1 /gps/fix 1.2 0.0 0.8 1.5 0.0 0.5

# Full TF mode (recommended after calibration)
./launch_test.sh field_test_S1 /gps/fix \
  <camera_x> <camera_y> <camera_z> <camera_roll> <camera_pitch> <camera_yaw> \
  <radar_x>  <radar_y>  <radar_z>  <radar_roll>  <radar_pitch>  <radar_yaw>

./monitor_test.sh
sudo ./setup_can.sh can0 1000000
```

### Useful ROS checks

```bash
ros2 node list
ros2 topic list
ros2 topic hz /radar_detections
ros2 topic hz /camera_detections
ros2 topic hz /fused_detections
ros2 topic hz /tracked_detections
ros2 topic hz /ego_motion
ros2 topic echo /rosout --once
```

### GPS + RTK verification

```bash
ros2 topic echo /gps/fix --once
ros2 topic info /rtcm -v
ros2 topic echo /rxmrtcm --once
```

---

## Safety Reminders

This is a research prototype and is **not** safety-certified.

Minimum rules:
- Always have a trained safety operator in control.
- Use a controlled area with clear exclusion zones.
- Keep an emergency stop accessible and tested.
- Start with low speeds and large clearances.
- Stop immediately on any unexpected behavior, warnings, or missing sensor data.

---

## Additional Resources

- [Project README](../README.md)
- [Calibration Guide](CALIBRATION.md)
- Scripts:
  - [gps_rtk_start.sh](gps_rtk_start.sh)
  - [validate_system.sh](validate_system.sh)
  - [launch_test.sh](launch_test.sh)
  - [monitor_test.sh](monitor_test.sh)
  - [setup_can.sh](setup_can.sh)
- Analysis tools: [analysis_tools](analysis_tools)
