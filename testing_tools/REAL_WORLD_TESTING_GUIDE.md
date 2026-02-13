# Real-World Tractor Testing - Complete Guide

**Last Updated:** February 12, 2026  
**System:** Autonomous Tractor Safety System (Radar + Camera + GPS)

This guide provides step-by-step instructions for preparing and executing real-world tests of the tractor safety system.

---

## Table of Contents
1. [Quick Start](#quick-start)
2. [Hardware Requirements](#hardware-requirements)
3. [Pre-Test Preparation](#pre-test-preparation)
4. [Test Day Execution](#test-day-execution)
5. [Post-Test Analysis](#post-test-analysis)
6. [Command Reference](#command-reference)
7. [Troubleshooting](#troubleshooting)
8. [Safety Reminders](#safety-reminders)
9. [Additional Resources](#additional-resources)

---

## Quick Start

Recommended script-based workflow:

```bash
cd ~/autonomous-tractor-safety-system/testing_tools

# Terminal A: start GPS (interactive; optional --no-ntrip)
./gps_rtk_start.sh

# Terminal B: (optional) run pre-test checks
./validate_system.sh

# Terminal B: start the perception/logging stack
./launch_test.sh my_test /gps/fix 1.2 0.0 0.8 1.5 0.0 0.5

# Terminal C: monitor the run
./monitor_test.sh

# Optional: CAN-only setup (if you want to do it separately)
sudo ./setup_can.sh can0 1000000
```

## Hardware Requirements

At minimum:
- Radar sensor + CAN interface (CAN @ 1 Mbps)
- Depth camera + USB connection
- RTK-capable GNSS receiver (u-blox ZED-F9P / ArduSimple RTK2B recommended)
- Power supplies for sensors

## Pre-Test Preparation

Complete these steps BEFORE test day in workshop/lab.

### Step 1: Build System

```bash
cd ~/autonomous-tractor-safety-system

# Rebuild packages
colcon build

# Source workspace
source install/setup.bash

# Verify executables
ros2 pkg executables simulations | grep -E "(gps_ego_motion|real_world_logger)"
ros2 pkg executables radar_interface | grep radar_node
ros2 pkg executables camera_interface | grep camera_node
```

**Expected output:**
```
simulations gps_ego_motion
simulations real_world_logger
radar_interface radar_node
camera_interface camera_node
```

---

### Step 2: Configure CAN Interface (Kvaser)

**Test CAN connection:**
```bash
# Bring up CAN interface at 1 Mbps (radar bitrate)
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# Verify interface is up
ip link show can0
# Should show: can0: <NOARP,UP,LOWER_UP,ECHO>

# Test for CAN messages (radar should be powered on)
candump can0
# Should see messages with ID 0x701 when radar detects objects
# Press Ctrl+C to stop
```

**CAN startup script** (recommended):

This repo already includes a helper script:
```bash
cd ~/autonomous-tractor-safety-system/testing_tools
./setup_can.sh
```

---

### Step 3: Set Up GPS

This project is set up to work well with the **ArduSimple RTK2B** using the
**u-blox (UBX) driver + NTRIP corrections**.

#### 3.1 Install ROS 2 packages

```bash
sudo apt update
sudo apt install -y \
   ros-jazzy-ntrip-client
```

If your distro splits the node into a separate package, also install:
```bash
sudo apt-get install -y ros-jazzy-ntrip-client-node
```

Notes:
- This repo provides a workspace overlay of `ublox_gps` with a working `rtcm_input` parameter (RTCM corrections injected via a ROS topic).
- If you also installed `ros-jazzy-ublox-gps`, make sure you source this workspace (`source install/setup.bash`) so the overlay `ublox_gps` is the one that runs.

#### 3.2 USB/serial access

1) Plug in the RTK2B and find the device:
```bash
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

2) If you get permission errors, add yourself to `dialout`:
```bash
sudo usermod -a -G dialout $USER
```
Then log out/in.

#### 3.3 Start u-blox GPS node (publishes position)

Run the u-blox driver (replace `/dev/ttyACM0` and baud if needed):
```bash
ros2 run ublox_gps ublox_gps_node \
   --ros-args \
   -p device:=/dev/ttyACM0 \
   -p baudrate:=115200 \
   -p rtcm_input:=/rtcm \
   -r /fix:=/gps/fix
```

Notes:
- The u-blox driver parameter names can differ by version. If the command above errors,
   first run `ros2 run ublox_gps ublox_gps_node --ros-args --help`, then inspect:
   `ros2 param list /ublox_gps_node`.
- The `-r /fix:=/gps/fix` remap standardizes the topic name for the rest of this system.

Verify fix:
```bash
ros2 topic echo /gps/fix --once
ros2 topic hz /gps/fix
```

#### 3.4 Start NTRIP client (RTCM corrections)

You need an NTRIP caster account (host/port/mountpoint + credentials).

Start the NTRIP client and publish RTCM corrections (example parameters):
```bash
ros2 run ntrip_client ntrip_ros.py \
   --ros-args \
   -p host:="YOUR_CASTER_HOST" \
   -p port:=2101 \
   -p mountpoint:="YOUR_MOUNTPOINT" \
   -p username:="YOUR_USERNAME" \
   -p password:="YOUR_PASSWORD" \
   -p rtcm_topic:=/rtcm
```

Notes:
- On Jazzy, `ntrip_client` is commonly installed with executables like `ntrip_ros.py`.
   If `ros2 run ntrip_client ntrip_ros.py` fails, list available executables with:
   `ros2 pkg executables ntrip_client`.
- Parameter names can vary by version; confirm with `ros2 param list /<ntrip_node_name>` after it starts.

Then ensure your u-blox node is configured to consume RTCM from `/rtcm`.

In this repo, `ublox_gps_node` consumes RTCM from `/rtcm` via the `rtcm_input` parameter.

Verification checklist:
- `ros2 topic info /rtcm -v` shows a subscriber from `ublox_gps_node`
- `ros2 topic echo /rxmrtcm --once` prints a decoded RTCM message (proves the receiver is parsing corrections)

#### 3.4b Recommended: interactive GPS start script

If you prefer not to type (or export) NTRIP credentials manually, use:
```bash
cd ~/autonomous-tractor-safety-system/testing_tools
./gps_rtk_start.sh
```

Defaults are set for RTK2go:
- `ntrip_host`: `rtk2go.com`
- `ntrip_mountpoint`: `Ranta`

To start GPS without NTRIP (no RTK corrections):
```bash
cd ~/autonomous-tractor-safety-system/testing_tools
./gps_rtk_start.sh --no-ntrip
```

#### 3.5 Start GPS-based ego motion

Once `/gps/fix` is publishing, you can run ego-motion:
```bash
ros2 run simulations gps_ego_motion --ros-args -p gps_fix_topic:=/gps/fix

ros2 topic echo /ego_motion --once
ros2 topic hz /ego_motion
```

If you also have:
- velocity topic (`geometry_msgs/TwistStamped`), pass it as `/gps/vel` and set `use_gps_vel:=true`
- heading/course topic (`std_msgs/Float64`, degrees), pass it as `/gps/heading` and set `use_gps_heading:=true`

**GPS Performance Guide:**
- **RTK GPS**: ±2cm accuracy, best for tracking
- **DGPS**: ±0.5-1m accuracy, very good
- **Standard GPS**: ±3-5m accuracy, acceptable
- **Velocity accuracy**: ±0.1-0.5 m/s depending on GPS quality

---

### Step 4: Measure Sensor Positions

**Critical:** Accurately measure sensor positions on the tractor!

**Define base_link:** Choose tractor reference point (typically center of rear axle)

**Measure offsets from base_link:**
```
Positive X = forward
Positive Y = left
Positive Z = up
```

**Example measurements:**
```bash
# Camera: 1.2m forward, 0m lateral, 0.8m up
camera_tf_x=1.2
camera_tf_y=0.0
camera_tf_z=0.8

# Radar: 1.5m forward, 0m lateral, 0.5m up
radar_tf_x=1.5
radar_tf_y=0.0
radar_tf_z=0.5
```

**Record these values** - you'll need them for every test!

Optional (recommended): compute TFs using the calibration tools
- Follow [testing_tools/CALIBRATION.md](testing_tools/CALIBRATION.md)
- Sequential calibration matches targets by name: `--target-name` values must match `target_name` in `ground_truth.csv`

**Sensor orientations** (usually don't need to change):
- Camera: `roll=-1.5708, yaw=-1.5708` (converts camera frame to ROS convention)
- Radar: `roll=0, pitch=0, yaw=0` (forward-facing)

---

### Step 5: Component Testing

Test each component individually before full system test.

#### Test 1: Radar Only

```bash
# Terminal 1: Ensure CAN is up
cd ~/autonomous-tractor-safety-system/testing_tools && ./setup_can.sh

# Terminal 2: Start radar node
ros2 run radar_interface radar_node

# Terminal 3: Monitor radar detections
ros2 topic echo /radar_detections

# Walk in front of radar - you should see detections
# Press Ctrl+C in all terminals when done
```

**Expected:** Detections appear when objects are 2-80m in front of radar

#### Test 2: Camera Only

```bash
# Terminal 1: Launch camera driver
ros2 launch camera_interface oak_d_s2.launch.py

# Terminal 2: Check neural network detections
ros2 topic echo /oak/nn/spatial_detections --once

# Terminal 3: Start camera interface node
ros2 run camera_interface camera_node

# Terminal 4: Monitor camera detections
ros2 topic echo /camera_detections

# Walk in front of camera - you should see person detections
```

**Expected:** Person detections appear when within ~2-30m of camera

#### Test 3: Perception Stack (All Sensors + Fusion + Tracking)

```bash
# Single command to launch everything
ros2 launch tractor_safety_system_launch perception_stack.launch.py \
    start_camera_driver:=true \
    camera_tf_x:=1.2 \
    camera_tf_z:=0.8 \
    radar_tf_x:=1.2 \
    radar_tf_z:=0.7

# In another terminal, monitor outputs
ros2 topic hz /radar_detections
ros2 topic hz /camera_detections
ros2 topic hz /fused_detections
ros2 topic hz /tracked_detections

# Walk in front of tractor - verify all topics update
# Check TF tree
ros2 run rqt_tf_tree rqt_tf_tree
```

**Expected:**
- All topics publish at ~10-30 Hz when objects present
- TF tree shows: `base_link → camera_link` and `base_link → radar_link`
- Fusion combines detections from both sensors
- Tracker maintains consistent IDs for moving objects

#### Test 4: Full System with GPS and Logger

```bash
# Start GPS first.
# Recommended: `./gps_rtk_start.sh` (launches ublox + NTRIP and wires `/rtcm` -> ublox `rtcm_input`).
# Manual alternative: see Step 3 for the exact commands.

# Quick check before launching:
ros2 topic echo /gps/fix --once

# Launch full test system
ros2 launch tractor_safety_system_launch real_world_test.launch.py \
    test_name:=bench_test \
    gps_fix_topic:=/gps/fix \
   use_gps_vel:=false \
   use_gps_heading:=false \
    camera_tf_x:=1.2 \
    camera_tf_z:=0.8 \
    radar_tf_x:=1.5 \
    radar_tf_z:=0.5

# Verify logging
ls -lh real_world_logs/
# Should see 4 CSV files with current timestamp

# Let run for 30 seconds, then Ctrl+C
# Check files have data
wc -l real_world_logs/*.csv
```

**Expected:**
- 4 CSV files created in `real_world_logs/`
- Files grow in size over time
- `ego_motion` file shows GPS-derived velocity/yaw rate

---

### Step 6: Static Calibration Test

Verify sensor accuracy with stationary obstacles at known distances.

**Setup:**
1. Park tractor in open area
2. Place markers at 5m, 10m, 20m in front
3. Ensure GPS has good fix

**Run test:**
```bash
# Start system
ros2 launch tractor_safety_system_launch real_world_test.launch.py \
    test_name:=static_calibration \
    gps_fix_topic:=/gps/fix \
    camera_tf_x:=<YOUR_VALUE> \
    camera_tf_z:=<YOUR_VALUE> \
    radar_tf_x:=<YOUR_VALUE> \
    radar_tf_z:=<YOUR_VALUE>

# Let run for 1 minute
# Then Ctrl+C
```

**Analyze data:**
```bash
# Open CSVs in spreadsheet or Python
# Check:
# 1. Do radar ranges match actual distances? (±0.5m expected)
# 2. Do camera ranges match? (±1-2m expected for camera)
# 3. Are lateral positions (Y) near 0 for centered objects?
# 4. Does fusion combine both sensors correctly?
```

If accuracy is poor, re-check sensor mounting positions and TF parameters.

---

## Test Day Execution

### Pre-Launch Checklist

**Hardware:**
- [ ] All sensors powered on
- [ ] CAN interface connected and tested (`candump can0`)
- [ ] Camera USB connected (check `lsusb | grep -Ei "luxonis|movidius|myriad|oak"`)
- [ ] GPS has fix (`ros2 topic echo /gps/fix --once`)
- [ ] Sensors securely mounted
- [ ] Test area prepared and marked

**Software:**
- [ ] Workspace built and sourced
- [ ] CAN interface configured (`testing_tools/setup_can.sh`)
- [ ] GPS driver running
- [ ] Sensor TF measurements ready

---

### Launch System

**Method 1: Using the Test Launcher Script (Recommended)**

This repo includes a helper script that:
- sets up CAN (`kvaser_usb`, `can0` @ 1 Mbps)
- launches the full stack

Related helper scripts in `testing_tools/`:
- `setup_can.sh` - bring up CAN at the correct bitrate
- `validate_system.sh` - pre-test checks (CAN/GPS/camera/executables/disk)
- `gps_rtk_start.sh` - interactive GPS + optional NTRIP (keeps credentials out of shell history)
- `launch_test.sh` - starts the perception/logging stack (assumes GPS already running)
- `monitor_test.sh` - live status dashboard during tests

From the repo root:
```bash
cd ~/autonomous-tractor-safety-system/testing_tools

# (Optional but recommended) run a quick pre-test validation
./validate_system.sh

# Recommended 2-terminal workflow:
# Terminal A: start GPS (interactive)
./gps_rtk_start.sh

# Terminal B: start the perception/logging stack
./launch_test.sh field_test_1 /gps/fix 1.2 0.0 0.8 1.5 0.0 0.5

# Optional Terminal C: live monitoring dashboard
./monitor_test.sh
```

This is intentionally split into two terminals so NTRIP credentials never need to be exported into your shell.

**Method 2: Manual Launch (Recommended for first tests)**

```bash
# Terminal 1: u-blox GPS driver (replace device/baud)
ros2 run ublox_gps ublox_gps_node \
   --ros-args \
   -p device:=/dev/ttyACM0 \
   -p baudrate:=115200 \
   -p rtcm_input:=/rtcm \
   -r /fix:=/gps/fix

# Terminal 2: NTRIP client (replace with your caster details)
ros2 run ntrip_client ntrip_ros.py \
   --ros-args \
   -p host:="YOUR_CASTER_HOST" \
   -p port:=2101 \
   -p mountpoint:="YOUR_MOUNTPOINT" \
   -p username:="YOUR_USERNAME" \
   -p password:="YOUR_PASSWORD" \
   -p rtcm_topic:=/rtcm

# Terminal 3: Main system
ros2 launch tractor_safety_system_launch real_world_test.launch.py \
    test_name:=field_test_1 \
      gps_fix_topic:=/gps/fix \
    camera_tf_x:=1.2 \
    camera_tf_z:=0.8 \
    radar_tf_x:=1.5 \
    radar_tf_z:=0.5

# Terminal 4: Monitor status
cd ~/autonomous-tractor-safety-system/testing_tools && ./monitor_test.sh
```

Tip: the GPS stack can also be launched directly via:
```bash
ros2 launch tractor_safety_system_launch gps_rtk_ublox_ntrip.launch.py \
   gps_device:=/dev/ttyACM0 \
   gps_baudrate:=115200 \
   rtcm_topic:=/rtcm \
   ntrip_host:="YOUR_CASTER_HOST" \
   ntrip_port:=2101 \
   ntrip_mountpoint:="YOUR_MOUNTPOINT" \
   ntrip_username:="YOUR_USERNAME" \
   ntrip_password:="YOUR_PASSWORD"
```


---

### System Health Monitoring

Keep Terminal 3 visible during tests to monitor:

```bash
# Recommended: run the dashboard monitor script
cd ~/autonomous-tractor-safety-system/testing_tools && ./monitor_test.sh

# Quick status check
ros2 topic hz /radar_detections     # Should be ~20 Hz when objects present
ros2 topic hz /camera_detections    # Should be ~10-30 Hz when objects present
ros2 topic hz /fused_detections     # Updates when detections present
ros2 topic hz /tracked_detections   # Should be ~20 Hz when tracking
ros2 topic hz /ego_motion           # Should be ~10 Hz (GPS rate)

# Check for errors
ros2 topic echo /rosout --once

# Verify GPS
ros2 topic echo /ego_motion --once
# Check: vx and yaw_rate updating when tractor moves

# Verify logging
ls -lh real_world_logs/
# Files should be growing
```

---

### During Testing

**Test Scenarios (moving tractor, standing pedestrian only):**

Definitions:
- **Working width**: tractor width + implement width (the lateral zone the tractor/implement occupies).
- **Centerline**: forward path of the tractor (base_link +X direction).

S1. **Center (standing pedestrian)**
- Pedestrian stands on the tractor centerline, directly ahead.
- Suggested start distance: 30 m.
- Tractor drives straight toward the pedestrian at a constant low speed.
- Repeat **7 times**.

S2. **Working-width edges (standing pedestrian)**
- Pedestrian stands at the **left edge** of the working width (just inside/at the boundary).
- Repeat **7 times**.
- Pedestrian stands at the **right edge** of the working width.
- Repeat **7 times**.

Practical setup tip:
- Measure and mark the working-width boundaries on the ground as:
   - $Y = +\frac{W}{2}$ (left boundary)
   - $Y = -\frac{W}{2}$ (right boundary)
   where $W$ is your working width in meters.

S3. **Outside working width (standing pedestrian)**
- Pedestrian stands **outside** the working width to test rejection / lower risk behavior.
- Recommended: run a small matrix of lateral offsets and start distances, for example:
  - Lateral offset (Y): +1 m and +2 m outside the working-width boundary (repeat for left and right)
  - Start distance (X): 30 m, 20 m, 10 m
- Repeat **7 times per configuration** you choose (or, if time-limited, prioritize 30 m and 20 m).

**For each test:**
- Repeat 7 times (ISO18497)
- Announce test start/end times
- Note any anomalies or system warnings
- Verify logging is active (`ls -lh real_world_logs/`)
- Record environmental conditions (lighting, weather)

Suggested notes to record each run:
- Pedestrian lateral position relative to working width boundary (m)
- Initial range (m) and tractor speed (m/s)
- First-detection range and any false detections
- Track stability (ID continuity) until stop/end

---

### Stopping System

```bash
# In each terminal, press Ctrl+C
# Wait for "Logger stopped" message in Terminal 2

# Verify data saved
ls -lh real_world_logs/
# All CSV files should have reasonable sizes (>1 KB)

# Backup data immediately
mkdir -p ~/test_results/$(date +%Y%m%d)
cp -r real_world_logs/* ~/test_results/$(date +%Y%m%d)/
```

---

## Post-Test Analysis

### Immediate Data Check

```bash
cd real_world_logs/

# Check file sizes
ls -lh *.csv

# Count data points
wc -l *.csv

# Quick preview
head -20 fused_detections_field_test_1_*.csv
```

### Detailed Analysis

Use the existing analysis tool:

```bash
python3 analysis_tools/analyze_scenario.py \
    --scenario field_test_1 \
    --log_dir real_world_logs/
```

**This generates plots of:**
- Detection ranges over time
- Track stability and lifetime
- Sensor fusion performance
- Ego-motion compensation quality

### Evaluation Criteria

**Detection Performance:**
- Radar: Objects detected at 2-80m? ✓/✗
- Camera: Objects detected at 2-30m? ✓/✗
- False positives: <5% of detections? ✓/✗

**Tracking Quality:**
- Tracks maintain consistent IDs? ✓/✗
- Static objects remain ~stationary when tractor moves? ✓/✗
- Average track lifetime: >5 seconds? ✓/✗

**Ego-Motion Compensation:**
- GPS update rate: ~10 Hz? ✓/✗
- Velocity accuracy: ±0.5 m/s? ✓/✗
- Static objects don't drift significantly? ✓/✗

**System Latency:**
- Detection to logging: <100ms? ✓/✗
- Check: `logger_stamp - header_stamp` in CSV files

---

## Command Reference

### Quick Start Commands

```bash
# Setup CAN
sudo modprobe kvaser_usb
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# Test CAN
candump can0

# Start GPS (ArduSimple RTK2B recommended: ublox + NTRIP)
# Recommended: interactive script (defaults: rtk2go.com / Ranta)
cd ~/autonomous-tractor-safety-system/testing_tools
./gps_rtk_start.sh

# Manual alternative (for debugging)
# Terminal A: u-blox driver (replace device/baud)
ros2 run ublox_gps ublox_gps_node \
   --ros-args \
   -p device:=/dev/ttyACM0 \
   -p baudrate:=115200 \
   -p rtcm_input:=/rtcm \
   -r /fix:=/gps/fix

# Terminal B: NTRIP client (replace with your caster details)
# Tip: avoid storing credentials in shell history.
ros2 run ntrip_client ntrip_ros.py \
   --ros-args \
   -p host:="YOUR_CASTER_HOST" \
   -p port:=2101 \
   -p mountpoint:="YOUR_MOUNTPOINT" \
   -p username:="YOUR_USERNAME" \
   -p password:="YOUR_PASSWORD" \
   -p rtcm_topic:=/rtcm

# Verify GPS fix
ros2 topic echo /gps/fix --once

# Verify RTCM is being consumed (RTK corrections)
ros2 topic info /rtcm -v
ros2 topic echo /rxmrtcm --once

# Launch full system
ros2 launch tractor_safety_system_launch real_world_test.launch.py \
    test_name:=my_test \
    gps_fix_topic:=/gps/fix \
    camera_tf_x:=1.2 camera_tf_z:=0.8 \
    radar_tf_x:=1.5 radar_tf_z:=0.5

# Monitor status
ros2 topic hz /ego_motion
ros2 topic hz /tracked_detections
```

### Debugging Commands

```bash
# Check node status
ros2 node list
ros2 node info /radar_node
ros2 node info /gps_ego_motion

# Check topics
ros2 topic list
ros2 topic info /ego_motion
ros2 topic echo /ego_motion --once

# Check TF tree
ros2 run rqt_tf_tree rqt_tf_tree
ros2 run tf2_ros tf2_echo base_link camera_link

# Check for errors
ros2 topic echo /rosout | grep -i error

# System resources
htop
```

### Data Commands

```bash
# Check logs directory
ls -lh real_world_logs/

# Count data points
wc -l real_world_logs/*.csv

# Backup data
mkdir -p ~/test_backups/$(date +%Y%m%d_%H%M%S)
cp -r real_world_logs/* ~/test_backups/$(date +%Y%m%d_%H%M%S)/

# Archive test data
tar -czf test_results_$(date +%Y%m%d).tar.gz real_world_logs/
```

---

## Troubleshooting

### No Radar Detections

**Symptoms:** `/radar_detections` topic not publishing

**Solutions:**
1. Check CAN connection
   ```bash
   candump can0
   # Should see messages 0x701
   ```

2. Verify CAN interface
   ```bash
   ip link show can0
   # Should show UP
   ```

3. Check radar power and configuration

4. Verify radar node running
   ```bash
   ros2 node list | grep radar
   ```

5. Check CAN bitrate (must be 1 Mbps)
   ```bash
   ip -details link show can0
   ```

---

### No Camera Detections

**Symptoms:** `/camera_detections` topic not publishing

**Solutions:**
1. Check USB connection
   ```bash
   lsusb | grep -Ei "luxonis|movidius|myriad|oak"
   # OAK-D often enumerates as Intel Movidius/MyriadX before the driver boots
   ```

2. Check camera driver
   ```bash
   ros2 node list | grep oak
   ros2 topic list | grep oak
   ```

3. Verify neural network running
   ```bash
   ros2 topic echo /oak/nn/spatial_detections --once
   ```

4. Check camera node
   ```bash
   ros2 node info /camera_node
   ```

---

### No GPS Fix

**Symptoms:** GPS status = -1 (NO_FIX)

**Solutions:**
1. Check GPS antenna placement (needs clear sky view)
2. Wait for satellite acquisition (30-60 seconds)
3. Verify GPS power
4. Check GPS configuration

```bash
ros2 topic echo /gps/fix --once
# Look at status.status field
# -1 = NO_FIX, 0 = FIX, 1 = SBAS_FIX, 2 = GBAS_FIX
```

---

### GPS Velocity Too Noisy

**Symptoms:** Jumpy vx and yaw_rate when stationary

**Solutions:**
1. Normal for position-based estimation (±0.2-0.5 m/s noise acceptable)
2. Node filters velocities <0.2 m/s automatically
3. Consider upgrading to GPS with velocity output
4. Consider RTK GPS for better accuracy

---

### No Ego Motion Updates

**Symptoms:** `/ego_motion` not publishing

**Solutions:**
1. Check GPS data arriving
   ```bash
   ros2 topic hz /gps/fix
   ```

2. Verify ego motion node running
   ```bash
   ros2 node list | grep gps_ego_motion
   ```

3. Check GPS timeout warnings
   ```bash
   ros2 topic echo /rosout | grep -i gps
   ```

---

### Poor Sensor Alignment

**Symptoms:** Radar and camera detect same object but positions don't match

**Solutions:**
1. Re-measure sensor positions accurately
2. Verify TF transforms
   ```bash
   ros2 run tf2_ros tf2_echo base_link camera_link
   ros2 run tf2_ros tf2_echo base_link radar_link
   ```

3. Check transform values match your measurements
4. Re-launch with corrected values

---

### Tracking Not Working

**Symptoms:** No `/tracked_detections` output

**Solutions:**
1. Verify ego motion available
   ```bash
   ros2 topic echo /ego_motion --once
   # vx should be non-zero when moving
   ```

2. Check fused detections present
   ```bash
   ros2 topic hz /fused_detections
   ```

3. Verify tracker node running
   ```bash
   ros2 node info /kf_tracker
   ```

---

### Logger Not Saving Data

**Symptoms:** CSV files not created or empty

**Solutions:**
1. Check directory permissions
   ```bash
   ls -ld real_world_logs/
   # Should be writable
   ```

2. Check disk space
   ```bash
   df -h
   ```

3. Verify logger running
   ```bash
   ros2 node list | grep logger
   ```

4. Check for errors
   ```bash
   ros2 topic echo /rosout | grep logger
   ```

---

## Safety Reminders

⚠️ **CRITICAL:**
- This is a RESEARCH PROTOTYPE
- NOT safety-certified for autonomous operation
- Manual supervision REQUIRED at all times
- Emergency stop must be readily accessible
- Test in controlled environment only
- Never rely solely on the system for safety decisions

---

## Additional Resources

- [System Architecture](../README.md)
- [Analysis Tools](analysis_tools/analyze_scenario.py)
- [Analysis Tools Requirements](analysis_tools/requirements.txt)

---

**Document Version:** 2.1  
**Last Updated:** February 12, 2026  
**System Version:** As tested with ROS 2 Jazzy

Good luck with your testing!
