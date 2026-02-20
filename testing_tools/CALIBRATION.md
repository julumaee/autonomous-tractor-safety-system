# Sensor Calibration - Quick Guide

## What You Need

- One person to act as target (you!)
- Measuring tape to mark positions (5m, 10m, 15m from tractor)
- ~10 minutes

## Physical Setup

### 1. Define Base Point
- Choose tractor's base reference point
- This is your coordinate origin (0, 0, 0)

### 2. Mark Target Positions
Mark on the ground where you'll stand (relative to base point):

**Minimum (3 positions)**:
```
Position 1:  5m straight ahead   (x=5.0, y=0.0, z=0.0)
Position 2: 10m straight ahead   (x=10.0, y=0.0, z=0.0)
Position 3: 15m straight ahead   (x=15.0, y=0.0, z=0.0)
```

**Recommended (4 positions)**:
```
Position 4: 5m, 2m to left       (x=5.0, y=2.0, z=0.0)
```

---

## Calibration Procedure

### Step 1: Create Ground Truth File

```bash
cd ~/autonomous-tractor-safety-system/testing_tools
source ../install/setup.bash

# Create calibration_log/ground_truth.csv with YOUR actual target positions
nano calibration_log/ground_truth.csv
```

Example (edit to match your marked positions):
```csv
x,y,z,description
5.0,0.0,0.0,pos_5m
10.0,0.0,0.0,pos_10m
15.0,0.0,0.0,pos_15m
5.0,2.0,0.0,pos_5m_left
```

Important: in sequential calibration, `target_name` must match the `--target-name` values you used while collecting (e.g., `pos_5m`).

### Step 2: Start Perception Stack

Open new terminal:
```bash
cd ~/autonomous-tractor-safety-system
source install/setup.bash

# Start only radar and camera nodes (no fusion, no tracker)
ros2 launch tractor_safety_system_launch perception_stack.launch.py \
    start_tracker:=false
```

**Note**: Only these nodes are needed:
- `radar_node` - publishes `/radar_detections`
- `camera_node` - publishes `/camera_detections`

### Step 3: Collect Data (One Position at a Time)

Choose one of the two ways to run calibration.

**Interactive mode (easiest)**:
```bash
# Back in testing_tools terminal
./run_calibration.sh
# Select option 1 (Sequential calibration)
```

**Manual mode**:
```bash
# Position 1: Walk to first mark (5m), stand still, face tractor
python3 sensor_calibration.py --mode collect --duration 15 --start-delay 10 --target-name pos_5m

# Position 2: Walk to 10m mark, stand still
python3 sensor_calibration.py --mode collect --duration 15 --start-delay 10 --target-name pos_10m --append

# Position 3: Walk to 15m mark, stand still
python3 sensor_calibration.py --mode collect --duration 15 --start-delay 10 --target-name pos_15m --append

# Position 4: Walk to 5m left mark, stand still
python3 sensor_calibration.py --mode collect --duration 15 --start-delay 10 --target-name pos_5m_left --append
```

### Step 4: Analyze

```bash
python3 sensor_calibration.py --mode analyze \
    --data calibration_log/calibration_raw_combined.csv \
    --ground-truth calibration_log/ground_truth.csv
```

**For OAK-D camera**: The script knows the camera has fixed rotations (roll=-π/2, yaw=-π/2) and will calibrate position with these constraints.

**The script will output precise calibrated TF parameters.**

### Step 5: Apply Calibrated Values

Copy the output parameters and restart perception stack:

```bash
# Stop previous launch (Ctrl+C)

# Restart with CALIBRATED values from step 4 output
ros2 launch tractor_safety_system_launch perception_stack.launch.py \
    radar_tf_x:=1.523 \
    radar_tf_y:=-0.012 \
    radar_tf_z:=0.487 \
    radar_tf_roll:=0.0052 \
    radar_tf_pitch:=0.0123 \
    radar_tf_yaw:=-0.0089 \
    camera_tf_x:=1.189 \
    camera_tf_y:=0.034 \
    camera_tf_z:=0.812 \
    camera_tf_roll:=-1.5820 \
    camera_tf_pitch:=0.0034 \
    camera_tf_yaw:=-1.5650
```

Use the exact values from your calibration output.

### Step 6: Validate

```bash
# Check fused detections match expected positions
ros2 topic echo /fused_detections

# Stand at known position (e.g., 10m ahead)
# Detection position should be close to (10, 0, 0)
```

---

## Important Notes

### During Calibration
- ✅ Tractor must be stationary (engine off or parking brake)
- ✅ Stand very still at each position
- ✅ Face the tractor

### Good Calibration Indicators
- Average error < 0.5m
- Number of detected clusters = number of positions
- Both radar and camera detect you at each position

### Troubleshooting

**"No detections"**:
```bash
# Check topics are publishing
ros2 topic hz /radar_detections
ros2 topic hz /camera_detections

# Start closer (try 3m) if not detecting
```

**"Only radar OR camera detecting"**:
- This is OK! Calibration works with partial data
- Some positions may only be visible to one sensor

**"High error (>1m)"**:
- Verify ground truth positions are accurate
- Ensure you stood still at each position
- Try 4-6 positions instead of 3
- Increase collection time to 20-30 seconds

---

## Quick Reference

```bash
# Complete workflow
cd ~/autonomous-tractor-safety-system/testing_tools
source ../install/setup.bash

# 1. Create ground truth with your target positions
nano calibration_log/ground_truth.csv  # Edit to match your positions

# 2. Start perception (separate terminal)
cd ~/autonomous-tractor-safety-system
source install/setup.bash
ros2 launch tractor_safety_system_launch perception_stack.launch.py \
    start_tracker:=false

# 3. Interactive calibration (easiest)
cd ~/autonomous-tractor-safety-system/testing_tools
./run_calibration.sh
# Select option 1 (Sequential calibration)

# 4. Or manual collection
python3 sensor_calibration.py --mode collect --duration 15 --target-name pos_5m
python3 sensor_calibration.py --mode collect --duration 15 --target-name pos_10m --append
python3 sensor_calibration.py --mode collect --duration 15 --target-name pos_15m --append

# 5. Analyze and get calibrated parameters
python3 sensor_calibration.py --mode analyze \
    --data calibration_log/calibration_raw_combined.csv \
    --ground-truth calibration_log/ground_truth.csv

# 6. Restart with calculated parameters (from step 5 output)
ros2 launch tractor_safety_system_launch perception_stack.launch.py \
    radar_tf_x:=1.523 radar_tf_y:=-0.012 radar_tf_z:=0.487 \
    radar_tf_roll:=0.0052 radar_tf_pitch:=0.0123 radar_tf_yaw:=-0.0089 \
    camera_tf_x:=1.189 camera_tf_y:=0.034 camera_tf_z:=0.812 \
    camera_tf_roll:=-1.5820 camera_tf_pitch:=0.0034 camera_tf_yaw:=-1.5650
```

## Key Points

- **What YOU measure**: Target positions on ground (must be accurate!)
- **What SCRIPT calculates**: Exact sensor positions and orientations
- **Dependencies**: `pip3 install numpy scipy scikit-learn`
