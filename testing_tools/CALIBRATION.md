# Sensor Calibration (Current Workflow)

This guide matches the current scripts in `testing_tools/`.

## 1) Prepare ground truth

Create `testing_tools/calibration_log/ground_truth.csv` with measured target points in `base_link` coordinates.

Accepted columns:
- required: `x,y,z`
- optional label: `target_name` (preferred) or `description`

Example:
```csv
x,y,z,target_name
5.0,0.0,0.0,pos_5m
10.0,0.0,0.0,pos_10m
15.0,0.0,0.0,pos_15m
5.0,2.0,0.0,pos_5m_left
```

Important: labels must match the `target_name` entered during collection.

## 2) Start perception nodes for calibration

In a separate terminal:
```bash
cd ~/autonomous-tractor-safety-system
source install/setup.bash

ros2 launch tractor_safety_system_launch perception_stack.launch.py \
  start_tracker:=false
```

This is enough for calibration data collection (`/radar_detections`, `/camera_detections`).

## 3) Run calibration (recommended: interactive)

```bash
cd ~/autonomous-tractor-safety-system/testing_tools
./run_calibration.sh
```

Use option `1` (Sequential calibration), collect one target at a time.

What `run_calibration.sh` now does:
- appends all detections into `calibration_log/calibration_raw_combined.csv`
- logs GNSS helper files (`gps_origin.json`, `navpvt_log.csv`) if available
- runs analysis with selectable radar fit mode (`planar` or `6dof`)
- writes final TF to `calibration_log/calibration_tf.txt`

## 4) Apply calibration in runtime

`launch_test.sh` and `perception_stack.launch.py` read TF values from:
- `testing_tools/calibration_log/calibration_tf.txt`

You can still override values manually with launch args, but the default workflow is file-based.

## 5) Quick validation

```bash
ros2 topic hz /radar_detections
ros2 topic hz /camera_detections
ros2 topic hz /fused_detections
```

Then stand at a known point and verify detections/tracks are close to expected position.

## Notes

- Tractor must stay stationary during calibration collection.
- Use at least 3 points; 4–6 points gives more stable results.
- `planar` radar fit is usually more stable for flat-field setups.
