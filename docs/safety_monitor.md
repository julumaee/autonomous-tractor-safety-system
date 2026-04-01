# safety_monitor

## 1. Package purpose
- **Package:** `safety_monitor`
- **Primary path:** `src/safety_monitor`
- **Purpose:** Apply distance-based safety logic to perception outputs and modify vehicle speed commands accordingly.

This package contains the supervisory node that watches perceived obstacles and overrides or forwards upstream control commands based on configured safety distances.

---

## 2. Nodes

| Node | Source file | Short purpose |
|---|---|---|
| `safety_monitor` | `safety_monitor/safety_monitor.py` | Monitors obstacle distances and gates outgoing control speed commands based on safety state. |

### 2.1 `safety_monitor`

#### 2.1.1 Overview
- **Node name:** `safety_monitor`
- **Executable:** `safety_monitor`
- **Source file:** `src/safety_monitor/safety_monitor/safety_monitor.py`
- **Purpose:** Convert obstacle detections into high-level safety states and publish safe control commands.

The node acts as a simple state machine with four states: `agopen`, `moderate`, `slow`, and `stopped`. It listens to incoming detections and AgOpenGPS control commands, decides which safety state should be active, and republishes control commands with speed overrides or a stop command when obstacles are too close.

#### 2.1.2 Topics and interfaces

##### Subscribed topics
| Topic | Message type | Callback / handler | Purpose | Notes |
|---|---|---|---|---|
| `/fused_detections` | `tractor_safety_system_interfaces/msg/FusedDetection` | `control_speed_state()` | Update the safety state based on detected obstacle distance | One detection can change the vehicle state immediately |
| `/control/agopen` | `tractor_safety_system_interfaces/msg/ControlCommand` | `agopen_control()` | Receive upstream steering/speed commands from AgOpenGPS | Steering is preserved through overrides |

##### Published topics
| Topic | Message type | Publisher / function | Purpose | Notes |
|---|---|---|---|---|
| `/control` | `tractor_safety_system_interfaces/msg/ControlCommand` | `publisher_` via `send_stop_command()` and `speed_control()` | Publish the safe command that should be sent to the vehicle controller | Output depends on the current safety state |

##### Other interfaces (optional)
- **Services:** none
- **Actions:** none
- **TF frames used or produced:** none
- **Timers:** `state_control()` runs every `0.1` s

#### 2.1.3 Parameters
Document every parameter declared or used by this node.

| Parameter | Type | Default | Units | Used in | Description |
|---|---|---:|---|---|---|
| `safety_distance_1` | `float` | `40.0` | m | `control_speed_state()`, `state_control()` | Outer safety zone threshold for entering `moderate` speed state |
| `safety_distance_2` | `float` | `10.0` | m | `control_speed_state()`, `state_control()` | Inner safety zone threshold for entering `slow` speed state |
| `stop_distance` | `float` | `3.0` | m | `control_speed_state()` | Immediate stop threshold |
| `speed_override_1` | `float` | `5.0` | speed units / m/s equivalent | `speed_control()` | Maximum allowed speed in `moderate` state |
| `speed_override_2` | `float` | `2.0` | speed units / m/s equivalent | `speed_control()` | Maximum allowed speed in `slow` state |
| `detection_active_reset_time` | `float` | `5.0` | s | `state_control()` | Time without detections required before relaxing `slow` or `moderate` state |
| `vehicle_stopped_reset_time` | `float` | `5.0` | s | `state_control()`, `speed_control()` | Minimum time spent in `stopped` before relaxing to `slow` |

#### 2.1.4 Functions
Document every function or method in this node.

##### `__init__()`
- **Purpose:** Initialize the node, declare parameters, create subscriptions/publisher/timer, and initialize safety-state bookkeeping.
- **Called by:** `main()`
- **Inputs:** none
- **Returns:** none
- **Main steps:**
  1. Create the control publisher.
  2. Subscribe to detections and AgOpenGPS control commands.
  3. Create the periodic safety-state timer.
  4. Declare and read safety parameters.
  5. Register the parameter callback.
  6. Initialize vehicle state and timestamps.
- **Parameters used:** all declared parameters
- **Notes:** Starts in state `agopen`.

##### `on_set_parameters(params)`
- **Purpose:** Apply runtime updates to safety thresholds and timers.
- **Called by:** ROS parameter callback system
- **Inputs:** parameter list
- **Returns:** `SetParametersResult`
- **Main steps:** update cached threshold and timeout values.
- **Parameters used:** all declared parameters
- **Notes:** Lets the safety distances and speed limits be tuned at runtime.

##### `agopen_control(agopen_cmd)`
- **Purpose:** Receive upstream control commands and pass them through the safety policy.
- **Called by:** `/control/agopen` subscription
- **Inputs:** `ControlCommand`
- **Returns:** none
- **Main steps:** cache latest steering angle and speed, then call `speed_control()`.
- **Parameters used:** none directly
- **Notes:** Steering is always preserved; speed may be overridden.

##### `control_speed_state(detection)`
- **Purpose:** Update the safety state based on one obstacle distance.
- **Called by:** `/fused_detections` subscription
- **Inputs:** `FusedDetection`
- **Returns:** none
- **Main steps:**
  1. Read obstacle distance.
  2. If distance is inside `stop_distance`, issue stop and enter `stopped`.
  3. Else if distance is inside `safety_distance_2`, enter `slow`.
  4. Else if distance is inside `safety_distance_1`, enter `moderate`.
  5. Refresh the relevant detection timers.
- **Parameters used:** `stop_distance`, `safety_distance_2`, `safety_distance_1`
- **Notes:** A stop command is published immediately when entering `stopped`.

##### `send_stop_command()`
- **Purpose:** Publish an explicit stop command.
- **Called by:** `control_speed_state()` and fallback safety logic indirectly
- **Inputs:** none
- **Returns:** none
- **Main steps:** create a `ControlCommand` with zero speed, preserve latest steering angle, publish it.
- **Parameters used:** none
- **Notes:** Used for immediate stop response.

##### `state_control()`
- **Purpose:** Relax the safety state after configured no-detection or stopped durations.
- **Called by:** timer
- **Inputs:** none
- **Returns:** none
- **Main steps:**
  1. Compute elapsed time since the last relevant detections.
  2. If in `stopped` long enough, move to `slow`.
  3. If in `slow` long enough without close detections, move to `moderate`.
  4. If in `moderate` long enough without detections, return to `agopen`.
  5. Force `stopped` if the current state is unknown.
- **Parameters used:** `detection_active_reset_time`, `vehicle_stopped_reset_time`
- **Notes:** Handles state relaxation, not obstacle-triggered tightening.

##### `speed_control()`
- **Purpose:** Publish a safe control command based on the current vehicle state.
- **Called by:** `agopen_control()`, `state_control()` fallback path
- **Inputs:** none directly; uses cached AgOpenGPS command and current state
- **Returns:** none
- **Main steps:**
  1. Copy latest steering angle.
  2. In `agopen`, forward the upstream speed unchanged.
  3. In `moderate`, cap speed at `speed_override_1`.
  4. In `slow`, cap speed at `speed_override_2`.
  5. In `stopped`, only allow reverse or zero-speed commands to pass.
  6. In unknown state, publish stop.
- **Parameters used:** `speed_override_1`, `speed_override_2`, `vehicle_stopped_reset_time`
- **Notes:** The `stopped` state intentionally blocks forward commands but allows reverse commands.

##### `main(args=None)`
- **Purpose:** ROS 2 entry point.
- **Called by:** console script `safety_monitor`
- **Inputs:** optional ROS arguments
- **Returns:** none
- **Main steps:** initialize ROS, create node, spin, destroy node, shut down.
- **Parameters used:** none directly
- **Notes:** Standard ROS 2 Python entry point.

#### 2.1.5 Processing flow
1. The node receives obstacle detections on `/fused_detections`.
2. Each detection can tighten the safety state immediately based on configured distance thresholds.
3. The node also receives upstream control commands from `/control/agopen`.
4. On each incoming control command, `speed_control()` republishes a safe output command for the current state.
5. A timer periodically checks whether enough time has passed without hazardous detections to relax the state.
6. The node publishes the resulting safe command on `/control`.

#### 2.1.6 Notes / failure cases
- **Notes:**
  - The safety logic is purely distance-threshold based; it does not classify obstacle direction, type, or motion.
  - Steering commands are passed through unchanged; only speed is modified.
  - The state machine is asymmetric: it tightens immediately on detections and relaxes only after timeout conditions.
- **Failure cases:**
  - If `/fused_detections` stops arriving, the node may eventually relax to less restrictive states once timers expire.
  - If an unknown `vehicle_state` value appears, the node falls back to a safe stop behavior.
  - The node subscribes to `/fused_detections`, not `/tracked_detections`, so it reacts to raw fused outputs rather than tracker continuity.

---

## 3. Tests in `/test`

### Test overview
This package contains focused unit tests for:
- runtime parameter updates,
- safety-state transitions from detection distances,
- speed override behavior for control commands,
plus the standard ROS package quality tests.

### Test files
| Test file | Target | Type | Purpose | Notes |
|---|---|---|---|---|
| `test_safety_monitor_parameters.py` | `safety_monitor` | unit | Verifies that runtime parameter updates correctly change safety thresholds and timing values | Tests `on_set_parameters()` |
| `test_speed_control.py` | `safety_monitor` | unit | Verifies that control commands are overridden or blocked correctly in `moderate`, `slow`, `stopped`, and unknown states | Mocks the control publisher |
| `test_state_transitions.py` | `safety_monitor` | unit | Verifies state transitions caused by detections and timeout-based relaxation logic | Exercises `control_speed_state()` and `state_control()` |
| `test_flake8.py` | package | lint | Verifies style compliance | Standard ROS packaging test |
| `test_pep257.py` | package | lint | Verifies docstring compliance | Standard ROS packaging test |
| `test_copyright.py` | package | lint | Verifies copyright headers | Standard ROS packaging test |

### Running the tests
- **Command(s):** `colcon test --packages-select safety_monitor` or `pytest src/safety_monitor/test`
- **Prerequisites:** ROS 2 environment sourced; project message packages available
- **Expected environment:** No vehicle hardware or live perception nodes are required because the tests use synthetic messages and mocked publishers.

### Notes on test coverage
- Parameter updates, state transitions, and speed override behavior are covered.
- End-to-end integration with the real vehicle controller is not covered by these package tests.
- The tests assume the current simple four-state safety machine; any redesign of that state machine will require corresponding updates.
