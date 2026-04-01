# radar_interface

## 1. Package purpose
- **Package:** `radar_interface`
- **Primary path:** `src/radar_interface`
- **Purpose:** Provide a ROS 2 interface from the automotive radar CAN stream to the rest of the perception pipeline.

This package contains the radar ingress node that reads raw CAN frames, reconstructs radar targets, applies basic validity filtering, and publishes ROS-native radar detections.

---

## 2. Nodes

| Node | Source file | Short purpose |
|---|---|---|
| `radar_publisher` | `radar_interface/radar_node.py` | Reads radar CAN frames, reconstructs target messages, and publishes `/radar_detections`. |

### 2.1 `radar_publisher`

#### 2.1.1 Overview
- **Node name:** `radar_publisher`
- **Executable:** `radar_node`
- **Source file:** `src/radar_interface/radar_interface/radar_node.py`
- **Purpose:** Convert paired CAN frames from the radar into `RadarDetection` messages.

The node is the sensor-side entry point for radar data. It owns the CAN connection, polls the bus periodically, buffers partial two-frame radar target messages, decodes target geometry and speed, filters detections by height, and publishes a standardized message for downstream fusion.

#### 2.1.2 Topics and interfaces

##### Subscribed topics
This node does not subscribe to ROS topics.

##### Published topics
| Topic | Message type | Publisher / function | Purpose | Notes |
|---|---|---|---|---|
| `/radar_detections` | `tractor_safety_system_interfaces/msg/RadarDetection` | `publisher_` via `publish_radar_detection()` | Publish one decoded radar target as a ROS message | `header.frame_id` is `radar_link` |

##### Other interfaces (optional)
- **Services:** none
- **Actions:** none
- **TF frames used or produced:** Publishes detections in `radar_link`; TF itself is not published here.
- **Timers:** `poll_can()` runs every `poll_period` seconds.
- **Other I/O:** Connects to SocketCAN through `python-can`.

#### 2.1.3 Parameters
Document every parameter declared or used by this node.

| Parameter | Type | Default | Units | Used in | Description |
|---|---|---:|---|---|---|
| `can_channel` | `string` | `can0` | - | `__init__()`, `_connect_to_can_bus()`, `on_set_parameters()` | SocketCAN channel used for radar communication |
| `frame_timeout` | `float` | `0.5` | s | `__init__()`, `_cleanup_stale_frames()` | Maximum allowed age for incomplete frame pairs before they are discarded |
| `poll_period` | `float` | `0.01` | s | `__init__()`, `poll_can()` | Timer period for draining the CAN receive queue |
| `min_height_m` | `float` | `0.5` | m | `__init__()`, `publish_radar_detection()` | Minimum accepted target height before publication |
| `use_abs_height` | `bool` | `True` | - | `__init__()`, `publish_radar_detection()` | If true, filter by absolute height magnitude instead of signed height |
| `lateral_sign` | `float` | `-1.0` | sign | `__init__()`, `on_set_parameters()`, `publish_radar_detection()` | Flips radar lateral axis to match ROS base-frame convention (`+y` left) |

#### 2.1.4 Functions
Document every function or method in this node.

##### `__init__()`
- **Purpose:** Initialize the node, declare parameters, create the publisher and timer, and open the CAN bus.
- **Called by:** `main()`
- **Inputs:** none
- **Returns:** none
- **Main steps:**
  1. Create the node and publisher.
  2. Declare and read runtime parameters.
  3. Connect to the CAN bus.
  4. Initialize frame buffers and the polling timer.
  5. Register the parameter-update callback.
- **Parameters used:** all declared parameters
- **Notes:** Starts with `bus=None` and tries immediate CAN connection.

##### `_connect_to_can_bus()`
- **Purpose:** Connect or reconnect the node to the configured CAN channel.
- **Called by:** `__init__()`, `on_set_parameters()`
- **Inputs:** none
- **Returns:** none
- **Main steps:**
  1. Shut down an existing bus if present.
  2. Create a new `can.interface.Bus` using SocketCAN.
  3. Log success or failure.
- **Parameters used:** `can_channel`
- **Notes:** On failure, sets `self.bus` to `None` so polling safely no-ops.

##### `on_set_parameters(params)`
- **Purpose:** Apply runtime parameter updates.
- **Called by:** ROS parameter callback system
- **Inputs:** parameter list
- **Returns:** `SetParametersResult`
- **Main steps:**
  1. Check each changed parameter.
  2. Update cached values.
  3. Reconnect the CAN bus if the channel changes.
- **Parameters used:** `can_channel`, `lateral_sign`
- **Notes:** Other parameters are not dynamically mirrored here.

##### `poll_can()`
- **Purpose:** Drain all queued CAN frames and trigger stale-buffer cleanup.
- **Called by:** timer
- **Inputs:** none
- **Returns:** none
- **Main steps:**
  1. Exit if no CAN bus is connected.
  2. Repeatedly read all immediately available frames.
  3. Pass each frame to `process_radar_data()`.
  4. Remove timed-out incomplete frame pairs.
- **Parameters used:** indirectly `frame_timeout`
- **Notes:** Drains the queue fully each tick to avoid frame backlog.

##### `_decode_confidence_percent(frame1)`
- **Purpose:** Decode the radar-reported confidence value from the second CAN frame.
- **Called by:** `publish_radar_detection()`
- **Inputs:** raw frame-1 bytes
- **Returns:** confidence percentage as `float`
- **Main steps:**
  1. Extract bits 0..5 from byte 7.
  2. Scale to percent by `raw * 5.0`.
  3. Clamp to `0..100`.
- **Parameters used:** none
- **Notes:** Implements the vendor-specific bit layout.

##### `process_radar_data(frame)`
- **Purpose:** Accept one CAN frame, buffer it by target ID, and publish once both halves are present.
- **Called by:** `poll_can()`
- **Inputs:** one `can.Message`
- **Returns:** none
- **Main steps:**
  1. Reject messages with the wrong arbitration ID or length.
  2. Decode target ID and frame number.
  3. Update the target timestamp.
  4. Store frame 0 or frame 1 in the per-target buffer.
  5. Call `publish_radar_detection()` when both halves exist.
- **Parameters used:** none directly
- **Notes:** Handles out-of-order arrival of the two frame halves.

##### `_cleanup_stale_frames()`
- **Purpose:** Remove incomplete target frame pairs that have aged out.
- **Called by:** `poll_can()`
- **Inputs:** none
- **Returns:** none
- **Main steps:**
  1. Compare current time against each buffered target timestamp.
  2. Collect targets older than `frame_timeout`.
  3. Remove them from both tracking dictionaries.
  4. Log a warning.
- **Parameters used:** `frame_timeout`
- **Notes:** Prevents unbounded growth of incomplete frame buffers.

##### `publish_radar_detection(target_id)`
- **Purpose:** Decode a complete two-frame target and publish a `RadarDetection`.
- **Called by:** `process_radar_data()`
- **Inputs:** buffered target ID
- **Returns:** none
- **Main steps:**
  1. Retrieve `frame0` and `frame1`.
  2. Remove the target from the buffer to avoid duplicate publishing.
  3. Decode longitudinal distance, lateral distance, speed, height, and confidence.
  4. Apply lateral sign correction.
  5. Filter detections below the configured height threshold.
  6. Populate and publish a `RadarDetection` message.
- **Parameters used:** `lateral_sign`, `use_abs_height`, `min_height_m`
- **Notes:** Publishes in `radar_link` and uses Euclidean distance for the `distance` field.

##### `main(args=None)`
- **Purpose:** ROS 2 entry point.
- **Called by:** console script `radar_node`
- **Inputs:** optional ROS arguments
- **Returns:** none
- **Main steps:** initialize ROS, create node, spin, destroy node, shut down ROS.
- **Parameters used:** none directly
- **Notes:** Standard ROS 2 Python launcher.

#### 2.1.5 Processing flow
1. The node starts, reads its CAN configuration, and attempts to connect to the radar bus.
2. A timer periodically calls `poll_can()`.
3. Each CAN frame is classified as frame 0 or frame 1 for a target ID and stored.
4. Once both frame halves are available, the node decodes physical values from the bit fields.
5. Height and axis-convention filtering are applied.
6. The decoded target is published on `/radar_detections`.
7. Incomplete stale pairs are removed if their timeout expires.

#### 2.1.6 Notes / failure cases
- **Notes:**
  - The node assumes radar messages arrive on arbitration ID `0x701` and are exactly 8 bytes long.
  - Lateral sign conversion is important because many radar vendor frames use a right-positive convention.
  - Height filtering is applied before publication, so some low objects are intentionally suppressed.
- **Failure cases:**
  - If the CAN bus cannot be opened, the node stays alive but publishes nothing.
  - If only one half of a target pair arrives, the pair is eventually discarded after `frame_timeout`.
  - If the radar message format changes, decoded geometry and confidence will be wrong because the bit extraction is hard-coded.

---

## 3. Tests in `/test`

### Test overview
This package contains one functional unit test for radar decoding and three standard ROS package quality tests:
- behavioral test for target decoding and publication,
- `flake8` style check,
- `pep257` docstring check,
- copyright check.

### Test files
| Test file | Target | Type | Purpose | Notes |
|---|---|---|---|---|
| `test_radar_node.py` | `radar_publisher` | unit | Verifies that paired CAN frames are decoded into one published `RadarDetection` with the expected coordinates and speed | Mocks the CAN bus and publisher |
| `test_flake8.py` | package | lint | Verifies style compliance | Standard ROS packaging test |
| `test_pep257.py` | package | lint | Verifies docstring compliance | Standard ROS packaging test |
| `test_copyright.py` | package | lint | Verifies copyright headers | Standard ROS packaging test |

### Running the tests
- **Command(s):** `colcon test --packages-select radar_interface` or `pytest src/radar_interface/test`
- **Prerequisites:** ROS 2 environment sourced; Python test dependencies available
- **Expected environment:** No live CAN hardware is required for the main behavioral test because the bus is mocked.

### Notes on test coverage
- The decoding path from raw frame bytes to `RadarDetection` publication is covered.
- Runtime parameter reconnection and stale-frame cleanup behavior are not directly unit-tested here.
- CAN connection failures are handled in code but not explicitly exercised by the package tests.
