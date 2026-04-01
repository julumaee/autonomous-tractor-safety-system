# camera_interface

## 1. Package purpose
- **Package:** `camera_interface`
- **Primary path:** `src/camera_interface`
- **Purpose:** Convert camera detections from `depthai-ros-driver` into the project-specific camera detection message used by the rest of the perception stack.

This package provides the ROS 2 bridge from neural-network-based `Detection3DArray` outputs to `CameraDetection` messages in the coordinate convention expected by fusion.

---

## 2. Nodes

| Node | Source file | Short purpose |
|---|---|---|
| `camera_node` | `camera_interface/camera_node.py` | Converts `Detection3DArray` messages from the camera stack into `/camera_detections`. |

### 2.1 `camera_node`

#### 2.1.1 Overview
- **Node name:** `camera_node`
- **Executable:** `camera_node`
- **Source file:** `src/camera_interface/camera_interface/camera_node.py`
- **Purpose:** Repackage camera detections into the system’s custom message format and normalize coordinate conventions.

The node sits directly downstream of the OAK-D / `depthai-ros-driver` pipeline. It reads 3D detections, extracts class hypotheses and object position, converts from camera optical-frame coordinates to `camera_link`, and publishes one `CameraDetection` per detected object.

#### 2.1.2 Topics and interfaces

##### Subscribed topics
| Topic | Message type | Callback / handler | Purpose | Notes |
|---|---|---|---|---|
| `/oak/nn/spatial_detections` | `vision_msgs/msg/Detection3DArray` | `publish_detections()` | Consume spatial detections from the camera NN pipeline | Topic is configurable with `input_topic` |

##### Published topics
| Topic | Message type | Publisher / function | Purpose | Notes |
|---|---|---|---|---|
| `/camera_detections` | `tractor_safety_system_interfaces/msg/CameraDetection` | `publisher_` via `publish_detections()` | Publish one converted project-specific camera detection per object | Topic is configurable with `output_topic`; frame defaults to `camera_link` |

##### Other interfaces (optional)
- **Services:** none
- **Actions:** none
- **TF frames used or produced:** Does not perform TF lookups; directly remaps coordinates from optical frame to `camera_link`
- **Timers:** none

#### 2.1.3 Parameters
Document every parameter declared or used by this node.

| Parameter | Type | Default | Units | Used in | Description |
|---|---|---:|---|---|---|
| `input_topic` | `string` | `/oak/nn/spatial_detections` | - | `__init__()` | Input `Detection3DArray` topic from the camera stack |
| `output_topic` | `string` | `/camera_detections` | - | `__init__()` | Output topic for converted `CameraDetection` messages |
| `frame_id` | `string` | `camera_link` | frame | `__init__()`, `on_set_parameters()`, `convert_message()` | Output frame assigned to published detections after coordinate conversion |
| `queue_size` | `int` | `10` | messages | `__init__()` | Queue depth used for the subscriber and publisher |
| `enable_logging` | `bool` | `False` | - | `__init__()`, `on_set_parameters()`, `convert_message()`, `publish_detections()` | Enables detailed per-detection logging and warnings |

#### 2.1.4 Functions
Document every function or method in this node.

##### `__init__()`
- **Purpose:** Initialize the node, declare parameters, and create the camera subscription and publisher.
- **Called by:** `main()`
- **Inputs:** none
- **Returns:** none
- **Main steps:**
  1. Declare runtime parameters.
  2. Read topic names, queue size, frame ID, and logging mode.
  3. Create the publisher and subscription.
  4. Register the parameter-update callback.
  5. Log startup configuration.
- **Parameters used:** `input_topic`, `output_topic`, `frame_id`, `queue_size`, `enable_logging`
- **Notes:** Assumes the input topic publishes `Detection3DArray` objects compatible with `depthai-ros-driver` spatial detections.

##### `on_set_parameters(params)`
- **Purpose:** Apply runtime updates to selected parameters.
- **Called by:** ROS parameter callback system
- **Inputs:** parameter list
- **Returns:** `SetParametersResult`
- **Main steps:**
  1. Update `frame_id` if changed.
  2. Update logging mode if changed.
  3. Log the change.
- **Parameters used:** `frame_id`, `enable_logging`
- **Notes:** Topic remapping is not dynamically recreated after startup.

##### `convert_message(detection)`
- **Purpose:** Convert one `Detection3D` into one `CameraDetection`.
- **Called by:** `publish_detections()`
- **Inputs:** one `vision_msgs/Detection3D`
- **Returns:** `CameraDetection` or `None`
- **Main steps:**
  1. Create a new `CameraDetection` with current timestamp and configured frame.
  2. Extract class hypotheses from `results` if present.
  3. Try to read 3D position first from `results[0].pose.pose.position`.
  4. Fall back to `bbox.center.position` if needed.
  5. Convert coordinates from optical frame `(x right, y down, z forward)` to `camera_link` `(x forward, y left, z up)`.
  6. Warn and fall back to origin if no valid 3D position is found.
- **Parameters used:** `frame_id`, `enable_logging`
- **Notes:** Includes heuristics to detect likely pixel-coordinate values masquerading as 3D positions.

##### `publish_detections(detection_array_msg)`
- **Purpose:** Process an incoming detection array and publish each converted detection.
- **Called by:** ROS subscription callback on the input topic
- **Inputs:** one `Detection3DArray`
- **Returns:** none
- **Main steps:**
  1. Validate that the message has a usable `detections` field.
  2. Iterate over each detection.
  3. Convert each detection with `convert_message()`.
  4. Publish every successful conversion.
  5. Optionally log detailed per-detection output or a summary.
- **Parameters used:** `enable_logging`
- **Notes:** Publishes detections individually rather than republishing the full array.

##### `main(args=None)`
- **Purpose:** ROS 2 entry point.
- **Called by:** console script `camera_node`
- **Inputs:** optional ROS arguments
- **Returns:** none
- **Main steps:** initialize ROS, create node, spin, destroy node, shut down ROS.
- **Parameters used:** none directly
- **Notes:** Standard ROS 2 Python entry point.

#### 2.1.5 Processing flow
1. The node subscribes to the configured `Detection3DArray` topic from the camera stack.
2. For each incoming array, it validates the message structure.
3. Each `Detection3D` is converted into a `CameraDetection`.
4. Class hypotheses are copied and the object position is extracted.
5. Coordinates are transformed from optical-frame convention to `camera_link` convention.
6. One converted message per detection is published to `/camera_detections`.

#### 2.1.6 Notes / failure cases
- **Notes:**
  - The node expects spatial 3D detections, not plain 2D image detections.
  - Preferred 3D position source is `results[0].pose.pose.position`; `bbox.center.position` is only a fallback.
  - Output frame labeling is independent of TF; this node performs a direct coordinate remapping only.
- **Failure cases:**
  - If the upstream camera driver publishes pixel-like coordinates instead of metric 3D positions, output positions will be invalid even though the node still runs.
  - If a detection has no usable `results` or no valid 3D position source, the node warns and publishes with a default position.
  - If the incoming message structure is malformed, the callback skips the message and logs a warning or error.

---

## 3. Tests in `/test`

### Test overview
This package contains one main functional unit test for message conversion and the standard ROS packaging quality tests:
- conversion and publication behavior for sample `Detection3DArray` input,
- `flake8` style check,
- `pep257` docstring check,
- copyright check.

### Test files
| Test file | Target | Type | Purpose | Notes |
|---|---|---|---|---|
| `test_camera_node.py` | `camera_node` | unit | Verifies that two sample `Detection3D` objects are converted and published with the correct frame and optical-to-link coordinate conversion | Uses synthetic `Detection3DArray` input |
| `test_flake8.py` | package | lint | Verifies style compliance | Standard ROS packaging test |
| `test_pep257.py` | package | lint | Verifies docstring compliance | Standard ROS packaging test |
| `test_copyright.py` | package | lint | Verifies copyright headers | Standard ROS packaging test |

### Running the tests
- **Command(s):** `colcon test --packages-select camera_interface` or `pytest src/camera_interface/test`
- **Prerequisites:** ROS 2 environment sourced; Python message packages available
- **Expected environment:** No live camera hardware is required because the functional test uses synthetic ROS messages.

### Notes on test coverage
- The core conversion path from `Detection3DArray` to `CameraDetection` is covered.
- Runtime parameter updates for `frame_id` and logging mode are not explicitly unit-tested.
- Edge cases around malformed upstream `Detection3D` content are partly handled in code but only lightly exercised by tests.
