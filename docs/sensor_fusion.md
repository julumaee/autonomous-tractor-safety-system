# sensor_fusion

## 1. Package purpose
- **Package:** `sensor_fusion`
- **Primary path:** `src/sensor_fusion`
- **Purpose:** Combine camera and radar detections into a unified target stream and maintain target tracks over time.

This package contains the mid-pipeline perception logic. It first performs decision-level radar–camera fusion and then tracks fused or single-sensor detections with a Kalman filter that compensates for ego motion.

---

## 2. Nodes

| Node | Source file | Short purpose |
|---|---|---|
| `fusion_node` | `sensor_fusion/fusion_node.py` | Time-aligns, transforms, associates, and fuses camera/radar detections into `/fused_detections`. |
| `ego_kf_tracker` | `sensor_fusion/kf_tracker.py` | Tracks detections in a world frame and publishes `/tracked_detections`. |

### 2.1 `fusion_node`

#### 2.1.1 Overview
- **Node name:** `fusion_node`
- **Executable:** `fusion_node`
- **Source file:** `src/sensor_fusion/sensor_fusion/fusion_node.py`
- **Purpose:** Perform decision-level radar–camera fusion and publish fused or trusted single-sensor detections.

This node subscribes to `CameraDetection` and `RadarDetection` streams, transforms both into a common fusion frame, buffers them briefly, and tries to pair them using temporal gating and Mahalanobis-distance spatial gating. Matched pairs are fused with weighted least squares. Unmatched detections can still be published individually if they pass source-specific trust rules.

#### 2.1.2 Topics and interfaces

##### Subscribed topics
| Topic | Message type | Callback / handler | Purpose | Notes |
|---|---|---|---|---|
| `/camera_detections` | `tractor_safety_system_interfaces/msg/CameraDetection` | `listen_to_camera()` | Receive camera detections for buffering and fusion | Expected in `camera_frame`, then transformed to `fusion_frame` |
| `/radar_detections` | `tractor_safety_system_interfaces/msg/RadarDetection` | `listen_to_radar()` | Receive radar detections for buffering and fusion | Expected in `radar_frame`, then transformed to `fusion_frame` |

##### Published topics
| Topic | Message type | Publisher / function | Purpose | Notes |
|---|---|---|---|---|
| `/fused_detections` | `tractor_safety_system_interfaces/msg/FusedDetection` | `publisher_` via `attempt_fusion()` | Publish fused detections and selected camera-only / radar-only detections | Frame is `fusion_frame`, normally `base_link` |

##### Other interfaces (optional)
- **Services:** none
- **Actions:** none
- **TF frames used or produced:** looks up transforms `camera_frame -> fusion_frame` and `radar_frame -> fusion_frame` using TF2
- **Timers:** `attempt_fusion()` runs every `0.05` s

#### 2.1.3 Parameters
Document every parameter declared or used by this node.

| Parameter | Type | Default | Units | Used in | Description |
|---|---|---:|---|---|---|
| `time_threshold` | `float` | `0.1` | s | `__init__()`, `on_set_parameters()`, `temporal_match()`, single-sensor timeout logic | Maximum timestamp difference allowed for candidate camera–radar pairing |
| `camera_trust_max` | `float` | `12.0` | m | `verify_detection()`, `camera_cov_xy()`, `is_partner_expected()` | Maximum trusted range for camera-only publication |
| `chi2_threshold` | `float` | `5.99` | - | `match_mahalanobis()` | Chi-square gate for spatial compatibility in 2D |
| `max_buffer_age` | `float` | `1.0` | s | `prune_stale()` | Maximum age for buffered detections before they are discarded |
| `near_immediate_range` | `float` | `3.0` | m | `should_publish_immediately()` | Distance below which detections are published without waiting for a partner |
| `selection_radius` | `float` | `0.5` | m | `select_targets()` | Suppression radius used to keep only one target per neighborhood |
| `fusion_frame` | `string` | `base_link` | frame | `__init__()`, transform helpers | Common frame in which fusion is performed |
| `radar_frame` | `string` | `radar_link` | frame | `transform_radar_to_base()` | Expected source frame for radar detections |
| `camera_frame` | `string` | `camera_link` | frame | `transform_camera_to_base()` | Expected source frame for camera detections |
| `cam_sigma_theta_deg` | `float` | `0.5` | deg | `camera_cov_xy()` | Camera bearing standard deviation |
| `cam_range_a` | `float` | `0.05` | m/m² | `camera_cov_xy()` | Quadratic coefficient for camera range uncertainty |
| `cam_range_b` | `float` | `0.20` | m | `camera_cov_xy()` | Offset term for camera range uncertainty |
| `cam_far_sigma_cap` | `float` | `30.0` | m | `camera_cov_xy()` | Upper cap for camera range sigma at large distance |
| `radar_range_base` | `float` | `0.5` | m | `radar_cov_xy()` | Radar range standard deviation |
| `radar_beamwidth_deg` | `float` | `2.0` | deg | `radar_cov_xy()` | Radar angular uncertainty used as bearing sigma |
| `camera_min_score_single` | `float` | `0.8` | score | `verify_detection()` | Minimum camera classification score for camera-only publication |
| `radar_min_confidence_percent_single` | `float` | `80.0` | % | `verify_detection()` | Minimum radar confidence for radar-only publication |

#### 2.1.4 Functions
Document every function or method in this node.

##### `__init__()`
- **Purpose:** Initialize parameters, TF interfaces, message buffers, subscribers, publisher, and timer.
- **Called by:** `main()`
- **Inputs:** none
- **Returns:** none
- **Main steps:**
  1. Declare and read configuration parameters.
  2. Allocate camera/radar deques and the publication buffer.
  3. Register parameter callback.
  4. Create TF buffer and listener.
  5. Create camera/radar subscriptions, fused publisher, and fusion timer.
- **Parameters used:** all declared parameters
- **Notes:** Uses fixed-size deques to cap buffered detections.

##### `on_set_parameters(params)`
- **Purpose:** Apply runtime updates to cached parameter values.
- **Called by:** ROS parameter callback system
- **Inputs:** parameter list
- **Returns:** `SetParametersResult`
- **Main steps:** update each recognized parameter and keep the node’s cached values synchronized.
- **Parameters used:** all dynamically mirrored parameters
- **Notes:** Frame strings are not explicitly updated here; the main dynamic fields are thresholds and covariance constants.

##### `listen_to_camera(msg)`
- **Purpose:** Transform an incoming camera detection into the fusion frame and buffer it.
- **Called by:** camera subscription callback
- **Inputs:** `CameraDetection`
- **Returns:** none
- **Main steps:** transform with `transform_camera_to_base()`, append to `camera_detections`, warn on failure.
- **Parameters used:** `camera_frame`, `fusion_frame`
- **Notes:** Buffering is deferred; fusion itself happens in the timer.

##### `listen_to_radar(msg)`
- **Purpose:** Transform an incoming radar detection into the fusion frame and buffer it.
- **Called by:** radar subscription callback
- **Inputs:** `RadarDetection`
- **Returns:** none
- **Main steps:** transform with `transform_radar_to_base()`, append to `radar_detections`, warn on failure.
- **Parameters used:** `radar_frame`, `fusion_frame`
- **Notes:** Symmetric to `listen_to_camera()`.

##### `process_radar_detection_without_fusion(radar_detection)`
- **Purpose:** Decide whether an unmatched radar detection should be published by itself.
- **Called by:** `attempt_fusion()`
- **Inputs:** one buffered radar detection
- **Returns:** none
- **Main steps:** verify detection, check immediate-publish rule, wait for a camera partner if expected, otherwise create a radar-only fused message.
- **Parameters used:** `time_threshold`, `camera_trust_max`, `near_immediate_range`, `radar_min_confidence_percent_single`
- **Notes:** Untrusted radar detections remain in the buffer for possible fusion until they age out.

##### `process_camera_detection_without_fusion(camera_detection)`
- **Purpose:** Decide whether an unmatched camera detection should be published by itself.
- **Called by:** `attempt_fusion()`
- **Inputs:** one buffered camera detection
- **Returns:** none
- **Main steps:** same logic as the radar-only path, but using camera trust and score checks.
- **Parameters used:** `time_threshold`, `camera_trust_max`, `near_immediate_range`, `camera_min_score_single`
- **Notes:** Camera-only publication is more restrictive because camera range is less trusted.

##### `transform_camera_to_base(camera_msg)`
- **Purpose:** Transform a camera detection from `camera_frame` to `fusion_frame`.
- **Called by:** `listen_to_camera()`
- **Inputs:** `CameraDetection`
- **Returns:** transformed `CameraDetection` or `None`
- **Main steps:** wrap position in `PointStamped`, look up TF, apply transform, rewrite frame ID.
- **Parameters used:** `camera_frame`, `fusion_frame`
- **Notes:** Uses message timestamp for TF lookup.

##### `transform_radar_to_base(radar_msg)`
- **Purpose:** Transform a radar detection from `radar_frame` to `fusion_frame`.
- **Called by:** `listen_to_radar()`
- **Inputs:** `RadarDetection`
- **Returns:** transformed `RadarDetection` or `None`
- **Main steps:** same pattern as `transform_camera_to_base()`.
- **Parameters used:** `radar_frame`, `fusion_frame`
- **Notes:** Required so both sources share one geometric frame before association.

##### `_stamp_to_float(stamp)`
- **Purpose:** Convert a ROS stamp to floating-point seconds.
- **Called by:** `_newer_stamp()` and time-sorting logic
- **Inputs:** ROS time stamp
- **Returns:** `float`
- **Main steps:** `sec + nanosec * 1e-9`
- **Parameters used:** none
- **Notes:** Simple helper.

##### `_newer_stamp(a, b)`
- **Purpose:** Choose the newer of two message timestamps.
- **Called by:** `create_fused_detection()`
- **Inputs:** two ROS time stamps
- **Returns:** newer stamp
- **Main steps:** compare converted float times and return the later one.
- **Parameters used:** none
- **Notes:** Fused message timestamp is taken as the newer of the two source timestamps.

##### `verify_detection(detection, detection_type)`
- **Purpose:** Check whether a single-sensor detection is trustworthy enough to publish on its own.
- **Called by:** single-sensor processing paths
- **Inputs:** detection object and source type string
- **Returns:** `bool`
- **Main steps:**
  1. For camera, reject if outside `camera_trust_max` or below `camera_min_score_single`.
  2. For radar, reject if confidence is below `radar_min_confidence_percent_single`.
- **Parameters used:** `camera_trust_max`, `camera_min_score_single`, `radar_min_confidence_percent_single`
- **Notes:** These checks do not block fusion; they only gate single-sensor publication.

##### `get_detection_time(detection)`
- **Purpose:** Read a detection timestamp in seconds.
- **Called by:** multiple timing helpers
- **Inputs:** detection message
- **Returns:** `float`
- **Main steps:** convert header stamp to seconds.
- **Parameters used:** none
- **Notes:** Shared time helper.

##### `prune_stale(dq, time_now)`
- **Purpose:** Remove detections older than the allowed buffer age.
- **Called by:** `attempt_fusion()`
- **Inputs:** detection deque and current time
- **Returns:** none
- **Main steps:** iterate over buffered detections and remove old entries.
- **Parameters used:** `max_buffer_age`
- **Notes:** Prevents stale detections from pairing with fresh ones.

##### `temporal_match(camera_time, radar_time)`
- **Purpose:** Test whether two detections are close enough in time to be considered a candidate pair.
- **Called by:** `attempt_fusion()`
- **Inputs:** camera timestamp, radar timestamp
- **Returns:** `bool`
- **Main steps:** compare absolute time difference to `time_threshold`.
- **Parameters used:** `time_threshold`
- **Notes:** First-stage gate before spatial matching.

##### `polar_to_cart_cov(r, theta, sigma_r, sigma_theta)`
- **Purpose:** Convert polar-coordinate uncertainty into a Cartesian covariance matrix.
- **Called by:** `camera_cov_xy()`, `radar_cov_xy()`
- **Inputs:** range, bearing, range sigma, bearing sigma
- **Returns:** `2x2` covariance matrix
- **Main steps:** build the Jacobian and compute `J R J^T`.
- **Parameters used:** none directly
- **Notes:** Used for Mahalanobis gating and weighted least-squares fusion.

##### `camera_cov_xy(cam_pt)`
- **Purpose:** Build a 2D covariance model for a camera detection.
- **Called by:** `match_mahalanobis()`, `fuse_xy_wls()`
- **Inputs:** camera point in fusion frame
- **Returns:** `2x2` covariance matrix
- **Main steps:** compute range/bearing, build distance-dependent camera sigma, cap far-range sigma, convert to Cartesian covariance.
- **Parameters used:** `cam_sigma_theta_deg`, `cam_range_a`, `cam_range_b`, `cam_far_sigma_cap`, `camera_trust_max`
- **Notes:** Models camera range as worsening with distance.

##### `radar_cov_xy(rad_pt, distance)`
- **Purpose:** Build a 2D covariance model for a radar detection.
- **Called by:** `match_mahalanobis()`, `fuse_xy_wls()`
- **Inputs:** radar point, radar distance
- **Returns:** `2x2` covariance matrix
- **Main steps:** use fixed range sigma and beamwidth-derived angular sigma, then convert to Cartesian covariance.
- **Parameters used:** `radar_range_base`, `radar_beamwidth_deg`
- **Notes:** Radar model assumes solid range and coarser bearing.

##### `match_mahalanobis(camera_detection, radar_detection)`
- **Purpose:** Evaluate spatial compatibility of one camera–radar pair.
- **Called by:** `attempt_fusion()`
- **Inputs:** one camera detection and one radar detection
- **Returns:** squared Mahalanobis distance if inside gate, otherwise `-1.0`
- **Main steps:** compute XY residual, build covariance sum, invert it, compute `D^2`, compare against `chi2_threshold`.
- **Parameters used:** `chi2_threshold` plus covariance-model parameters indirectly
- **Notes:** Only 2D XY is used for spatial gating.

##### `attempt_fusion()`
- **Purpose:** Perform one fusion cycle over the current camera and radar buffers.
- **Called by:** timer
- **Inputs:** none
- **Returns:** none
- **Main steps:**
  1. Prune stale detections.
  2. If only one source is available, process detections as single-sensor candidates.
  3. Otherwise, build all temporally and spatially compatible camera–radar candidate pairs.
  4. Solve greedy one-to-one assignment based on best Mahalanobis match.
  5. Create fused detections for assigned pairs.
  6. Process leftover detections individually.
  7. Run spatial suppression with `select_targets()`.
  8. Publish the selected targets.
- **Parameters used:** timing, trust, gating, and suppression parameters
- **Notes:** Central orchestration function of the node.

##### `select_targets(detections, radius=0.5)`
- **Purpose:** Suppress duplicate detections within a local radius.
- **Called by:** `attempt_fusion()`
- **Inputs:** list of `FusedDetection`, suppression radius
- **Returns:** filtered list
- **Main steps:** sort by source priority and recency, use a grid neighborhood search, keep only the best detection inside each local radius.
- **Parameters used:** usually `selection_radius`
- **Notes:** Priority order is fused > camera > radar.

##### `is_partner_expected(source, msg)`
- **Purpose:** Decide whether a single-sensor detection should wait for a partner.
- **Called by:** single-sensor processing paths
- **Inputs:** source string and detection
- **Returns:** `bool`
- **Main steps:** for radar, expect a camera partner only inside the camera trust range; for camera, generally expect radar coverage.
- **Parameters used:** `camera_trust_max`
- **Notes:** Avoids unnecessary waiting for radar detections far beyond camera range.

##### `should_publish_immediately(source, msg)`
- **Purpose:** Decide whether a very close detection should be published without waiting.
- **Called by:** single-sensor processing paths
- **Inputs:** source string and detection
- **Returns:** `bool`
- **Main steps:** compute range and compare to `near_immediate_range`.
- **Parameters used:** `near_immediate_range`
- **Notes:** Reduces delay near the vehicle.

##### `fuse_xy_wls(cam_det, rad_det)`
- **Purpose:** Fuse camera and radar XY positions using weighted least squares in information form.
- **Called by:** `create_fused_detection()`
- **Inputs:** matched camera detection and radar detection
- **Returns:** fused 2D position vector
- **Main steps:** build source vectors, compute source covariances, invert to information matrices, combine into one weighted estimate.
- **Parameters used:** covariance-model parameters indirectly
- **Notes:** Uses radar/camera covariance models to weight each sensor contribution.

##### `create_radar_detection(radar_detection)`
- **Purpose:** Wrap a radar-only detection in a `FusedDetection` message.
- **Called by:** single-sensor radar path
- **Inputs:** one radar detection
- **Returns:** none
- **Main steps:** copy radar values into a `FusedDetection`, mark type as `radar`, append to publication list.
- **Parameters used:** none
- **Notes:** Does not publish immediately; publication is centralized at the end of `attempt_fusion()`.

##### `create_camera_detection(camera_detection)`
- **Purpose:** Wrap a camera-only detection in a `FusedDetection` message.
- **Called by:** single-sensor camera path
- **Inputs:** one camera detection
- **Returns:** none
- **Main steps:** copy camera results and position, compute planar distance, mark type as `camera`, append to publication list.
- **Parameters used:** none
- **Notes:** Uses planar XY norm for the `distance` field.

##### `create_fused_detection(camera_detection, radar_detection)`
- **Purpose:** Build a fused detection from a matched camera–radar pair.
- **Called by:** `attempt_fusion()`
- **Inputs:** matched camera and radar detections
- **Returns:** none
- **Main steps:** choose newer timestamp, copy camera classifications, fuse XY position with `fuse_xy_wls()`, keep radar Z and speed, mark type as `fused`, append to publication list.
- **Parameters used:** none directly
- **Notes:** The fused detection carries camera semantics and radar kinematics.

##### `main(args=None)`
- **Purpose:** ROS 2 entry point.
- **Called by:** console script `fusion_node`
- **Inputs:** optional ROS arguments
- **Returns:** none
- **Main steps:** initialize ROS, create node, spin, destroy node, shut down.
- **Parameters used:** none directly
- **Notes:** Standard ROS 2 Python entry point.

#### 2.1.5 Processing flow
1. Camera and radar detections arrive on their respective topics.
2. Each detection is transformed into the common fusion frame and buffered.
3. Every timer cycle, stale detections are removed.
4. Candidate camera–radar pairs are formed using time and Mahalanobis gates.
5. Best non-conflicting pairs are fused into one detection.
6. Leftover detections may be published individually if they pass single-sensor trust rules.
7. A final suppression step removes redundant nearby outputs.
8. The node publishes selected `FusedDetection` messages on `/fused_detections`.

#### 2.1.6 Notes / failure cases
- **Notes:**
  - TF availability is mandatory for both sensor streams.
  - Camera trust and score thresholds only affect single-sensor publication, not fusion eligibility.
  - This is decision-level fusion: it works on object-level detections, not raw sensor data.
- **Failure cases:**
  - Missing or delayed TF transforms cause detections to be dropped.
  - Poor calibration between radar, camera, and base frame degrades Mahalanobis matching and fusion quality.
  - Sparse or asynchronous detections can reduce fusion frequency and lead to more single-sensor outputs.

### 2.2 `ego_kf_tracker`

#### 2.2.1 Overview
- **Node name:** `ego_kf_tracker`
- **Executable:** `kf_tracker`
- **Source file:** `src/sensor_fusion/sensor_fusion/kf_tracker.py`
- **Purpose:** Track fused and single-sensor detections over time in a world frame using a constant-velocity Kalman filter.

This node receives `FusedDetection` measurements from the fusion node and ego-motion updates from `/ego_motion`. It converts detections to a world frame using a buffered ego pose estimate, associates measurements to tracks with greedy Mahalanobis gating, maintains tracks through missed detections, and publishes confirmed tracked objects.

#### 2.2.2 Topics and interfaces

##### Subscribed topics
| Topic | Message type | Callback / handler | Purpose | Notes |
|---|---|---|---|---|
| `/fused_detections` | `tractor_safety_system_interfaces/msg/FusedDetection` | `on_detection()` | Receive fused or single-sensor target measurements | Measurements arrive in `base_link` and are transformed to world coordinates |
| `/ego_motion` | `geometry_msgs/msg/TwistWithCovarianceStamped` | `on_ego_odom()` | Receive vehicle forward speed, yaw rate, and motion covariance | Used for ego-pose integration and adaptive process noise |

##### Published topics
| Topic | Message type | Publisher / function | Purpose | Notes |
|---|---|---|---|---|
| `/tracked_detections` | `tractor_safety_system_interfaces/msg/FusedDetectionArray` | `pub_tracks` via `publish_tracks()` | Publish confirmed tracks as tracked detections | Output positions are converted back to `base_link` |

##### Other interfaces (optional)
- **Services:** none
- **Actions:** none
- **TF frames used or produced:** No TF lookup; the node maintains its own world/ego conversion using integrated ego pose
- **Timers:** `on_timer()` runs at `update_rate`

#### 2.2.3 Parameters
Document every parameter declared or used by this node.

| Parameter | Type | Default | Units | Used in | Description |
|---|---|---:|---|---|---|
| `wheelbase` | `float` | `2.5` | m | `__init__()` | Stored vehicle parameter; not currently used in the tracking logic |
| `steer_unit_per_rad` | `float` | `180/pi` | units/rad | `__init__()` | Stored scaling parameter; not currently used in active tracking logic |
| `speed_unit_per_mps` | `float` | `1.0` | units/(m/s) | `__init__()` | Stored scaling parameter; not currently used in active tracking logic |
| `control_latency` | `float` | `0.10` | s | `__init__()` | Stored latency value; not currently used in active tracking logic |
| `max_yaw_rate` | `float` | `1.0` | rad/s | `_process_noise_scale()` | Reference yaw rate used when scaling process noise |
| `sigma_accel` | `float` | `3.0` | m/s² | `_predict_track_to_time()` | Baseline acceleration noise for the constant-velocity process model |
| `R_meas_xy` | `float` | `0.3` | m | `__init__()` | Default isotropic measurement std fallback |
| `gate_chi2` | `float` | `10.597` | - | `associate_and_update()` | Mahalanobis gate for measurement-to-track association |
| `spawn_hits` | `int` | `3` | hits | `publish_tracks()`, `maintain_tracks()` | Hits required before a track is confirmed |
| `spawn_hit_ratio` | `float` | `0.75` | ratio | `maintain_tracks()` | Minimum hit-to-age ratio for keeping unconfirmed tracks |
| `max_miss` | `int` | `10` | cycles | `maintain_tracks()` | Maximum consecutive misses before deleting a confirmed track |
| `init_speed_std` | `float` | `3.0` | m/s | `associate_and_update()` | Initial velocity uncertainty for new tracks |
| `update_rate` | `float` | `20.0` | Hz | `__init__()` | Timer frequency for association, maintenance, and publishing |
| `enable_ego_drag` | `bool` | `True` | - | `_process_noise_scale()` | Enables adaptive process-noise inflation based on ego motion |
| `measurement_latency` | `float` | `0.0` | s | `on_detection()` | Time offset subtracted from detection timestamps before pose lookup |
| `pose_history_seconds` | `float` | `10.0` | s | `EgoPoseBuffer` setup | Duration of ego-pose history kept for interpolation |
| `R_meas_fused_xy` | `float` | `0.3` | m | `get_measurement_covariance()` fallback path | Fallback fused-measurement std when covariance models are disabled |
| `R_meas_camera_xy` | `float` | `0.6` | m | `get_measurement_covariance()` fallback path | Fallback camera-measurement std when covariance models are disabled |
| `R_meas_radar_xy` | `float` | `0.4` | m | `get_measurement_covariance()` fallback path | Fallback radar-measurement std when covariance models are disabled |
| `use_measurement_covariance_models` | `bool` | `True` | - | `get_measurement_covariance()` | Enables source-specific covariance models instead of fixed isotropic stds |
| `cam_sigma_theta_deg` | `float` | `0.5` | deg | `_camera_cov_xy_ego()` | Camera angular uncertainty |
| `cam_range_a` | `float` | `0.03` | m/m² | `_camera_cov_xy_ego()` | Quadratic camera range-uncertainty coefficient |
| `cam_range_b` | `float` | `0.15` | m | `_camera_cov_xy_ego()` | Offset camera range-uncertainty coefficient |
| `cam_far_sigma_cap` | `float` | `20.0` | m | `_camera_cov_xy_ego()` | Cap on large-distance camera range sigma |
| `camera_trust_max` | `float` | `12.0` | m | `_camera_cov_xy_ego()` | Range at which camera sigma capping logic changes |
| `radar_range_base` | `float` | `0.8` | m | `_radar_cov_xy_ego()` | Radar range sigma |
| `radar_beamwidth_deg` | `float` | `3.0` | deg | `_radar_cov_xy_ego()` | Radar angular uncertainty |
| `use_twist_covariance` | `bool` | `True` | - | `_process_noise_scale()` | Enables process-noise scaling from incoming twist covariance |
| `q_yaw_rate_scale` | `float` | `2.0` | gain | `_process_noise_scale()` | Gain applied to normalized yaw-rate contribution |
| `q_twist_cov_scale` | `float` | `2.0` | gain | `_process_noise_scale()` | Gain applied to normalized twist-covariance contribution |
| `ref_yaw_rate_std` | `float` | `0.3` | rad/s | `_process_noise_scale()` | Reference yaw-rate std for covariance normalization |
| `ref_speed_std` | `float` | `0.5` | m/s | `_process_noise_scale()` | Reference speed std for covariance normalization |

#### 2.2.4 Functions
Document every function or method in this node.

##### Helper class `EgoPoseBuffer`
- **Purpose:** Maintain a short time history of ego poses for interpolation and slight extrapolation.
- **Used by:** `EgoKFTracker`
- **Key methods:**
  - `__init__(history_seconds)`: initialize pose state and history buffer.
  - `_wrap_angle(a)`: wrap an angle into `[-pi, pi)`.
  - `_angle_lerp(a0, a1, w)`: interpolate yaw safely across angle wrap.
  - `_integrate(v, yaw_rate, dt)`: integrate body-frame motion into world pose.
  - `ingest_twist(t, v, yaw_rate)`: append a new timestamped ego pose sample.
  - `pose_at(t)`: return interpolated or slightly extrapolated pose at arbitrary time `t`.
- **Notes:** Lets the tracker use the ego pose valid at the measurement timestamp instead of the current timer time.

##### Helper class `Track`
- **Purpose:** Store the state and bookkeeping for one tracked object.
- **Used by:** `EgoKFTracker`
- **Key members:** state vector `x`, covariance `P`, `age`, `hits`, `miss`, `id`, last timestamps, update flag.
- **Key methods:**
  - `__init__(x0, P0, stamp, t_sec, tid)`: initialize one new track.
- **Notes:** Lightweight container; no behavior beyond initialization.

##### `__init__()`
- **Purpose:** Initialize parameters, subscriptions, publisher, timer, ego buffer, and track storage.
- **Called by:** `main()`
- **Inputs:** none
- **Returns:** none
- **Main steps:**
  1. Declare and read tracking, covariance, and ego-motion parameters.
  2. Initialize ego pose state and pose buffer.
  3. Create detection and ego-motion subscriptions.
  4. Create tracked-detections publisher and timer.
  5. Initialize track list and latest-motion cache.
- **Parameters used:** all declared parameters
- **Notes:** Some vehicle-control parameters are declared but not actively used in the current tracker logic.

##### `on_detection(msg)`
- **Purpose:** Receive one detection, convert it to world coordinates, and buffer it for the next timer cycle.
- **Called by:** `/fused_detections` subscription
- **Inputs:** `FusedDetection`
- **Returns:** none
- **Main steps:**
  1. Convert detection stamp to seconds and apply measurement latency correction.
  2. Query ego pose at the corrected time.
  3. Transform detection position from ego frame to world frame.
  4. Build source-specific measurement covariance in ego frame.
  5. Rotate covariance into world frame.
  6. Append the measurement tuple to the measurement buffer.
- **Parameters used:** `measurement_latency` and covariance-related parameters
- **Notes:** No association is performed here; callbacks stay lightweight.

##### `on_ego_odom(twist_msg)`
- **Purpose:** Update ego pose history and latest ego-motion uncertainty.
- **Called by:** `/ego_motion` subscription
- **Inputs:** `TwistWithCovarianceStamped`
- **Returns:** none
- **Main steps:** extract speed and yaw rate, extract covariance terms, integrate the ego pose history, store latest motion and covariance.
- **Parameters used:** pose-history settings indirectly
- **Notes:** Uses twist covariance indices `(0,0)` for `vx` and `(5,5)` for yaw rate.

##### `get_ego_pose_at(t)`
- **Purpose:** Return ego pose at an arbitrary time.
- **Called by:** `on_detection()`
- **Inputs:** time in seconds
- **Returns:** `(x, y, yaw)` tuple
- **Main steps:** delegate to `EgoPoseBuffer.pose_at()`.
- **Parameters used:** none directly
- **Notes:** Thin wrapper.

##### `publish_tracks()`
- **Purpose:** Publish confirmed tracks in `base_link` coordinates.
- **Called by:** `on_timer()`
- **Inputs:** none
- **Returns:** none
- **Main steps:**
  1. Build a `FusedDetectionArray`.
  2. Skip unconfirmed tracks.
  3. Predict each track to current time.
  4. Convert predicted world position and velocity back to ego frame.
  5. Populate and publish the tracked-detection message array.
- **Parameters used:** `spawn_hits`
- **Notes:** Publish-time prediction does not mutate the stored track state.

##### `_wrap_angle(a)`
- **Purpose:** Wrap an angle into `[-pi, pi)`.
- **Called by:** currently a local static helper for consistency
- **Inputs:** angle
- **Returns:** wrapped angle
- **Main steps:** modular arithmetic.
- **Parameters used:** none
- **Notes:** Separate from the `EgoPoseBuffer` helper of the same name.

##### `_polar_to_cart_cov(r, theta, sigma_r, sigma_theta)`
- **Purpose:** Convert polar uncertainty into Cartesian covariance.
- **Called by:** `_camera_cov_xy_ego()`, `_radar_cov_xy_ego()`
- **Inputs:** range, bearing, range sigma, bearing sigma
- **Returns:** `2x2` covariance matrix
- **Main steps:** construct Jacobian and compute `J R J^T`.
- **Parameters used:** none directly
- **Notes:** Forms the basis of source-specific measurement uncertainty.

##### `_camera_cov_xy_ego(p_ego)`
- **Purpose:** Build camera measurement covariance in ego frame.
- **Called by:** `get_measurement_covariance()`
- **Inputs:** 2D ego-frame position
- **Returns:** `2x2` covariance matrix
- **Main steps:** derive range and bearing, compute range sigma from quadratic model, cap far-range sigma, convert to Cartesian covariance.
- **Parameters used:** `cam_sigma_theta_deg`, `cam_range_a`, `cam_range_b`, `cam_far_sigma_cap`, `camera_trust_max`
- **Notes:** Models stereo range uncertainty growing with distance.

##### `_radar_cov_xy_ego(p_ego)`
- **Purpose:** Build radar measurement covariance in ego frame.
- **Called by:** `get_measurement_covariance()`
- **Inputs:** 2D ego-frame position
- **Returns:** `2x2` covariance matrix
- **Main steps:** use fixed radar range sigma and beamwidth-based angular sigma, then convert to Cartesian covariance.
- **Parameters used:** `radar_range_base`, `radar_beamwidth_deg`
- **Notes:** Simpler than the camera model.

##### `get_measurement_covariance(source, position_ego)`
- **Purpose:** Return the measurement covariance for a detection source at a given ego-frame position.
- **Called by:** `on_detection()`
- **Inputs:** source string and position
- **Returns:** `2x2` covariance matrix
- **Main steps:** either return fixed fallback covariance or build source-specific covariance models; for fused detections combine camera and radar covariances in information form.
- **Parameters used:** fixed-std parameters and covariance-model parameters
- **Notes:** For fused detections the combined covariance is `inv(inv(Rc)+inv(Rr))`.

##### `_rotate_covariance(R_ego, yaw)`
- **Purpose:** Rotate an ego-frame covariance matrix into world coordinates.
- **Called by:** `on_detection()`
- **Inputs:** ego-frame covariance and yaw
- **Returns:** rotated covariance
- **Main steps:** apply `Rot * R * Rot^T`.
- **Parameters used:** none
- **Notes:** Needed because the filter state lives in the world frame.

##### `ego_to_world_pos(p_ego, ego_pose=None)`
- **Purpose:** Convert a 2D point from `base_link` to world coordinates.
- **Called by:** `on_detection()`
- **Inputs:** ego-frame point and optional explicit ego pose
- **Returns:** world-frame point
- **Main steps:** rotate by yaw and add ego translation.
- **Parameters used:** none
- **Notes:** Uses the current ego pose if no explicit pose is supplied.

##### `world_to_ego_pos(p_world, ego_pose=None)`
- **Purpose:** Convert a 2D point from world coordinates back to `base_link`.
- **Called by:** `publish_tracks()`
- **Inputs:** world-frame point and optional pose
- **Returns:** ego-frame point
- **Main steps:** subtract translation and rotate by transpose of the yaw matrix.
- **Parameters used:** none
- **Notes:** Inverse of `ego_to_world_pos()`.

##### `ego_to_world_vel(v_ego)`
- **Purpose:** Rotate a 2D velocity vector from ego frame into world frame.
- **Called by:** currently helper only
- **Inputs:** ego-frame velocity
- **Returns:** world-frame velocity
- **Main steps:** apply yaw rotation.
- **Parameters used:** none
- **Notes:** Translation is not applied because this is a vector, not a point.

##### `world_to_ego_vel(v_world)`
- **Purpose:** Rotate a 2D velocity vector from world frame back to ego frame.
- **Called by:** `publish_tracks()`
- **Inputs:** world-frame velocity
- **Returns:** ego-frame velocity
- **Main steps:** apply transpose of yaw rotation.
- **Parameters used:** none
- **Notes:** Used for publishing speed in the vehicle frame.

##### `on_timer()`
- **Purpose:** Run one tracker cycle.
- **Called by:** timer
- **Inputs:** none
- **Returns:** none
- **Main steps:**
  1. Reset per-cycle update flags.
  2. Associate and update tracks with buffered measurements.
  3. Increment miss counters for tracks not updated this cycle.
  4. Maintain and age tracks.
  5. Publish confirmed tracks.
- **Parameters used:** indirectly all tracking parameters
- **Notes:** Ego integration does not happen here; it happens in `on_ego_odom()`.

##### `_process_noise_scale()`
- **Purpose:** Compute a multiplicative factor that inflates process noise during turning or uncertain ego motion.
- **Called by:** `_predict_track_to_time()`
- **Inputs:** none
- **Returns:** scale factor
- **Main steps:** normalize yaw rate, optionally normalize twist covariance terms, combine them multiplicatively.
- **Parameters used:** `enable_ego_drag`, `max_yaw_rate`, `use_twist_covariance`, `q_yaw_rate_scale`, `q_twist_cov_scale`, `ref_yaw_rate_std`, `ref_speed_std`
- **Notes:** Keeps process noise small in calm motion and larger during turns or uncertain ego motion.

##### `_predict_track_to_time(tr, t_target)`
- **Purpose:** Predict one track to an arbitrary time without mutating it.
- **Called by:** `publish_tracks()`, `associate_and_update()`, `kf_update_track()`
- **Inputs:** track and target time
- **Returns:** predicted state and covariance
- **Main steps:** compute `dt`, build constant-velocity transition matrix and process-noise matrix, apply the Kalman prediction equations.
- **Parameters used:** `sigma_accel` and process-noise scaling parameters
- **Notes:** Clips `dt` to 5 seconds.

##### `kf_update_track(tr, position, Rm, H, stamp, t_meas)`
- **Purpose:** Apply one Kalman update to a matched track.
- **Called by:** `associate_and_update()`
- **Inputs:** track, measurement position, covariance, measurement matrix, stamp, measurement time
- **Returns:** none
- **Main steps:** predict to measurement time, compute innovation and innovation covariance, compute Kalman gain, update state and covariance with Joseph form, update bookkeeping.
- **Parameters used:** none directly
- **Notes:** Uses pseudoinverse fallback if matrix inversion fails.

##### `associate_and_update(meas_list)`
- **Purpose:** Associate buffered measurements to tracks and either update tracks or spawn new ones.
- **Called by:** `on_timer()`
- **Inputs:** measurement list
- **Returns:** none
- **Main steps:**
  1. Sort measurements by source priority (`fused`, `camera`, `radar`) and recency.
  2. For each measurement, search for the best gated track by Mahalanobis distance.
  3. Update the best track if found.
  4. Otherwise suppress duplicate spawns near already-existing tracks.
  5. Spawn a new track if the measurement is not a duplicate.
- **Parameters used:** `gate_chi2`, `init_speed_std`
- **Notes:** Uses greedy nearest-neighbor association, not global multi-target assignment.

##### `maintain_tracks()`
- **Purpose:** Remove tracks that fail confirmation or retention rules.
- **Called by:** `on_timer()`
- **Inputs:** none
- **Returns:** none
- **Main steps:** keep confirmed tracks with `miss <= max_miss`, keep young unconfirmed tracks, otherwise drop them.
- **Parameters used:** `spawn_hits`, `spawn_hit_ratio`, `max_miss`
- **Notes:** Track age is updated outside this function.

##### `main()`
- **Purpose:** ROS 2 entry point.
- **Called by:** console script `kf_tracker`
- **Inputs:** none
- **Returns:** none
- **Main steps:** initialize ROS, create node, spin, destroy node, shut down.
- **Parameters used:** none directly
- **Notes:** Standard ROS 2 Python entry point.

#### 2.2.5 Processing flow
1. The node receives detections from `/fused_detections` and ego motion from `/ego_motion`.
2. Ego motion is integrated into a short pose history buffer.
3. Each detection is time-aligned using the ego pose valid at measurement time.
4. The detection is transformed into world coordinates and stored with a world-frame covariance.
5. Every timer cycle, buffered measurements are associated to tracks using Mahalanobis gating.
6. Existing tracks are updated; unmatched measurements may spawn new tracks.
7. Tracks with too many misses are removed, and surviving tracks age by one cycle.
8. Confirmed tracks are predicted to the current time and published in `base_link` on `/tracked_detections`.

#### 2.2.6 Notes / failure cases
- **Notes:**
  - The tracker is intended for single-target or low-density multi-target scenes.
  - It tracks in a world frame to avoid interpreting ego motion as target motion.
  - Source-dependent covariance modeling is central to both gating and update weighting.
- **Failure cases:**
  - If `/ego_motion` is missing or badly timed, world-frame transformation and process-noise adaptation degrade.
  - Long TF-independent ego-motion integration can drift over time because the pose buffer is dead-reckoned from twist only.
  - Dense scenes with crossing targets can cause association errors because the tracker uses greedy nearest-neighbor assignment.

---

## 3. Tests in `/test`

### Test overview
This package contains functional tests for the fusion node, parameter-update tests, trust-limit tests, and single-sensor publication behavior, plus standard lint/copyright checks.

Notably, the current `/test` directory focuses on `fusion_node`; there are no dedicated `kf_tracker` behavior tests in this package at present.

### Test files
| Test file | Target | Type | Purpose | Notes |
|---|---|---|---|---|
| `test_fusion.py` | `fusion_node` | unit | Verifies that matching radar and camera detections are fused and published correctly | Uses synthetic messages |
| `test_fusion_parameters.py` | `fusion_node` | unit | Verifies runtime parameter updates for core fusion thresholds | Tests `on_set_parameters()` |
| `test_limits.py` | `fusion_node` | unit | Verifies that camera detections beyond the trust range are not published individually | Focuses on single-sensor trust logic |
| `test_single_sensor_detection.py` | `fusion_node` | unit | Verifies correct publication behavior for trusted and untrusted single-sensor detections | Uses synthetic camera and radar detections |
| `test_flake8.py` | package | lint | Verifies style compliance | Standard ROS packaging test |
| `test_pep257.py` | package | lint | Verifies docstring compliance | Standard ROS packaging test |
| `test_copyright.py` | package | lint | Verifies copyright headers | Standard ROS packaging test |

### Running the tests
- **Command(s):** `colcon test --packages-select sensor_fusion` or `pytest src/sensor_fusion/test`
- **Prerequisites:** ROS 2 environment sourced; required message packages and TF dependencies installed
- **Expected environment:** Functional tests use synthetic messages and mocked publication, so no live sensors are required.

### Notes on test coverage
- Fusion pairing, fusion output, trust-limit gating, and selected runtime parameter updates are covered.
- The Kalman tracker node currently has no dedicated unit test coverage in `/test`.
- TF lookup success/failure paths in the fusion node are not comprehensively exercised by the current tests.
