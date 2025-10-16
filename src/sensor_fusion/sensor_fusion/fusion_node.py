# Copyright 2025 Eemil Kulmala, University of Oulu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from collections import deque

from geometry_msgs.msg import Point
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from tractor_safety_system_interfaces.msg import CameraDetection, FusedDetection, RadarDetection


class FusionNode(Node):

    def __init__(self):
        super().__init__('fusion_node')
        self.camera_subscriber = self.create_subscription(
            CameraDetection,
            '/camera_detections',
            self.listen_to_camera,
            10)
        self.radar_subscriber = self.create_subscription(
            RadarDetection,
            '/radar_detections',
            self.listen_to_radar,
            10)
        self.publisher_ = self.create_publisher(FusedDetection, '/fused_detections', 10)

        self.timer = self.create_timer(0.05, self.attempt_fusion)

        # Initialize detection deques
        self.camera_detections = deque(maxlen=200)  # Limit to 200 recent detections
        self.radar_detections = deque(maxlen=200)   # Limit to 200 recent detections
        self.targets_to_publish = []

        # Declare parameters with default values
        # --- Threshold values ---
        self.declare_parameter('time_threshold', 0.1)        # Default value 0.1 seconds
        self.declare_parameter('camera_trust_max', 12.0)     # Default value 12 meters
        self.declare_parameter('chi2_threshold', 5.99)       # 95% in 2D
        self.declare_parameter('max_buffer_age', 0.75)       # s, drop stale messages
        self.declare_parameter('near_immediate_range', 3.0)  # immediate publish distance
        # --- Transformation parameters from camera to radar coordinates ---
        self.declare_parameter('rotation_matrix', [1.0, 0.0, 0.0,
                                                   0.0, 1.0, 0.0,
                                                   0.0, 0.0, 1.0])
        self.declare_parameter('translation_vector', [0.0, 0.0, 0.0])
        # --- Covariance parameters for sensors ---
        self.declare_parameter('cam_sigma_theta_deg', 0.5)     # bearing std in degrees
        self.declare_parameter('cam_range_a', 0.05)            # sigma_r = a*r^2 + b
        self.declare_parameter('cam_range_b', 0.20)
        self.declare_parameter('cam_far_sigma_cap', 30.0)      # cap (m) for far range
        self.declare_parameter('radar_range_base', 0.3)        # m
        self.declare_parameter('radar_range_slope', 0.01)      # m per meter
        self.declare_parameter('radar_beamwidth_deg', 2.0)     # deg (≈ azimuth std)

        # Retrieve parameter values
        self.time_threshold = self.get_parameter('time_threshold').value
        self.camera_trust_max = self.get_parameter('camera_trust_max').value
        self.chi2_threshold = self.get_parameter('chi2_threshold').value
        self.max_buffer_age = self.get_parameter('max_buffer_age').value
        self.near_immediate_range = self.get_parameter('near_immediate_range').value
        rotation_matrix_param = self.get_parameter('rotation_matrix').value
        translation_vector_param = self.get_parameter('translation_vector').value
        self.R = np.array(rotation_matrix_param).reshape(3, 3)
        self.T = np.array(translation_vector_param)
        self.cam_sigma_theta_deg = self.get_parameter('cam_sigma_theta_deg').value
        self.cam_range_a = self.get_parameter('cam_range_a').value
        self.cam_range_b = self.get_parameter('cam_range_b').value
        self.cam_far_sigma_cap = self.get_parameter('cam_far_sigma_cap').value
        self.radar_range_base = self.get_parameter('radar_range_base').value
        self.radar_range_slope = self.get_parameter('radar_range_slope').value
        self.radar_beamwidth_deg = self.get_parameter('radar_beamwidth_deg').value

        # Subscribe to parameter updates
        self.add_on_set_parameters_callback(self.on_set_parameters)

    def on_set_parameters(self, params):
        """Set parameters their new values."""
        for param in params:
            if param.name == 'time_threshold':
                self.time_threshold = param.value
            elif param.name == 'camera_trust_max':
                self.camera_trust_max = param.value
            elif param.name == 'chi2_threshold':
                self.chi2_threshold = param.value
            elif param.name == 'max_buffer_age':
                self.max_buffer_age = param.value
            elif param.name == 'near_immediate_range':
                self.near_immediate_range = param.value
            elif param.name == 'rotation_matrix':
                rotation_matrix_param = param.value
                self.R = np.array(rotation_matrix_param).reshape(3, 3)
            elif param.name == 'translation_vector':
                translation_vector_param = param.value
                self.T = np.array(translation_vector_param)
            elif param.name == 'cam_sigma_theta_deg':
                self.cam_sigma_theta_deg = param.value
            elif param.name == 'cam_range_a':
                self.cam_range_a = param.value
            elif param.name == 'cam_range_b':
                self.cam_range_b = param.value
            elif param.name == 'cam_far_sigma_cap':
                self.cam_far_sigma_cap = param.value
            elif param.name == 'radar_range_base':
                self.radar_range_base = param.value
            elif param.name == 'radar_range_slope':
                self.radar_range_slope = param.value
            elif param.name == 'radar_beamwidth_deg':
                self.radar_beamwidth_deg = param.value
        return SetParametersResult(successful=True)

    def listen_to_camera(self, camera_msg):
        camera_msg.position = self.transform_camera_to_radar(camera_msg.position)
        self.camera_detections.append(camera_msg)

    def listen_to_radar(self, radar_msg):
        self.radar_detections.append(radar_msg)

    def process_radar_detection_without_fusion(self, radar_detection):
        """Process radar detections independently if no camera detections exist."""
        current_time = self.get_clock().now().nanoseconds * 1e-9
        detection_time = self.get_detection_time(radar_detection)
        if self.verify_detection(radar_detection, 'radar'):
            if self.should_publish_immediately('radar', radar_detection):
                self.publish_radar_detection(radar_detection)
                self.radar_detections.remove(radar_detection)
            elif self.is_partner_expected('radar', radar_detection):
                # Publish single detection if no partner is found within time threshold
                if current_time - detection_time >= self.time_threshold:
                    self.publish_radar_detection(radar_detection)
                    self.radar_detections.remove(radar_detection)
            else:
                self.publish_radar_detection(radar_detection)
                self.radar_detections.remove(radar_detection)

    def process_camera_detection_without_fusion(self, camera_detection):
        """Process camera detections independently if no radar detections exist."""
        current_time = self.get_clock().now().nanoseconds * 1e-9
        detection_time = self.get_detection_time(camera_detection)
        if self.verify_detection(camera_detection, 'camera'):
            if self.should_publish_immediately('camera', camera_detection):
                self.publish_camera_detection(camera_detection)
                self.camera_detections.remove(camera_detection)
            elif self.is_partner_expected('camera', camera_detection):
                # Publish single detection if no partner is found within time threshold
                if current_time - detection_time >= self.time_threshold:
                    self.publish_camera_detection(camera_detection)
                    self.camera_detections.remove(camera_detection)
            else:
                self.publish_camera_detection(camera_detection)
                self.camera_detections.remove(camera_detection)

    def transform_camera_to_radar(self, camera_point):
        """Transform camera coordinates to radar coordinates."""
        # Re-map to radar coordinate frame: [z, -x, -y]
        camera_point = Point(x=camera_point.z,
                             y=-camera_point.x,
                             z=-camera_point.y)
        # Convert to homogeneous coordinates
        camera_point = np.array([camera_point.x,
                                camera_point.y,
                                camera_point.z, 1])
        # Apply transformation
        radar_coordinates = np.dot(np.hstack(
            (self.R, self.T.reshape(-1, 1))), camera_point)
        # Convert back to Cartesian coordinates and return as a Point()
        return Point(x=radar_coordinates[0],
                     y=radar_coordinates[1],
                     z=radar_coordinates[2])

    def verify_detection(self, detection, detection_type):
        """Verify detection validity."""
        if detection_type == 'camera':
            distance = np.linalg.norm([detection.position.x,
                                       detection.position.y,
                                       detection.position.z])
            if distance < self.camera_trust_max:
                return True
        elif detection_type == 'radar':
            return True
        return False

    def get_detection_time(self, detection):
        """Extract the detection timestamp from a ROS message."""
        return detection.header.stamp.sec + float(detection.header.stamp.nanosec * 1e-9)

    def prune_stale(self, dq: deque, time_now: float):
        for msg in list(dq):
            if self.get_detection_time(msg) < time_now - self.max_buffer_age:
                dq.remove(msg)

    def temporal_match(self, camera_time, radar_time):
        """Return the True if a match is found, False if not."""
        if abs((camera_time - radar_time)) < self.time_threshold:  # Match!
            return True
        return False  # No match found

    def polar_to_cart_cov(self, r, theta, sigma_r, sigma_theta):
        """2x2 Cartesian covariance from polar stds at (r,theta)."""
        c, s = float(np.cos(theta)), float(np.sin(theta))
        J = np.array([[c, -r * s],
                      [s,  r * c]], dtype=float)
        Rpolar = np.diag([sigma_r ** 2, sigma_theta ** 2])
        return J @ Rpolar @ J.T

    def camera_cov_xy(self, cam_pt_in_radar_frame: Point):
        """
        Generate camera covariance.

        - good bearing (sigma_theta ~ 0.5 deg),
        - range uncertainty grows with distance,
        - capped far away but still finite so long-range detections can pass with small weight.
        """
        x, y = float(cam_pt_in_radar_frame.x), float(cam_pt_in_radar_frame.y)
        r = float(np.hypot(x, y))
        theta = float(np.arctan2(y, x))
        sigma_theta = np.deg2rad(float(self.cam_sigma_theta_deg))
        sigma_r = float(self.cam_range_a) * (r ** 2) + float(self.cam_range_b)
        if r > self.camera_trust_max:
            sigma_r = min(sigma_r, float(self.cam_far_sigma_cap))
        return self.polar_to_cart_cov(r, theta, sigma_r, sigma_theta)

    def radar_cov_xy(self, rad_pt_in_radar_frame: Point, reported_distance: float):
        """
        Generate radar covariance.

        - solid range accuracy degrading mildly with distance,
        - moderate beamwidth (~2 deg) as bearing std.
        """
        x, y = float(rad_pt_in_radar_frame.x), float(rad_pt_in_radar_frame.y)
        r_geom = float(np.hypot(x, y))
        r = float(reported_distance) if reported_distance > 0.0 else r_geom
        theta = float(np.arctan2(y, x))
        sigma_r = float(self.radar_range_base) + float(self.radar_range_slope) * r
        sigma_theta = np.deg2rad(float(self.radar_beamwidth_deg))
        return self.polar_to_cart_cov(r, theta, sigma_r, sigma_theta)

    def match_mahalanobis(self, camera_detection, radar_detection):
        """
        Perform spatial matching using Mahalanobis distance.

        Return D^2 if in gate, else -1.
        """
        # Innovation vector in XY (radar frame)
        d = np.array([camera_detection.position.x - radar_detection.position.x,
                      camera_detection.position.y - radar_detection.position.y], dtype=float)

        # Cartesian covariances
        Rc = self.camera_cov_xy(camera_detection.position)
        Rr = self.radar_cov_xy(radar_detection.position, radar_detection.distance)

        # Innovation covariance with independent sensor covariances
        S = Rc + Rr

        # Mahalanobis D^2 = d^T S^{-1} d
        try:
            Sinv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            Sinv = np.linalg.pinv(S)

        D2 = float(d.T @ Sinv @ d)

        # Gate with chi-square threshold
        if D2 < self.chi2_threshold:
            # self.get_logger().info(f'D2={D2:.3f}'
            #                       f'|d|={np.linalg.norm(d):.3f} '
            #                       f'log|S|={np.linalg.slogdet(S)[1]:.3f}')
            return D2
        return -1.0  # No match found

    def attempt_fusion(self):
        """Match radar and camera detections, and perform fusion if a match is found."""
        time_now = self.get_clock().now().nanoseconds * 1e-9
        # Drop old messages from buffers
        self.prune_stale(self.camera_detections, time_now)
        self.prune_stale(self.radar_detections, time_now)

        # If no camera detections, process radar detections without fusion
        if not self.camera_detections:
            for radar_detection in list(self.radar_detections):
                self.process_radar_detection_without_fusion(radar_detection)

        # If no radar detections, process camera detections without fusion
        elif not self.radar_detections:
            for camera_detection in list(self.camera_detections):
                self.process_camera_detection_without_fusion(camera_detection)

        else:
            candidate_pairs = []
            # Form all possible candidate pairs
            for cam_idx, camera_detection in enumerate(self.camera_detections):
                camera_time = self.get_detection_time(camera_detection)
                for rad_idx, radar_detection in enumerate(self.radar_detections):
                    radar_time = self.get_detection_time(radar_detection)
                    # Temporal matching
                    if not self.temporal_match(camera_time, radar_time):
                        continue  # No match found, continue to next detection

                    # Spatial matching
                    d2 = self.match_mahalanobis(camera_detection, radar_detection)
                    if d2 >= 0.0:
                        candidate_pairs.append((d2,
                                                cam_idx,
                                                rad_idx,
                                                abs(camera_time - radar_time)
                                                ))

            # Greedy global 1-to-1 assignment (ordered by best distance, then closest in time)
            candidate_pairs.sort(key=lambda item: (item[0], item[3]))
            used_cam = set()
            used_rad = set()

            for d2, cam_idx, rad_idx, _ in candidate_pairs:
                if cam_idx in used_cam or rad_idx in used_rad:
                    continue
                cam_msg = self.camera_detections[cam_idx]
                rad_msg = self.radar_detections[rad_idx]
                self.publish_fused_detection(cam_msg, rad_msg)
                used_cam.add(cam_idx)
                used_rad.add(rad_idx)

            # Remove consumed fused messages (delete from back to preserve indices)
            for idx in sorted(used_cam, reverse=True):
                del self.camera_detections[idx]
            for idx in sorted(used_rad, reverse=True):
                del self.radar_detections[idx]

            # Process the remaining detections individually:
            for camera_detection in list(self.camera_detections):
                self.process_camera_detection_without_fusion(camera_detection)
            for radar_detection in list(self.radar_detections):
                self.process_radar_detection_without_fusion(radar_detection)

        self.targets_to_publish = self.select_targets(self.targets_to_publish, radius=1.0)
        for target in self.targets_to_publish:
            self.publisher_.publish(target)
        self.targets_to_publish = []

    def select_targets(self, detections, radius=1.0):
        """
        Select the best detection for each target.

        detections : List[FusedDetection]
        radius   : suppression radius (meters); keep at most one per radius neighborhood.

        Returns: List[FusedDetection]
        """
        if not detections:
            return []

        # Sort by source priority (fused > camera > radar) and recency (newest first)
        SOURCE_PRIORITY = {'fused': 0, 'camera': 1, 'radar': 2}

        def stamp_seconds(det):
            return det.header.stamp.sec + float(det.header.stamp.nanosec * 1e-9)

        def priority_key(det):
            src = (det.detection_type or '').strip().casefold()
            return (SOURCE_PRIORITY.get(src, 99), -stamp_seconds(det))

        detections.sort(key=priority_key)

        cell_size = radius
        grid = {}
        kept = []

        def grid_cell_key(x: float, y: float):
            return (int(np.floor(x / cell_size)), int(np.floor(y / cell_size)))

        for det in detections:
            x = det.position.x
            y = det.position.y
            cell_x, cell_y = grid_cell_key(x, y)
            is_within_radius = False

            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    neighbor_key = (cell_x + dx, cell_y + dy)
                    for idx in grid.get(neighbor_key, []):
                        neighbor_det = kept[idx]
                        dxm = x - float(neighbor_det.position.x)
                        dym = y - float(neighbor_det.position.y)
                        if (dxm**2 + dym**2) < radius**2:
                            is_within_radius = True
                            break
                    if is_within_radius:
                        break
                if is_within_radius:
                    break
            if not is_within_radius:
                kept.append(det)
                grid.setdefault((cell_x, cell_y), []).append(len(kept) - 1)
        return kept

    def is_partner_expected(self, source: str, msg):
        """
        Return True if a partner detection is expected, False if not.

          - For radar detections: if target is within camera
            range, expect camera partner.
          - For camera detections: generally expect radar partner.
        """
        if source == 'radar':
            return msg.distance <= (self.camera_trust_max)
        else:
            # Camera is short-range; radar generally covers it.
            return True

    def should_publish_immediately(self, source: str, msg):
        """Publish immediately if very close."""
        if source == 'radar':
            rng = msg.distance
        else:
            rng = float(np.hypot(msg.position.x, msg.position.y))
        if rng <= self.near_immediate_range:
            return True
        return False

    def publish_radar_detection(self, radar_detection):
        """Publish radar detection as a single sensor detection."""
        modified_radar_detection = FusedDetection()
        modified_radar_detection.header = radar_detection.header
        modified_radar_detection.distance = radar_detection.distance
        modified_radar_detection.speed = radar_detection.speed
        modified_radar_detection.position = radar_detection.position
        modified_radar_detection.detection_type = 'radar'
        self.targets_to_publish.append(modified_radar_detection)

        # self.publisher_.publish(modified_radar_detection)
        # self.get_logger().info('Publishing radar detection at distance: '
        #                       f'{modified_radar_detection.distance}')

    def publish_camera_detection(self, camera_detection):
        """Publish camera detection as a single sensor detection."""
        modified_camera_detection = FusedDetection()
        modified_camera_detection.header = camera_detection.header
        modified_camera_detection.results = camera_detection.results
        modified_camera_detection.bbox = camera_detection.bbox
        modified_camera_detection.position = camera_detection.position
        distance = np.linalg.norm([camera_detection.position.x,
                                   camera_detection.position.y,])
        modified_camera_detection.distance = distance
        modified_camera_detection.is_tracking = camera_detection.is_tracking
        modified_camera_detection.tracking_id = camera_detection.tracking_id
        modified_camera_detection.detection_type = 'camera'
        self.targets_to_publish.append(modified_camera_detection)

        # self.publisher_.publish(modified_camera_detection)
        # self.get_logger().info('Publishing camera detection at distance: '
        #                       f'{modified_camera_detection.distance}')

    def publish_fused_detection(self, camera_detection, radar_detection):
        """Create a FusedDetection message from camera and radar detections."""
        fused_detection = FusedDetection()
        if camera_detection.header.frame_id != 'untracked_target':
            fused_detection.header = camera_detection.header
        else:
            fused_detection.header = radar_detection.header
        fused_detection.results = camera_detection.results
        fused_detection.bbox = camera_detection.bbox
        fused_detection.position = radar_detection.position
        fused_detection.is_tracking = camera_detection.is_tracking
        fused_detection.tracking_id = camera_detection.tracking_id
        fused_detection.distance = radar_detection.distance
        fused_detection.speed = radar_detection.speed
        fused_detection.detection_type = 'fused'
        self.targets_to_publish.append(fused_detection)

        # self.publisher_.publish(fused_detection)
        # self.get_logger().info('Publishing fused detection at distance: '
        #                       f'{fused_detection.distance}.'
        #                       f'Camera distance:'


def main(args=None):
    rclpy.init(args=args)
    fusion_node = FusionNode()
    rclpy.spin(fusion_node)
    fusion_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
