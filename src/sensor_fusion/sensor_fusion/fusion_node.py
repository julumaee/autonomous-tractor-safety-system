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

        self.timer = self.create_timer(0.1, self.attempt_fusion)

        # Initialize detection deques
        self.camera_detections = deque(maxlen=20)  # Limit to 20 recent detections
        self.radar_detections = deque(maxlen=20)   # Limit to 20 recent detections

        # Declare parameters with default values
        self.declare_parameter('time_threshold', 0.5)         # Default value 0.5 seconds
        self.declare_parameter('distance_threshold', 1.0)     # Default value 1 meter
        self.declare_parameter('radar_trust_min', 4)          # Default value 4 meters
        self.declare_parameter('camera_trust_max', 12)        # Default value 12 meters
        self.declare_parameter('detection_score_trust', 0.5)  # Default value 0.5
        self.declare_parameter('rotation_matrix', [1.0, 0.0, 0.0,
                                                   0.0, 1.0, 0.0,
                                                   0.0, 0.0, 1.0])
        self.declare_parameter('translation_vector', [0.0, 0.0, 0.0])

        # Retrieve parameter values
        self.time_threshold = self.get_parameter('time_threshold').value
        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.radar_trust_min = self.get_parameter('radar_trust_min').value
        self.camera_trust_max = self.get_parameter('camera_trust_max').value
        self.detection_score_trust = self.get_parameter('detection_score_trust').value
        rotation_matrix_param = self.get_parameter('rotation_matrix').value
        translation_vector_param = self.get_parameter('translation_vector').value
        self.R = np.array(rotation_matrix_param).reshape(3, 3)
        self.T = np.array(translation_vector_param)

        # Subscribe to parameter updates
        self.add_on_set_parameters_callback(self.on_set_parameters)

    def on_set_parameters(self, params):
        """Set parameters their new values."""
        for param in params:
            if param.name == 'time_threshold':
                self.time_threshold = param.value
            elif param.name == 'distance_threshold':
                self.distance_threshold = param.value
            elif param.name == 'camera_trust_max':
                self.camera_trust_max = param.value
            elif param.name == 'radar_trust_min':
                self.radar_trust_min = param.value
            elif param.name == 'detection_score_trust':
                self.detection_score_trust = param.value
            elif param.name == 'rotation_matrix':
                rotation_matrix_param = param.value
                self.R = np.array(rotation_matrix_param).reshape(3, 3)
            elif param.name == 'translation_vector':
                translation_vector_param = param.value
                self.T = np.array(translation_vector_param)
        return SetParametersResult(successful=True)

    def listen_to_camera(self, camera_msg):
        self.camera_detections.append(camera_msg)
        self.get_logger().info(f'Received a detection from camera: {camera_msg.header.frame_id}')

    def listen_to_radar(self, radar_msg):
        self.radar_detections.append(radar_msg)
        self.get_logger().info('Received a detection from radar: '
                               f'{radar_msg.header.frame_id}')

    def process_radar_detection_without_fusion(self, radar_detection):
        """Process radar detections independently if no camera detections exist."""
        current_time = self.get_clock().now().to_msg().sec
        radar_time = self.get_detection_time(radar_detection)
        if self.verify_detection(radar_detection, 'radar'):
            self.publish_radar_detection(radar_detection)
        elif radar_time < current_time - 1:  # Remove old detections
            self.radar_detections.remove(radar_detection)

    def process_camera_detection_without_fusion(self, camera_detection):
        """Process camera detections independently if no radar detections exist."""
        current_time = self.get_clock().now().to_msg().sec
        camera_time = self.get_detection_time(camera_detection)
        if self.verify_detection(camera_detection, 'camera'):
            self.publish_camera_detection(camera_detection)
        elif camera_time < current_time - 1:  # Remove old detections
            self.camera_detections.remove(camera_detection)

    def transform_radar_to_camera(self, radar_point):
        """Transform radar coordinates to camera coordinates."""
        # Convert to homogeneous coordinates
        radar_point = np.array([radar_point.x,
                                radar_point.y,
                                radar_point.z, 1])
        # Apply transformation
        camera_coordinates = np.dot(np.hstack(
            (self.R, self.T.reshape(-1, 1))), radar_point)
        # Convert back to Cartesian coordinates and return as a Point()
        return Point(x=camera_coordinates[0],
                     y=camera_coordinates[1],
                     z=camera_coordinates[2])

    def verify_detection(self, detection, detection_type):
        """Verify detection validity."""
        if detection_type == 'camera':
            distance = np.linalg.norm([detection.position.x,
                                       detection.position.y,
                                       detection.position.z])
            if distance < self.camera_trust_max:
                if detection.results[0].score > self.detection_score_trust:
                    return True
        elif detection_type == 'radar':
            if detection.distance > self.radar_trust_min:
                return True
        return False

    def get_detection_time(self, detection):
        """Extract the detection timestamp from a ROS message."""
        return detection.header.stamp.sec + float(detection.header.stamp.nanosec * 1e-9)

    def temporal_match(self, camera_detection, radar_detection):
        """Return the distance of objects if a match is found, -1 if not."""
        camera_time = self.get_detection_time(camera_detection)
        radar_time = self.get_detection_time(radar_detection)
        if abs((camera_time - radar_time)) < self.time_threshold:  # Match!
            return True
        return False  # No match found

    def spatial_match(self, camera_detection, radar_detection):
        """Perform spatial matching. Return True if a match is found, False if not."""
        transformed_radar_point = self.transform_radar_to_camera(radar_detection.position)
        distance = np.linalg.norm([transformed_radar_point.x
                                   - camera_detection.position.x, transformed_radar_point.y
                                   - camera_detection.position.y])
        if distance < self.distance_threshold:  # Match!
            return True
        return False  # No match found

    def attempt_fusion(self):
        """Match radar and camera detections, and perform fusion if a match is found."""
        # If no camera detections, process radar detections without fusion
        if not self.camera_detections:
            for radar_detection in list(self.radar_detections):
                self.process_radar_detection_without_fusion(radar_detection)

        elif not self.radar_detections:
            for camera_detection in list(self.camera_detections):
                self.process_camera_detection_without_fusion(camera_detection)

        else:
            for camera_detection in list(self.camera_detections):
                matches = []  # Store matching radar detections here

                for radar_detection in list(self.radar_detections):

                    # Temporal matching
                    if not self.temporal_match(camera_detection, radar_detection):
                        continue  # No match found, continue to next detection

                    # Spatial matching
                    distance = self.spatial_match(camera_detection, radar_detection)
                    if distance != -1:
                        matches.append((radar_detection, distance))

                if matches:
                    # Find best match from matches
                    best_match, _ = min(matches, key=lambda x: x[1])
                    # Fuse the detections and publish
                    self.publish_fused_detection(camera_detection, best_match)

                    # Remove radar and camera detections from deques
                    for radar_detection, _ in matches:
                        if radar_detection in self.radar_detections:
                            self.radar_detections.remove(radar_detection)
                    self.camera_detections.remove(camera_detection)

            # Process the remaining detections individually:

            for camera_detection in list(self.camera_detections):
                self.process_camera_detection_without_fusion(camera_detection)

            for radar_detection in list(self.radar_detections):
                self.process_radar_detection_without_fusion(radar_detection)

    def publish_radar_detection(self, radar_detection):
        modified_radar_detection = FusedDetection()
        modified_radar_detection.header = radar_detection.header
        modified_radar_detection.distance = radar_detection.distance
        modified_radar_detection.speed = radar_detection.speed
        modified_radar_detection.position = self.transform_radar_to_camera(
            radar_detection.position
        )
        modified_radar_detection.detection_type = 'radar'
        self.publisher_.publish(modified_radar_detection)
        self.radar_detections.remove(radar_detection)
        self.get_logger().info('Publishing radar detection at distance: '
                               f'{modified_radar_detection.distance}')

    def publish_camera_detection(self, camera_detection):
        modified_camera_detection = FusedDetection()
        modified_camera_detection.bbox = camera_detection.bbox
        modified_camera_detection.position = camera_detection.position
        distance = np.linalg.norm([camera_detection.position.x,
                                   camera_detection.position.y,
                                   camera_detection.position.z])
        modified_camera_detection.distance = int(distance)
        modified_camera_detection.is_tracking = camera_detection.is_tracking
        modified_camera_detection.tracking_id = camera_detection.tracking_id
        modified_camera_detection.detection_type = 'camera'
        self.publisher_.publish(modified_camera_detection)
        self.camera_detections.remove(camera_detection)
        self.get_logger().info('Publishing camera detection at distance: '
                               f'{modified_camera_detection.distance}')

    def publish_fused_detection(self, camera_detection, radar_detection):
        """Create a FusedDetection message from camera and radar detections."""
        fused_detection = FusedDetection()
        fused_detection.header = camera_detection.header
        fused_detection.results = camera_detection.results
        fused_detection.bbox = camera_detection.bbox
        fused_detection.position = radar_detection.position
        fused_detection.is_tracking = camera_detection.is_tracking
        fused_detection.tracking_id = camera_detection.tracking_id
        fused_detection.distance = radar_detection.distance
        fused_detection.speed = radar_detection.speed
        fused_detection.detection_type = 'fused'
        self.publisher_.publish(fused_detection)
        self.get_logger().info('Publishing fused detection with id: '
                               f'{fused_detection.header.frame_id}')


def main(args=None):
    rclpy.init(args=args)
    fusion_node = FusionNode()
    rclpy.spin(fusion_node)
    fusion_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
