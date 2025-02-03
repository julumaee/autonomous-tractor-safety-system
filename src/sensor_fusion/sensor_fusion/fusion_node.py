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
        self.declare_parameter('time_threshold', 0.5)      # Default value 0.5 seconds
        self.declare_parameter('distance_threshold', 1.0)  # Default value 1 meter
        self.declare_parameter('radar_trust_min', 4)       # Default value 4 meters
        self.declare_parameter('camera_trust_max', 12)     # Default value 12 meters
        self.declare_parameter('rotation_matrix', [1.0, 0.0, 0.0,
                                                   0.0, 1.0, 0.0,
                                                   0.0, 0.0, 1.0])
        self.declare_parameter('translation_vector', [0.0, 0.0, 0.0])

        # Retrieve parameter values
        self.time_threshold = self.get_parameter('time_threshold').value
        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.radar_trust_min = self.get_parameter('radar_trust_min').value
        self.camera_trust_max = self.get_parameter('camera_trust_max').value
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
                self.distance_threshold = param.value
            elif param.name == 'radar_trust_min':
                self.distance_threshold = param.value
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
        self.get_logger().info(f'Received a detection from radar: \
                               {radar_msg.header.frame_id}')

    def attempt_fusion(self):
        """Match radar and camera detections, and perform fusion if a match is found."""
        if not self.camera_detections:
            for radar_msg in list(self.radar_detections):
                if radar_msg.distance > self.radar_trust_min:
                    self.publish_radar_detection(radar_msg)
                self.radar_detections.remove(radar_msg)
        for camera_msg in list(self.camera_detections):
            camera_time = camera_msg.header.stamp.sec
            + float(camera_msg.header.stamp.nanosec * 1e-9)

            best_matches = []  # Store matching radar detections here

            for radar_msg in list(self.radar_detections):
                radar_time = radar_msg.header.stamp.sec
                + float(radar_msg.header.stamp.nanosec * 1e-9)

                # Temporal matching
                if abs((camera_time - radar_time)) > self.time_threshold:
                    # If detection distance is over the minimum
                    # radar trusted distance, publish without fusion.
                    if radar_msg.distance > self.radar_trust_min:
                        self.publish_radar_detection(radar_msg)
                    self.radar_detections.remove(radar_msg)
                    continue  # No match found, continue to next detection

                # Spatial matching
                transformed_radar_point = self.transform_radar_to_camera(radar_msg.position)
                distance = np.linalg.norm([transformed_radar_point.x
                                           - camera_msg.position.x, transformed_radar_point.y
                                           - camera_msg.position.y])
                if distance < self.distance_threshold:
                    best_matches.append((radar_msg, distance))
                else:
                    # If detection distance is over the minimum
                    # radar trusted distance, publish without fusion.
                    if radar_msg.distance > self.radar_trust_min:
                        self.publish_radar_detection(radar_msg)
                    self.radar_detections.remove(radar_msg)
            if best_matches:
                best_match, _ = min(best_matches, key=lambda x: x[1])
                fused_detection = self.create_fused_detection(camera_msg, best_match)
                self.publisher_.publish(fused_detection)
                self.get_logger().info(f'Publishing fused detection with id: \
                                       {fused_detection.header.frame_id}')

                # Remove the radar and camera detections from deques
                for radar_msg, _ in best_matches:
                    if radar_msg in self.radar_detections:
                        self.radar_detections.remove(radar_msg)
                self.camera_detections.remove(camera_msg)
            else:
                camera_detection_distance = np.linalg.norm(
                    [camera_msg.position.x, camera_msg.position.y, camera_msg.position.z])
                # Publish camera detection if it is inside the trusted distance
                if camera_detection_distance < self.camera_trust_max:
                    modified_camera_msg = FusedDetection()
                    modified_camera_msg.bbox = camera_msg.bbox
                    modified_camera_msg.distance = int(camera_detection_distance)
                    modified_camera_msg.is_tracking = camera_msg.is_tracking
                    modified_camera_msg.tracking_id = camera_msg.tracking_id
                    self.publisher_.publish(modified_camera_msg)
                self.camera_detections.remove(camera_msg)

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

    def publish_radar_detection(self, radar_msg):
        modified_radar_msg = FusedDetection()
        modified_radar_msg.header = radar_msg.header
        modified_radar_msg.distance = radar_msg.distance
        modified_radar_msg.position = self.transform_radar_to_camera(radar_msg.position)
        self.publisher_.publish(modified_radar_msg)

    def create_fused_detection(self, camera_msg, radar_msg):
        """Create a FusedDetection message from camera and radar detections."""
        fused_detection = FusedDetection()
        fused_detection.header = camera_msg.header
        fused_detection.results = camera_msg.results
        fused_detection.bbox = camera_msg.bbox
        fused_detection.position = radar_msg.position
        fused_detection.is_tracking = camera_msg.is_tracking
        fused_detection.tracking_id = camera_msg.tracking_id
        fused_detection.distance = radar_msg.distance
        fused_detection.speed = radar_msg.speed
        return fused_detection


def main(args=None):
    rclpy.init(args=args)
    fusion_node = FusionNode()
    rclpy.spin(fusion_node)
    fusion_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
