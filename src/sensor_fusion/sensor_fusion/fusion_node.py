import rclpy
import numpy as np
from rclpy.node import Node
from collections import deque
from geometry_msgs.msg import Point
from tractor_safety_system_interfaces.msg import CameraDetection, RadarDetection, FusedDetection
from rcl_interfaces.msg import SetParametersResult

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

        # Initialize detection deques
        self.camera_detections = deque(maxlen=20) # Limit to 20 recent detections
        self.radar_detections = deque(maxlen=20)  # Limit to 20 recent detections

        # Declare parameters with default values
        self.declare_parameter('time_threshold', 1.0)
        self.declare_parameter('distance_threshold', 1.0)
        self.declare_parameter('rotation_matrix', [1.0, 0.0, 0.0,
                                                   0.0, 1.0, 0.0,
                                                   0.0, 0.0, 1.0])
        self.declare_parameter('translation_vector', [0.0, 0.0, 0.0])

        # Retrieve parameter values
        self.time_threshold         = self.get_parameter('time_threshold').value
        self.distance_threshold     = self.get_parameter('distance_threshold').value
        rotation_matrix_param       = self.get_parameter('rotation_matrix').value
        translation_vector_param    = self.get_parameter('translation_vector').value
        self.R = np.array(rotation_matrix_param).reshape(3, 3)
        self.T = np.array(translation_vector_param)

        # Subscribe to parameter updates
        self.add_on_set_parameters_callback(self.on_set_parameters)

    def on_set_parameters(self, params):
        """Sets parameters their new values when called"""
        for param in params:
            if param.name == 'time_threshold':
                self.time_threshold = param.value
            elif param.name == 'distance_threshold':
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
        self.get_logger().info(f"Received a detection from camera: {camera_msg.header.frame_id}")
        self.attempt_fusion()

    def listen_to_radar(self, radar_msg):
        self.radar_detections.append(radar_msg)
        self.get_logger().info(f"Received a detection from radar: {radar_msg.header.frame_id}")
        self.attempt_fusion()

    def attempt_fusion(self):
        """Matches radar and camera detections, and performs fusion if a match is found."""
        for camera_msg in list(self.camera_detections):
            camera_time = camera_msg.header.stamp.sec + float(camera_msg.header.stamp.nanosec * 1e-9)

            best_matches = [] # Store matching radar detections here

            for radar_msg in list(self.radar_detections):
                radar_time = radar_msg.header.stamp.sec + float(radar_msg.header.stamp.nanosec * 1e-9)

                # Temporal matching
                if abs((camera_time - radar_time)) > self.time_threshold:
                    continue

                radar_point = Point(
                    x=radar_msg.distance * np.cos(np.radians(radar_msg.angle)),
                    y=radar_msg.distance * np.sin(np.radians(radar_msg.angle)),
                    z=0
                )
                transformed_radar_point = self.transform_radar_to_camera(radar_point)
                # Check if radar detection is in the camera detection bbox (e.g. spatial matching)
                if self.is_within_bbox(transformed_radar_point, camera_msg.bbox):
                    # Calculate distance between detections
                    distance = np.linalg.norm([transformed_radar_point.x - camera_msg.position.x, transformed_radar_point.y - camera_msg.position.y])
                    if distance < self.distance_threshold:
                        best_matches.append((radar_msg, distance))

            if best_matches:
                best_match, _ = min(best_matches, key=lambda x: x[1])
                fused_detection = self.create_fused_detection(camera_msg, best_match)
                self.publisher_.publish(fused_detection)
                self.get_logger().info(f"Publishing fused detection with id: {fused_detection.header.frame_id}")
                
                # Remove the fused radar and camera detections from deques
                self.radar_detections.remove(best_match)
                self.camera_detections.remove(camera_msg)
            
    def transform_radar_to_camera(self, radar_point):
        """Transforms radar coordinates to camera coordinates."""

        # Convert to homogeneous coordinates
        radar_point = np.array([radar_point.x, radar_point.y, radar_point.z, 1])
        # Apply transformation
        camera_coordinates = np.dot(np.hstack((self.R, self.T.reshape(-1, 1))), radar_point)
        # Convert back to Cartesian coordinates and return as a Point()
        return Point(x=camera_coordinates[0], y=camera_coordinates[1], z=camera_coordinates[2])

    def is_within_bbox(self, point, bbox):
        """Checks if a point is within a bounding box."""
        x, y = point.x, point.y
        return (bbox.center.position.x - bbox.size_x / 2 <= x <= bbox.center.position.x + bbox.size_x / 2 and
                bbox.center.position.y - bbox.size_y / 2 <= y <= bbox.center.position.y + bbox.size_y / 2)

    def create_fused_detection(self, camera_msg, radar_msg):
        """Creates a FusedDetection message from camera and radar detections."""
        fused_detection = FusedDetection()
        fused_detection.header = camera_msg.header
        fused_detection.results = camera_msg.results
        fused_detection.bbox = camera_msg.bbox
        fused_detection.position = camera_msg.position  # Could update with radar info
        fused_detection.is_tracking = camera_msg.is_tracking
        fused_detection.tracking_id = camera_msg.tracking_id
        fused_detection.distance = radar_msg.distance
        fused_detection.angle = radar_msg.angle
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
