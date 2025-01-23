import rclpy
import numpy as np
from rclpy.node import Node
from collections import deque
from geometry_msgs.msg import Point
from tractor_safety_system_interfaces.msg import CameraDetection, RadarDetection, FusedDetection


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

        # Initialize variables
        self.camera_detections = deque(maxlen=20) # Limit to 20 recent detections
        self.radar_detections = deque(maxlen=20)  # Limit to 20 recent detections
        self.time_threshold = 1                   # Set time threshold for detection matching to 1 second
        self.distance_threshold = 1.0             # Set distance threshold for detection matching to 1 meter

        # Example extrinsic parameters (rotation matrix and translation vector)
        self.R = np.array([[1, 0, 0],
                           [0, 1, 0],
                           [0, 0, 1]])
        self.T = np.array([0, 0, 0])

    def listen_to_camera(self, camera_msg):
        self.camera_detections.append(camera_msg)
        #self.get_logger().info(f"Received a detection from camera: {camera_msg.header.frame_id}")
        self.attempt_fusion()

    def listen_to_radar(self, radar_msg):
        self.radar_detections.append(radar_msg)
        #self.get_logger().info(f"Received a detection from radar: {radar_msg.header.frame_id}")
        self.attempt_fusion()

    def attempt_fusion(self):
        """Matches radar and camera detections, and performs fusion if a match is found."""
        for camera_msg in list(self.camera_detections):
            camera_time = camera_msg.header.stamp.sec + float(camera_msg.header.stamp.nanosec * 1e-9)

            # Find the best radar match
            best_match = None
            best_distance = float('inf')

            for radar_msg in list(self.radar_detections):
                radar_time = radar_msg.header.stamp.sec + float(radar_msg.header.stamp.nanosec * 1e-9)

                # Check temporal proximity
                if abs((camera_time - radar_time)) > self.time_threshold:
                    continue

                radar_point = Point(
                    x=radar_msg.distance * np.cos(np.radians(radar_msg.angle)),
                    y=radar_msg.distance * np.sin(np.radians(radar_msg.angle)),
                    z=0
                )
                transformed_radar_point = self.transform_radar_to_camera(radar_point)
                if self.is_within_bbox(transformed_radar_point, camera_msg.bbox): # Check if radar detection is in the camera detection bbox
                    distance = np.linalg.norm([transformed_radar_point.x - camera_msg.position.x, transformed_radar_point.y - camera_msg.position.y])
                    if distance < self.distance_threshold and distance < best_distance:
                        best_distance = distance
                        best_match = radar_msg

            if best_match:
                fused_detection = FusedDetection()
                fused_detection.header = camera_msg.header
                fused_detection.results = camera_msg.results
                fused_detection.bbox = camera_msg.bbox
                fused_detection.position = camera_msg.position # This could be modified
                fused_detection.is_tracking = camera_msg.is_tracking
                fused_detection.tracking_id = camera_msg.tracking_id
                fused_detection.distance = radar_msg.distance
                fused_detection.angle = radar_msg.angle
                fused_detection.speed = radar_msg.speed

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

def main(args=None):
    rclpy.init(args=args)
    fusion_node = FusionNode()
    rclpy.spin(fusion_node)
    fusion_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
