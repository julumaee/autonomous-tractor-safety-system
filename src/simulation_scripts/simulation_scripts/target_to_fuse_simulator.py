import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Point
from vision_msgs.msg import ObjectHypothesis, BoundingBox2D
from std_msgs.msg import Header
from tractor_safety_system_interfaces.msg import CameraDetection, RadarDetection
import random

class TargetSimulationNode(Node):

    def __init__(self):
        super().__init__('target_simulation_node')
        self.camera_publisher = self.create_publisher(CameraDetection, '/camera_detections', 10)
        self.radar_publisher = self.create_publisher(RadarDetection, '/radar_detections', 10)
        self.timer = self.create_timer(2.0, self.publish_detections)  # Publish detections every second

        # Example extrinsic parameters (rotation matrix and translation vector)
        self.R = np.array([[1, 0, 0],
                           [0, 1, 0],
                           [0, 0, 1]])
        self.T = np.array([0, 0, 0])

    def publish_detections(self):
        # Generate a random target position in the world coordinate system
        target_position_world = self.generate_world_coordinates()
        self.publish_camera_detection(target_position_world)
        self.publish_radar_detection(target_position_world)

    def generate_world_coordinates(self):
        """Generates a random target position in the world coordinate system."""
        return Point(
            x=random.uniform(0, 10),  # Random x between 0 and 10 meters
            y=random.uniform(-5, 5),  # Random y between -5 and 5 meters
            z=0  # Assume target is on the ground plane
        )

    def publish_camera_detection(self, world_point):
        """Transforms world coordinates to camera coordinates, creates a camera detection and publishes it"""
        # Transform target world points into camera points
        world_point_homogeneous = np.array([world_point.x, world_point.y, world_point.z, 1])
        transformation_matrix = np.hstack((self.R, self.T.reshape(-1, 1)))
        transformation_matrix = np.vstack((transformation_matrix, [0, 0, 0, 1]))
        camera_point_homogeneous = np.dot(transformation_matrix, world_point_homogeneous)
        target_position_camera = Point(x=camera_point_homogeneous[0], y=camera_point_homogeneous[1], z=camera_point_homogeneous[2])
        # Create and publish CameraDetection
        camera_detection = CameraDetection()
        camera_detection.header = Header()
        camera_detection.header.stamp = self.get_clock().now().to_msg()
        camera_detection.results = []
        camera_detection.results = self.generate_object_hypotheses()
        camera_detection.header.frame_id = camera_detection.results[0].class_id
        camera_detection.position = target_position_camera
        camera_detection.bbox = self.generate_bounding_box(target_position_camera)
        self.camera_publisher.publish(camera_detection)
        self.get_logger().info(f"Published camera detection at {target_position_camera}")

    @staticmethod
    def generate_bounding_box(center):
        """Generate a random bounding box."""
        bbox = BoundingBox2D()
        bbox.center.position.x = center.x
        bbox.center.position.y = center.y
        bbox.center.theta = 0.0
        bbox.size_x = random.uniform(50, 150)
        bbox.size_y = random.uniform(50, 150)
        return bbox
    
    @staticmethod
    def generate_object_hypotheses():
        """Generate two simulated object hypotheses."""
        hypothesis1 = ObjectHypothesis(class_id="person", score=random.uniform(0.7, 0.95))
        hypothesis2 = ObjectHypothesis(class_id="car", score=random.uniform(0.4, 0.6))
        return [hypothesis1, hypothesis2]

    def publish_radar_detection(self, target_position_world):
        """Calculates and publishes radar detection (distance and angle) based on the target position."""
        distance = np.linalg.norm([target_position_world.x, target_position_world.y])
        angle = np.degrees(np.arctan2(target_position_world.y, target_position_world.x))
        speed = random.randint(-30, 30)
        # Create and publish RadarDetection
        radar_detection = RadarDetection()
        radar_detection.header.stamp = self.get_clock().now().to_msg()
        radar_detection.distance = int(distance)
        radar_detection.angle = int(angle)
        radar_detection.speed = speed
        self.radar_publisher.publish(radar_detection)
        self.get_logger().info(f"Published radar detection with distance {distance} and angle {angle}")


def main(args=None):
    rclpy.init(args=args)
    target_simulation_node = TargetSimulationNode()
    rclpy.spin(target_simulation_node)
    target_simulation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()