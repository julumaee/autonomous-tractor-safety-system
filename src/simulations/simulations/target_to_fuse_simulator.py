import random

from geometry_msgs.msg import Point
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from tractor_safety_system_interfaces.msg import CameraDetection, RadarDetection
from vision_msgs.msg import BoundingBox2D, ObjectHypothesis


class TargetSimulationNode(Node):

    def __init__(self):
        super().__init__('target_simulation_node')
        self.camera_publisher = self.create_publisher(CameraDetection, '/camera_detections', 10)
        self.radar_publisher = self.create_publisher(RadarDetection, '/radar_detections', 10)
        # Publish detections every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_detections)

        # Declare parameters with default values
        self.declare_parameter('rotation_matrix', [1.0, 0.0, 0.0,
                                                   0.0, 1.0, 0.0,
                                                   0.0, 0.0, 1.0])
        self.declare_parameter('translation_vector', [0.0, 0.0, 0.0])

        # Retrieve the parameter values
        rotation_matrix_param = self.get_parameter('rotation_matrix').value
        self.R = np.array(rotation_matrix_param).reshape(3, 3)
        translation_vector_param = self.get_parameter('translation_vector').value
        self.T = np.array(translation_vector_param)

        # Subscribe to parameter updates
        self.add_on_set_parameters_callback(self.on_set_parameters)

    def on_set_parameters(self, params):
        """Set parameters their new values."""
        for param in params:
            if param.name == 'rotation_matrix':
                rotation_matrix_param = param.value
                self.R = np.array(rotation_matrix_param).reshape(3, 3)
            elif param.name == 'translation_vector':
                translation_vector_param = param.value
                self.T = np.array(translation_vector_param)
        return SetParametersResult(successful=True)

    def publish_detections(self):
        """Generate and publish targets."""
        target_position_world = self.generate_world_coordinates()
        self.publish_camera_detection(target_position_world)
        self.publish_radar_detection(target_position_world)

    def generate_world_coordinates(self):
        """Generate a random target position in the world coordinate system."""
        return Point(
            x=random.uniform(-50, 50),  # Random x between -50 and 50 meters
            y=random.uniform(0, 50),  # Random y between 0 and 50 meters
            z=0.0  # Assume target is on the ground plane
        )

    def publish_camera_detection(self, world_point):
        """Create a camera detection based on given coordinates and publish it."""
        # Transform target world points into camera points
        world_point_homogeneous = np.array([world_point.x, world_point.y, world_point.z, 1])
        transformation_matrix = np.hstack((self.R, self.T.reshape(-1, 1)))
        transformation_matrix = np.vstack((transformation_matrix, [0, 0, 0, 1]))
        camera_point_homogeneous = np.dot(transformation_matrix, world_point_homogeneous)
        target_position_camera = Point(x=camera_point_homogeneous[0],
                                       y=camera_point_homogeneous[1],
                                       z=camera_point_homogeneous[2])
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
        self.get_logger().info(f'Published camera detection at {target_position_camera}')

    @staticmethod
    def generate_bounding_box(center):
        """Generate a random bounding box based on a given center point."""
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
        hypothesis1 = ObjectHypothesis(class_id='person', score=random.uniform(0.7, 0.95))
        hypothesis2 = ObjectHypothesis(class_id='car', score=random.uniform(0.4, 0.6))
        return [hypothesis1, hypothesis2]

    def publish_radar_detection(self, target_position_world):
        """Calculate and publish radar detection based on the target position."""
        distance = np.linalg.norm([target_position_world.x, target_position_world.y])
        speed = random.randint(-30, 30)
        # Create and publish RadarDetection
        radar_detection = RadarDetection()
        radar_detection.header.stamp = self.get_clock().now().to_msg()
        radar_detection.distance = int(distance)
        radar_detection.position = target_position_world
        radar_detection.speed = speed
        self.radar_publisher.publish(radar_detection)
        self.get_logger().info(f'Published radar detection with distance {distance}')


def main(args=None):
    rclpy.init(args=args)
    target_simulation_node = TargetSimulationNode()
    rclpy.spin(target_simulation_node)
    target_simulation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
