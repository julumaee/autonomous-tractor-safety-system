import random

from depthai_ros_msgs.msg import SpatialDetection, SpatialDetectionArray
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from tractor_safety_system_interfaces.msg import SimulatedObject
from vision_msgs.msg import BoundingBox2D, ObjectHypothesis


class CameraSimulator(Node):
    def __init__(self):
        super().__init__('camera_simulator')
        self.publisher_ = self.create_publisher(SpatialDetectionArray,
                                                '/color/yolov4_Spatial_detections',
                                                10)
        self.object_subscription = self.create_subscription(
            SimulatedObject,
            '/simulated_objects',
            self.simulate_detection,
            10)

    def simulate_detection(self, simulated_object):
        """Simulate detections and publish them as a SpatialDetectionArray message."""
        # Create a simulated SpatialDetectionArray message with 2 detections
        distance = (simulated_object.position.x ** 2 + simulated_object.position.y ** 2) ** 0.5
        if distance > 15:
            self.get_logger().info(f'Object is too far away: {distance}m')
        else:
            detection_array = SpatialDetectionArray()
            detection_array.header = Header()
            detection_array.header.stamp = self.get_clock().now().to_msg()
            detection_array.header.frame_id = 'oakd_camera_frame'

            detection = SpatialDetection()
            detection.results = []
            detection.results = self.generate_object_hypotheses()
            detection.bbox = self.generate_bounding_box()  # TODO Should match the position?
            detection.position = simulated_object.position
            detection.is_tracking = random.choice([True, False])
            if (detection.is_tracking):
                detection.tracking_id = f'object_{simulated_object.object_id}'
            detection_array.detections.append(detection)  # Add detection to the array

            # Publish the message
            self.publisher_.publish(detection_array)
            self.get_logger().info(f'Published a detection as {detection_array.header.frame_id}')

    @staticmethod
    def generate_object_hypotheses():
        """Generate two simulated object hypotheses."""
        hypothesis1 = ObjectHypothesis(class_id='person', score=random.uniform(0.7, 0.95))
        hypothesis2 = ObjectHypothesis(class_id='car', score=random.uniform(0.4, 0.6))
        return [hypothesis1, hypothesis2]

    @staticmethod
    def generate_bounding_box():
        """Generate a random bounding box."""
        bbox = BoundingBox2D()
        bbox.center.position.x = random.uniform(100, 500)
        bbox.center.position.y = random.uniform(100, 500)
        bbox.center.theta = 0.0
        bbox.size_x = random.uniform(50, 150)
        bbox.size_y = random.uniform(50, 150)
        return bbox

    @staticmethod
    def generate_position():
        """Generate a random 3D position."""
        position = Point()
        position.x = random.uniform(1.0, 15.0)  # Depth
        position.y = random.uniform(-5.0, 5.0)  # Left-Right
        position.z = random.uniform(-0.0, 0.0)  # Up-Down
        return position


def main(args=None):
    rclpy.init(args=args)
    camera_simulator = CameraSimulator()
    rclpy.spin(camera_simulator)
    camera_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
