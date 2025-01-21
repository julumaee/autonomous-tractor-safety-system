import rclpy
import random
from rclpy.node import Node
from vision_msgs.msg import ObjectHypothesis, BoundingBox2D
from geometry_msgs.msg import Point
from depthai_ros_msgs.msg import SpatialDetectionArray, SpatialDetection
from std_msgs.msg import Header

class CameraSimulator(Node):
    def __init__(self):
        super().__init__('camera_simulator')
        self.tracking_id = 1
        self.publisher_ = self.create_publisher(SpatialDetectionArray, '/oakd/detections', 10)
        self.timer = self.create_timer(2, self.simulate_detection)

    def simulate_detection(self):
        # Create a simulated SpatialDetectionArray message with 2 detections
        detection_array = SpatialDetectionArray()
        detection_array.header = Header()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = f"oakd_camera_frame"
        # Detection 1:
        detection1 = SpatialDetection()
        detection1.results = []
        detection1.results = self.generate_object_hypotheses()
        detection1.bbox = self.generate_bounding_box() # TODO does this need to be used?
        detection1.position = self.generate_position()
        detection1.is_tracking = random.choice([True, False])
        if (detection1.is_tracking):
            detection1.tracking_id = f"object_{self.tracking_id}"
            self.tracking_id += 1 # Update tracking_id
        detection_array.detections.append(detection1) # Add detection to the array

        # Detection 2:
        detection2 = SpatialDetection()
        detection2.results = []
        detection2.results = self.generate_object_hypotheses()
        detection2.bbox = self.generate_bounding_box() # TODO does this need to be used?
        detection2.position = self.generate_position()
        detection2.is_tracking = random.choice([True, False])
        if (detection2.is_tracking):
            detection2.tracking_id = f"object_{self.tracking_id}"
            self.tracking_id += 1 # Update tracking_id
        detection_array.detections.append(detection2) # Add detection to the array

        # Publish the message
        self.publisher_.publish(detection_array)
        self.get_logger().info(f"Published 2 detections as {detection_array.header.frame_id}")

    @staticmethod
    def generate_object_hypotheses():
        """Generate two simulated object hypotheses."""
        hypothesis1 = ObjectHypothesis(class_id="person", score=random.uniform(0.7, 0.95))
        hypothesis2 = ObjectHypothesis(class_id="car", score=random.uniform(0.4, 0.6))
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
        position.x = random.uniform(1.0, 5.0)  # Depth
        position.y = random.uniform(-1.0, 1.0)  # Left-Right
        position.z = random.uniform(-0.5, 0.5)  # Up-Down
        return position

def main(args=None):
    rclpy.init(args=args)
    camera_simulator = CameraSimulator()
    rclpy.spin(camera_simulator)
    camera_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()