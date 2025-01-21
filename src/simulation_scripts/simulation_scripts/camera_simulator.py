import rclpy
import random
from rclpy.node import Node
from vision_msgs.msg import ObjectHypothesis, BoundingBox2D
from geometry_msgs.msg import Point, Pose2D
from depthai_ros_msgs.msg import SpatialDetection

class CameraSimulator(Node):
    def __init__(self):
        super().__init__('camera_simulator')
        self.publisher_ = self.create_publisher(SpatialDetection, '/oakd/detections', 10)
        self.timer = self.create_timer(2, self.simulate_detection)

    def simulate_detection(self):
        # Create a simulated SpatialDetection message
        detection = SpatialDetection()
        detection.results = self.generate_object_hypotheses()
        #detection.bbox = self.generate_bounding_box() # TODO does this need to be used?
        detection.position = self.generate_position()
        detection.is_tracking = random.choice([True, False])
        detection.tracking_id = str(random.randint(1, 1000))

        # Publish the message
        self.publisher_.publish(detection)
        self.get_logger().info(f"Hypothesis: {detection.results} Position: {detection.position}")

    @staticmethod
    def generate_object_hypotheses():
        """Generate two simulated object hypotheses."""
        hypothesis1 = ObjectHypothesis(hypothesis_id="person", score=random.uniform(0.7, 0.95))
        hypothesis2 = ObjectHypothesis(hypothesis_id="car", score=random.uniform(0.5, 0.85))
        return [hypothesis1, hypothesis2]

    #@staticmethod
    #def generate_bounding_box():
    #    """Generate a random bounding box."""
    #    bbox = BoundingBox2D()
    #    bbox.center.x = random.uniform(100, 500)
    #    bbox.center.y = random.uniform(100, 500)
    #    bbox.center.theta = 0.0
    #    bbox.size_x = random.uniform(50, 150)
    #    bbox.size_y = random.uniform(50, 150)
    #    return bbox

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