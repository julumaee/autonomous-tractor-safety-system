from depthai_ros_msgs.msg import SpatialDetectionArray
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from tractor_safety_system_interfaces.msg import CameraDetection


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(CameraDetection, '/camera_detections', 10)
        self.subscription = self.create_subscription(
            SpatialDetectionArray,
            '/oakd/detections',
            self.publish_detections,
            10)
        self.subscription  # prevent unused variable warning

    def convert_message(self, oakd_msg):
        """Convert messages from Luxonis OAKD-lite 2 to CameraMessage format."""
        camera_detectin_msg = CameraDetection()
        camera_detectin_msg.results = oakd_msg.results
        camera_detectin_msg.bbox = oakd_msg.bbox
        camera_detectin_msg.position = oakd_msg.position
        camera_detectin_msg.is_tracking = oakd_msg.is_tracking
        camera_detectin_msg.tracking_id = oakd_msg.tracking_id
        return camera_detectin_msg

    def publish_detections(self, oakd_msg):
        """Publish detections as individual CameraDetections from a SpatialDetectionArray."""
        for detection in oakd_msg.detections:
            camera_detection_msg = self.convert_message(detection)
            camera_detection_msg.header = Header()
            camera_detection_msg.header.stamp = self.get_clock().now().to_msg()
            camera_detection_msg.header.frame_id = camera_detection_msg.results[0].class_id
            self.publisher_.publish(camera_detection_msg)
            self.get_logger().info(f'Publishing camera detection with id: \
                                   {camera_detection_msg.header.frame_id}')


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
