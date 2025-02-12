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
            # The topic name should match the one in the launch file
            '/color/yolov4_Spatial_detections',
            self.publish_detections,
            10)
        self.subscription  # prevent unused variable warning

    def convert_message(self, detection):
        """Convert messages from Luxonis OAKD-lite 2 to CameraMessage format."""
        camera_detectin_msg = CameraDetection()
        camera_detectin_msg.results = []
        for result in detection.results:
            camera_detectin_msg.results.append(result)
        camera_detectin_msg.bbox = detection.bbox
        camera_detectin_msg.position = detection.position
        camera_detectin_msg.is_tracking = detection.is_tracking
        camera_detectin_msg.tracking_id = detection.tracking_id
        return camera_detectin_msg

    def publish_detections(self, oakd_msg):
        """Publish detections as individual CameraDetections from a SpatialDetectionArray."""
        for detection in oakd_msg.detections:
            camera_detection_msg = self.convert_message(detection)
            camera_detection_msg.header = Header()
            camera_detection_msg.header.stamp = self.get_clock().now().to_msg()
            camera_detection_msg.header.frame_id = camera_detection_msg.results[0].class_id
            self.publisher_.publish(camera_detection_msg)
            self.get_logger().info('Publishing camera detection with id: '
                                   f'{camera_detection_msg.header.frame_id}')


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
