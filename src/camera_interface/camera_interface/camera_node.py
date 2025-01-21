import rclpy
from rclpy.node import Node
from depthai_ros_msgs.msg import SpatialDetection
from tractor_safety_system_interfaces.msg import CameraDetection


class CameraReceiver(Node):
    def __init__(self):
        super().__init__('camera_receiver')
        self.publisher_ = self.create_publisher(CameraDetection, '/detections', 10)
        self.subscription = self.create_subscription(
            SpatialDetection,
            '/oakd/detections',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def convert_message(self, oakd_msg):
        camera_detectin_msg = CameraDetection()
        camera_detectin_msg.results = oakd_msg.results
        camera_detectin_msg.bbox = oakd_msg.bbox
        camera_detectin_msg.position = oakd_msg.position
        camera_detectin_msg.is_tracking = oakd_msg.is_tracking
        camera_detectin_msg.tracking_id = oakd_msg.tracking_id
    
    def listener_callback(self, msg):
        camera_detection_msg = self.convert_message(msg)
        self.publisher_.publish(camera_detection_msg)

def main(args=None):
    rclpy.init(args=args)
    camera_receiver = CameraReceiver()
    rclpy.spin(camera_receiver)
    camera_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()