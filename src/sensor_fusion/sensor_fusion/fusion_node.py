import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from tractor_safety_system_interfaces.msg import CameraDetection, RadarDetection


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
        self.publisher = self.create_publisher(String, '/detections/fused', 10)

    def listen_to_camera(self, camera_msg):
        self.get_logger().info(f"Received a detection from camera: {camera_msg.header.frame_id}")

    def listen_to_radar(self, radar_msg):
        self.get_logger().info(f"Received a detection from radar: {radar_msg.header.frame_id}")

    #def fusion_logic():


def main(args=None):
    rclpy.init(args=args)
    fusion_node = FusionNode()
    rclpy.spin(fusion_node)
    fusion_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
