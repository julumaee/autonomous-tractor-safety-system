import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class FusionSubscriber(Node):

    def __init__(self):
        super().__init__('fusion_node')
        self.subscription = self.create_subscription(
            String,
            '/detections',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(String, '/detections/fused', 10)

    def listener_callback(self, msg):
        
        self.get_logger().info(msg.data)

    #def fusion_logic():


def main(args=None):
    rclpy.init(args=args)
    fusion_subscriber = FusionSubscriber()
    rclpy.spin(fusion_subscriber)
    fusion_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
