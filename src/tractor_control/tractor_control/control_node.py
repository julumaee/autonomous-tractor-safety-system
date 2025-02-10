import rclpy
from rclpy.node import Node
from tractor_safety_system_interfaces.msg import ControlCommand


class TractorControl(Node):

    def __init__(self):
        super().__init__('control_node')
        self.control_subscription = self.create_subscription(
            ControlCommand,
            '/control',
            self.control_tractor,
            10)

    def control_tractor(self, control_command):
        """Control the tractor based on the received command."""
        # TODO The tractor control logic goes here
        self.get_logger().info('Received control command: '
                               f'speed={control_command.speed}, '
                               f'steering_angle={control_command.steering_angle}')


def main(args=None):
    rclpy.init(args=args)
    control_node = TractorControl()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
