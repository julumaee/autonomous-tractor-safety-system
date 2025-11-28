# Copyright 2025 Eemil Kulmala, University of Oulu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import TwistWithCovarianceStamped as TWCS
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
        self.pub = self.create_publisher(TWCS, '/ego_motion', 50)

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
