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

import random

import rclpy
from rclpy.node import Node
from tractor_safety_system_interfaces.msg import ControlCommand


class AgOpenSimulator(Node):

    def __init__(self):
        super().__init__('agopen_simulator')
        self.publisher_ = self.create_publisher(ControlCommand, '/control/agopen', 10)
        self.timer = self.create_timer(1, self.simulate_control)

    def simulate_control(self):
        """Simulate agopen control commands."""
        command = ControlCommand()
        command.speed = random.randint(10, 15)
        command.steering_angle = random.randint(0, 50)
        self.publisher_.publish(command)


def main(args=None):
    rclpy.init(args=args)
    agopen_simulator = AgOpenSimulator()
    rclpy.spin(agopen_simulator)
    agopen_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
