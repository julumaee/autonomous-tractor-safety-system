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

import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from tractor_safety_system_interfaces.msg import ControlCommand


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class TwistToCustomControl(Node):
    """
    Convert keyboard Twist (/cmd_vel) to custom control commands.

      - steering_angle: int32 (scaled from radians)
      - speed:          float32 (scaled from m/s)
    """

    def __init__(self):
        super().__init__('twist_to_custom_control')

        # Vehicle & mapping params
        self.declare_parameter('wheelbase', 2.5)            # meters
        self.declare_parameter('max_steer_rad', 0.8)        # radians
        self.declare_parameter('v_epsilon', 0.2)            # m/s (avoid div by 0)

        # Scaling to integers
        # Default: steering in degrees, speed in m/s
        self.declare_parameter('steer_unit_per_rad', 180.0 / math.pi)  # rad → deg
        self.declare_parameter('speed_unit_per_mps', 1.0)              # m/s → cm/s

        # Topics
        self.declare_parameter('twist_topic', '/cmd_vel')
        self.declare_parameter('out_topic', '/control/agopen')

        # Read params
        self.L = float(self.get_parameter('wheelbase').value)
        self.max_steer_rad = float(self.get_parameter('max_steer_rad').value)
        self.v_eps = float(self.get_parameter('v_epsilon').value)
        self.k_steer = float(self.get_parameter('steer_unit_per_rad').value)
        self.k_speed = float(self.get_parameter('speed_unit_per_mps').value)
        twist_topic = self.get_parameter('twist_topic').value
        out_topic = self.get_parameter('out_topic').value

        # Derived integer limits
        self.max_steer_scaled = self.max_steer_rad * self.k_steer

        self.sub = self.create_subscription(Twist, twist_topic, self._on_twist, 10)
        self.pub = self.create_publisher(ControlCommand, out_topic, 10)

        self.get_logger().info(
            f'Bridging {twist_topic} → {out_topic} | L={self.L} m | '
        )

    def _on_twist(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        # Bicycle model: yaw rate -> steering angle
        v_for_delta = v if abs(v) > self.v_eps else (self.v_eps * (1 if w >= 0 else -1))
        delta = math.atan2(self.L * w, abs(v_for_delta)) * (1 if v_for_delta >= 0 else -1)
        delta = clamp(delta, -self.max_steer_rad, self.max_steer_rad)

        # Scale to ints
        steer_scaled = delta * self.k_steer
        speed_scaled = v * self.k_speed

        steer_scaled = clamp(steer_scaled, -self.max_steer_scaled, self.max_steer_scaled)

        out = ControlCommand()
        out.steering_angle = steer_scaled
        out.speed = speed_scaled
        self.get_logger().info(
            f'Publishing ControlCommand: steer {steer_scaled} | speed {speed_scaled}'
        )
        self.pub.publish(out)


def main():
    rclpy.init()
    node = TwistToCustomControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
