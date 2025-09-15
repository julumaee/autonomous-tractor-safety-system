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

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from tractor_safety_system_interfaces.msg import ControlCommand


def clamp(x, lo, hi): return max(lo, min(hi, x))


class ControlToVehicle(Node):
    """
    Convert ControlCommand to vehicle commands.

      - geometry_msgs/Twist   (diff/ackermann plugins that take /cmd_vel)
      - ackermann_msgs/AckermannDriveStamped
    """

    def __init__(self):
        super().__init__('control_to_vehicle')

        # Interface: "twist" or "ackermann"
        self.declare_parameter('interface', 'twist')
        self.declare_parameter('in_topic',  '/control')  # safety output
        self.declare_parameter('twist_topic', '/vehicle_cmd_vel')  # sim input (if interface=twist)
        self.declare_parameter('ack_topic',  '/vehicle/ackermann_cmd')  # if interface=ackermann

        # Units / scaling (inverse of what you used earlier)
        # If your ints are already m/s and radians: set both *_per_* := 1.0
        self.declare_parameter('speed_units_per_mps', 1.0)  # ints per 1 m/s
        self.declare_parameter('steer_units_per_rad', 180.0/math.pi)  # ints per 1 rad (deg → int)

        # Bicycle params (only used for Twist)
        self.declare_parameter('wheelbase', 2.5)  # meters
        self.declare_parameter('yawrate_limit', 2.0)  # rad/s clamp

        self.iface = self.get_parameter('interface').value
        self.in_topic = self.get_parameter('in_topic').value
        self.tw_out = self.get_parameter('twist_topic').value
        self.ack_out = self.get_parameter('ack_topic').value
        self.k_v = float(self.get_parameter('speed_units_per_mps').value)
        self.k_delta = float(self.get_parameter('steer_units_per_rad').value)
        self.L = float(self.get_parameter('wheelbase').value)
        self.w_max = float(self.get_parameter('yawrate_limit').value)

        self.sub = self.create_subscription(ControlCommand, self.in_topic, self.cb, 10)

        if self.iface == 'twist':
            self.pub_tw = self.create_publisher(Twist, self.tw_out, 10)
            self.get_logger().info(f'Control → Twist on {self.tw_out}')
        else:
            self.pub_ack = self.create_publisher(AckermannDriveStamped, self.ack_out, 10)
            self.get_logger().info(f'Control → Ackermann on {self.ack_out}')

    def cb(self, msg: ControlCommand):
        # ints → SI
        v = float(msg.speed) / self.k_v               # m/s
        δ = float(msg.steering_angle) / self.k_delta  # rad

        if self.iface == 'twist':
            # bicycle: w = v * tan(δ) / L
            w = 0.0 if abs(δ) < 1e-9 else v * math.tan(δ) / max(1e-6, self.L)
            w = clamp(w, -self.w_max, self.w_max)
            out = Twist()
            out.linear.x = v
            out.angular.z = w
            self.pub_tw.publish(out)
        else:
            out = AckermannDriveStamped()
            out.drive.speed = v
            out.drive.steering_angle = δ
            self.pub_ack.publish(out)


def main():
    rclpy.init()
    node = ControlToVehicle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
