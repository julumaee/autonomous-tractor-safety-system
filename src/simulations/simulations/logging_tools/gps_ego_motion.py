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

from geometry_msgs.msg import TwistWithCovarianceStamped as TWCS
import rclpy
from rclpy.node import Node
from ublox_msgs.msg import NavPVT


class FieldTestEgoMotion(Node):
    def __init__(self):
        super().__init__("gps_ego_motion_fieldtest")

        self.declare_parameter("navpvt_topic", "/navpvt")
        self.declare_parameter("publish_topic", "/ego_motion")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("min_fix_type", 2)   # 2D
        self.declare_parameter("min_sv", 4)
        self.declare_parameter("timeout_s", 1.0)

        self.navpvt_topic = self.get_parameter("navpvt_topic").value
        self.publish_topic = self.get_parameter("publish_topic").value
        self.rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.min_fix_type = int(self.get_parameter("min_fix_type").value)
        self.min_sv = int(self.get_parameter("min_sv").value)
        self.timeout_s = float(self.get_parameter("timeout_s").value)

        self.pub = self.create_publisher(TWCS, self.publish_topic, 50)
        self.create_subscription(NavPVT, self.navpvt_topic, self.cb, 20)

        self.timer = self.create_timer(1.0 / self.rate_hz, self.publish)

        self.last_rx_time = None
        self.speed_mps = 0.0

        # Conservative covariances for speed-only compensation
        self.cov = [0.0] * 36
        self.cov[0] = 0.09     # vx variance (~0.3 m/s std)
        self.cov[35] = 0.001   # yaw-rate variance small (we set yaw-rate to 0)

        self.get_logger().info(f"Field-test ego motion from {self.navpvt_topic}")
        self.get_logger().info(
            f"Publishing {self.publish_topic} at {self.rate_hz} Hz "
            "(yaw_rate forced to 0)"
        )

    def cb(self, msg: NavPVT):
        now = self.get_clock().now()
        self.last_rx_time = now

        if msg.fix_type < self.min_fix_type or msg.num_sv < self.min_sv:
            self.speed_mps = 0.0
            return

        # NavPVT vel_n/vel_e are typically mm/s in ublox; convert to m/s:
        vn = msg.vel_n * 1e-3
        ve = msg.vel_e * 1e-3
        self.speed_mps = math.hypot(vn, ve)

    def publish(self):
        if self.last_rx_time is None:
            return

        age = (self.get_clock().now() - self.last_rx_time).nanoseconds * 1e-9
        speed = self.speed_mps if age <= self.timeout_s else 0.0

        out = TWCS()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_link"
        out.twist.twist.linear.x = float(speed)
        out.twist.twist.angular.z = 0.0
        out.twist.covariance = self.cov
        self.pub.publish(out)


def main():
    rclpy.init()
    node = FieldTestEgoMotion()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
