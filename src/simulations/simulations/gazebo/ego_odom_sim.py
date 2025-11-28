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
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class OdomToEgoMotion(Node):

    def __init__(self):
        super().__init__('odom_to_ego_motion')
        self.declare_parameter('in_topic', '/odom')
        self.declare_parameter('out_topic', '/ego_motion')
        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value
        self.sub = self.create_subscription(Odometry, in_topic, self.cb, 50)
        self.pub = self.create_publisher(TWCS, out_topic, 50)

    def cb(self, od):
        msg = TWCS()
        msg.header = od.header             # keep the sim timestamp
        msg.header.frame_id = 'base_link'  # twist is in child frame (base_link)
        msg.twist = od.twist               # copy twist + covariance as-is
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToEgoMotion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
