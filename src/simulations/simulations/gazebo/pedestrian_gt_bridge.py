# pedestrian_gt_bridge.py
# Collects individual model pose topics and publishes a PoseArray on /gt_pedestrians.

from geometry_msgs.msg import Pose, PoseArray
import rclpy
from rclpy.node import Node
from typing import Dict, List


class PedestrianGTBridge(Node):
    def __init__(self):
        super().__init__('pedestrian_gt_bridge')

        # List of pedestrian model names (as in /model/<name>/pose)
        self.declare_parameter(
            'pedestrian_models',
            ['actor_1', 'actor_2']
        )
        self.pedestrian_models: List[str] = list(
            self.get_parameter('pedestrian_models')
                .get_parameter_value().string_array_value
        )

        # Store latest pose per pedestrian index
        self.latest_poses: Dict[int, Pose] = {}

        # Publisher for ground-truth pose array
        self.pub_gt = self.create_publisher(PoseArray, '/gt_pedestrians', 10)

        # Subscription for unfiltered gazebo poses
        self.create_subscription(
            PoseArray,
            '/poses/unfiltered',
            self._unfiltered_pose_callback,
            10
        )

        # Publish at 20 Hz (adjust if you like)
        self.timer = self.create_timer(0.05, self.on_timer)

        self.get_logger().info(
            f'PedestrianGTBridge started with models: {self.pedestrian_models}'
        )

    def _unfiltered_pose_callback(self, msg: PoseArray):
        """Callback for unfiltered gazebo poses."""
        idx = 0
        for pose in msg.poses:
            if (pose.position.x <= 10.0
                or pose.position.x >= 20.0):
                continue  # skip other than actor poses
            idx += 1
            self.latest_poses[idx] = pose

    def on_timer(self):
        """Publish PoseArray of all pedestrians with a pose received at least once."""
        if not self.latest_poses:
            return

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'  # or 'map', just be consistent across system

        # Sort by index so person_id is stable and matches logger analysis
        for idx in sorted(self.latest_poses.keys()):
            msg.poses.append(self.latest_poses[idx])

        self.pub_gt.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianGTBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
