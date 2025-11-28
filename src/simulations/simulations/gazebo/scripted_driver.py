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

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ScriptedDriver(Node):
    def __init__(self):
        super().__init__('scripted_driver')

        # Parameters
        self.declare_parameter('scenario_name', 'S1')
        self.declare_parameter('cmd_topic', '/vehicle_cmd_vel')

        self.scenario_name = (
            self.get_parameter('scenario_name')
                .get_parameter_value().string_value
        )
        cmd_topic = (
            self.get_parameter('cmd_topic')
                .get_parameter_value().string_value
        )

        self.get_logger().info(
            f'Starting scripted driver for scenario "{self.scenario_name}" '
            f'on cmd topic "{cmd_topic}"'
        )

        # Publisher
        self.pub_cmd = self.create_publisher(Twist, cmd_topic, 10)

        # Define path segments per scenario:
        # (duration [s], linear_x [m/s], angular_z [rad/s])
        self.segments = self._build_segments(self.scenario_name)

        # Precompute cumulative times
        self.segment_end_times = []
        t = 0.0
        for dur, _, _ in self.segments:
            t += dur
            self.segment_end_times.append(t)
        self.total_duration = t

        # Use sim time if available
        self.use_sim_time = True
        self.start_time = None

        # Timer for sending commands (20 Hz)
        self.timer = self.create_timer(0.05, self.on_timer)

    def _build_segments(self, scenario: str):
        if scenario == 'S1':
            # S1: drive straight towards a static pedestrian.
            return [
                (3.0, 0.0, 0.0),   # stand still 3 s (for setup)
                (29.0, 1.0, 0.0),  # drive straight 30 s
                (3.0, 0.0, 0.0),   # stand still
            ]
        elif scenario == 'S2':
            # S2: straight line past multiple static pedestrians
            return [
                (3.0, 0.0, 0.0),   # stand still 3 s (for setup)
                (30.0, 1.0, 0.0),  # drive straight 30 s
                (3.0, 0.0, 0.0),   # stand still
            ]
        elif scenario == 'S3':
            # S3: Pedestrians crossing in front of the vehicle
            return [
                (20.0, 0.0, 0.0),  # stand still 20 s (wait for pedestrians to cross)
            ]
        elif scenario == 'S4':
            # S4: approach three pedestrians with an s-shaped path
            # radius r = v / omega  => choose v, omega accordingly
            v = 1.0
            omega = 0.3          # ~3.3 m radius
            circle_time = 1.5
            return [
                (3.0, 0.0, 0.0),              # Stand still
                (0.5*circle_time, v, omega),  # Curve left
                (circle_time, v, -omega),     # Curve right
                (circle_time, v, omega),      # Curve left
                (circle_time, v, -omega),     # Curve right
                (circle_time, v, omega),      # Curve left
                (circle_time, v, -omega),     # Curve right
                (circle_time, v, omega),      # Curve left
                (circle_time, v, -omega),     # Curve right
                (circle_time, v, omega),      # Curve left
                (circle_time, v, -omega),     # Curve right
                (circle_time, v, omega),      # Curve left
                (circle_time, v, -omega),     # Curve right
                (3.0, 0.0, 0.0),              # Stop
            ]
        else:
            self.get_logger().warn(
                f'Unknown scenario "{scenario}", using default short straight.'
            )
            return [
                (3.0, 0.0, 0.0),
                (8.0, 1.0, 0.0),
                (3.0, 0.0, 0.0),
            ]

    def _elapsed_time(self):
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
            return 0.0
        dt = (now - self.start_time).nanoseconds * 1e-9
        return dt

    def on_timer(self):
        t = self._elapsed_time()

        # After the full scripted duration, just publish zero cmd
        if t > self.total_duration:
            cmd = Twist()
            self.pub_cmd.publish(cmd)
            # Log once and exit the process
            self.get_logger().info('Scenario finished, stopping scripted driver.')
            # Cancel the timer so we don't get called again
            self.timer.cancel()
            # Exit the process – launch will see this
            raise SystemExit(0)

        # Find current segment index
        seg_idx = 0
        for i, t_end in enumerate(self.segment_end_times):
            if t <= t_end:
                seg_idx = i
                break

        dur, v, omega = self.segments[seg_idx]

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ScriptedDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
