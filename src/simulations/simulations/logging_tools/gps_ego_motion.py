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

"""
GPS-based ego motion publisher for tractor testing.

Publishes tractor ego motion (forward speed + yaw rate) on `/ego_motion`.

Inputs (configurable):
- `sensor_msgs/NavSatFix` (position + fix status)
- Optional `geometry_msgs/TwistStamped` (velocity from GPS driver)
- Optional `std_msgs/Float64` heading/course (degrees)

ArduSimple RTK2B notes:
- It is typically used with a u-blox receiver and outputs NMEA.
- Many NMEA setups provide course/heading (RMC/VTG) and speed; these are better
    than estimating from position differences.

Fallback hierarchy:
1) `use_heading_topic:=true`: heading topic drives yaw-rate (best, no drift)
2) `use_velocity_topic:=true`: speed from velocity topic; yaw-rate from course
     inferred from velocity direction
3) Default: estimate speed + course from consecutive NavSatFix positions

Publishing: event-driven (publishes on GPS receipt).
"""

import math

import rclpy
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped as TWCS
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64


class GpsEgoMotion(Node):

    def __init__(self):
        super().__init__("gps_ego_motion")

        # Parameters
        self.declare_parameter("gps_fix_topic", "/gps/fix")
        self.declare_parameter("gps_velocity_topic", "/gps/vel")  # If available
        self.declare_parameter(
            "gps_heading_topic",
            "/gps/heading",
        )  # If available (course/heading in degrees)
        self.declare_parameter(
            "use_velocity_topic", False
        )  # Set true if GPS publishes velocity
        self.declare_parameter(
            "use_heading_topic", False
        )  # Set true if GPS publishes heading/course
        self.declare_parameter("gps_timeout", 1.0)  # Warn if no GPS for this many seconds

        gps_fix_topic = (
            self.get_parameter("gps_fix_topic").get_parameter_value().string_value
        )
        gps_vel_topic = (
            self.get_parameter("gps_velocity_topic").get_parameter_value().string_value
        )
        gps_heading_topic = (
            self.get_parameter("gps_heading_topic").get_parameter_value().string_value
        )
        use_vel_topic = (
            self.get_parameter("use_velocity_topic").get_parameter_value().bool_value
        )
        use_heading_topic = (
            self.get_parameter("use_heading_topic").get_parameter_value().bool_value
        )
        gps_timeout = self.get_parameter("gps_timeout").get_parameter_value().double_value

        # Publisher for ego motion
        self.pub = self.create_publisher(TWCS, "/ego_motion", 50)

        # Subscribe to GPS fix (always needed for position tracking)
        self.create_subscription(NavSatFix, gps_fix_topic, self.gps_fix_callback, 10)

        # Optionally subscribe to GPS velocity topic if available
        if use_vel_topic:
            self.create_subscription(
                TwistStamped, gps_vel_topic, self.gps_vel_callback, 10
            )
            self.get_logger().info(f"Subscribing to GPS velocity topic: {gps_vel_topic}")

        # Optionally subscribe to GPS heading/course topic (best accuracy when available)
        if use_heading_topic:
            self.create_subscription(Float64, gps_heading_topic, self.gps_heading_callback, 10)
            self.get_logger().info(f"Subscribing to GPS heading topic: {gps_heading_topic}")

        self.use_heading_topic = use_heading_topic

        # State variables
        self.vx = 0.0  # forward velocity (m/s)
        self.yaw_rate = 0.0  # yaw rate (rad/s)
        self.current_heading = None  # GPS course/heading in radians
        self.last_heading = None
        self.last_heading_time = None

        # For velocity estimation from position (if no velocity topic)
        self.last_lat = None
        self.last_lon = None
        self.last_position_time = None
        self.use_velocity_from_position = not use_vel_topic

        # GPS watchdog
        self.gps_timeout = gps_timeout
        self.last_gps_time = None
        self.watchdog_timer = self.create_timer(0.5, self.check_gps_timeout)

        # Covariances (tune based on your GPS)
        # Typical GPS velocity accuracy: ~0.1 m/s
        # Typical GPS heading accuracy: ~1-2 degrees when moving
        self.velocity_covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,   # vx variance = 0.01 (±0.1 m/s)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    # vy (not used)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    # vz (not used)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    # wx (not used)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    # wy (not used)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.005,  # wz (yaw_rate) variance = 0.005
        ]

        self.get_logger().info("GPS-based ego motion publisher started")
        self.get_logger().info(f"GPS fix topic: {gps_fix_topic}")
        self.get_logger().info(
            "Using velocity from: "
            f"{'GPS velocity topic' if use_vel_topic else 'position estimates'}"
        )
        self.get_logger().info(
            "Using heading from: "
            f"{'GPS heading topic' if self.use_heading_topic else 'velocity/position estimates'}"
        )
        self.get_logger().info("Publishing ego motion on GPS receipt (event-driven)")

    def gps_fix_callback(self, msg: NavSatFix):
        """
        Process GPS fix message and publish ego motion.

        Extract position and estimate velocity/heading if no dedicated velocity topic.
        Publishes immediately when GPS data is received (event-driven).
        """
        current_time = self.get_clock().now()
        self.last_gps_time = current_time  # Update watchdog

        # Skip if GPS has no fix
        if msg.status.status < 0:  # STATUS_NO_FIX
            self.get_logger().warn("GPS has no fix", throttle_duration_sec=5.0)
            return

        # If we're using position to estimate velocity
        if self.use_velocity_from_position:
            if self.last_lat is not None and self.last_lon is not None:
                # Calculate time delta
                dt = (current_time - self.last_position_time).nanoseconds * 1e-9

                if dt > 0.01:  # Avoid division by zero, need at least 10ms
                    # Calculate distance using Haversine formula
                    dlat = math.radians(msg.latitude - self.last_lat)
                    dlon = math.radians(msg.longitude - self.last_lon)

                    a = (
                        math.sin(dlat / 2) ** 2
                        + math.cos(math.radians(self.last_lat))
                        * math.cos(math.radians(msg.latitude))
                        * math.sin(dlon / 2) ** 2
                    )
                    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

                    # Earth radius in meters
                    R = 6371000.0
                    distance = R * c

                    # Estimate velocity
                    estimated_vx = distance / dt

                    # Only update if moving (avoid noise when stationary)
                    if estimated_vx > 0.2:  # 0.2 m/s threshold
                        self.vx = estimated_vx

                        # If no dedicated heading topic is available, infer course from position
                        # delta.
                        if not self.use_heading_topic:
                            heading = math.atan2(
                                math.sin(dlon) * math.cos(math.radians(msg.latitude)),
                                math.cos(math.radians(self.last_lat))
                                * math.sin(math.radians(msg.latitude))
                                - math.sin(math.radians(self.last_lat))
                                * math.cos(math.radians(msg.latitude))
                                * math.cos(dlon),
                            )
                            self.current_heading = heading

                            # Calculate yaw rate from heading change
                            if self.last_heading is not None:
                                dheading = self._angle_diff(heading, self.last_heading)
                                heading_dt = (
                                    (current_time - self.last_heading_time).nanoseconds
                                    * 1e-9
                                )
                                if heading_dt > 0.01:
                                    self.yaw_rate = dheading / heading_dt

                            self.last_heading = heading
                            self.last_heading_time = current_time
                    else:
                        # Stationary - zero out velocity and yaw rate
                        self.vx = 0.0
                        self.yaw_rate = 0.0

            # Update position history
            self.last_lat = msg.latitude
            self.last_lon = msg.longitude
            self.last_position_time = current_time

            # Publish ego motion immediately after processing GPS fix
            self.publish_ego_motion()

    def gps_heading_callback(self, msg: Float64):
        r"""
        Process GPS heading/course message.

        Expects degrees where 0°=North and 90°=East (common NMEA course-over-ground).
        Converts to ROS yaw in ENU: psi = deg2rad(90 - heading).

        Course/heading is unreliable when stationary, so updates are gated using the
        current speed estimate (vx).
        """
        current_time = self.get_clock().now()
        self.last_gps_time = current_time  # Update watchdog

        # Gate heading when nearly stationary (course over ground becomes noisy)
        if self.vx <= 0.2:
            self.yaw_rate = 0.0
            self.publish_ego_motion()
            return

        heading_deg = float(msg.data)
        yaw = math.radians(90.0 - heading_deg)

        # Normalize yaw
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        self.current_heading = yaw

        # Compute yaw rate from heading change
        if self.last_heading is not None:
            dyaw = self._angle_diff(yaw, self.last_heading)
            dt = (current_time - self.last_heading_time).nanoseconds * 1e-9
            if dt > 0.01:
                self.yaw_rate = dyaw / dt

        self.last_heading = yaw
        self.last_heading_time = current_time

        self.publish_ego_motion()

    def gps_vel_callback(self, msg: TwistStamped):
        """
        Process GPS velocity message and publish ego motion.

        This is more accurate than estimating from position changes.
        Publishes immediately when GPS velocity is received (event-driven).
        """
        current_time = self.get_clock().now()
        self.last_gps_time = current_time  # Update watchdog

        # Extract forward velocity (assuming GPS velocity is in vehicle frame)
        # Some GPS drivers publish velocity in ENU frame, adjust if needed
        self.vx = math.sqrt(msg.twist.linear.x**2 + msg.twist.linear.y**2)

        # Stationary handling
        if self.vx <= 0.2:
            self.vx = 0.0
            self.yaw_rate = 0.0
            self.publish_ego_motion()
            return

        # If a dedicated heading topic exists, it drives yaw/yaw_rate.
        if not self.use_heading_topic:
            # Calculate heading from velocity vector
            heading = math.atan2(msg.twist.linear.y, msg.twist.linear.x)
            self.current_heading = heading

            # Calculate yaw rate from heading change
            if self.last_heading is not None:
                dheading = self._angle_diff(heading, self.last_heading)
                dt = (current_time - self.last_heading_time).nanoseconds * 1e-9
                if dt > 0.01:
                    self.yaw_rate = dheading / dt

            self.last_heading = heading
            self.last_heading_time = current_time

        # Publish ego motion immediately after processing GPS velocity
        self.publish_ego_motion()

    def check_gps_timeout(self):
        """Watchdog timer to detect GPS data loss."""
        if self.last_gps_time is not None:
            time_since_gps = (
                (self.get_clock().now() - self.last_gps_time).nanoseconds * 1e-9
            )
            if time_since_gps > self.gps_timeout:
                self.get_logger().warn(
                    f"No GPS data for {time_since_gps:.1f}s! Check GPS connection.",
                    throttle_duration_sec=5.0,
                )

    def publish_ego_motion(self):
        """Publish current ego motion estimate."""
        msg = TWCS()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        # Linear velocity (only forward component matters for ground vehicles)
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0

        # Angular velocity (only yaw rate matters for ground vehicles)
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = self.yaw_rate

        # Add covariance
        msg.twist.covariance = self.velocity_covariance

        self.pub.publish(msg)

    def _angle_diff(self, a, b):
        """Calculate the shortest difference between two angles."""
        diff = a - b
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff


def main(args=None):
    rclpy.init(args=args)
    node = GpsEgoMotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("GPS ego motion publisher stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
