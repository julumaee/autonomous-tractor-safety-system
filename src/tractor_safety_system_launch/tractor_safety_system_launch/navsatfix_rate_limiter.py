#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus


class NavSatFixRateLimiter(Node):
    def __init__(self) -> None:
        super().__init__("navsatfix_rate_limiter")

        self.declare_parameter("input_topic", "/gps/fix")
        self.declare_parameter("output_topic", "/gps/fix_ntrip")
        self.declare_parameter("publish_hz", 1.0)
        self.declare_parameter("require_finite_latlon", True)
        self.declare_parameter("require_valid_fix_status", False)
        self.declare_parameter("reject_zero_latlon", True)

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        publish_hz = self.get_parameter("publish_hz").get_parameter_value().double_value
        require_finite_latlon = (
            self.get_parameter("require_finite_latlon").get_parameter_value().bool_value
        )
        require_valid_fix_status = (
            self.get_parameter("require_valid_fix_status").get_parameter_value().bool_value
        )
        reject_zero_latlon = (
            self.get_parameter("reject_zero_latlon").get_parameter_value().bool_value
        )

        if publish_hz <= 0.0:
            raise ValueError("publish_hz must be > 0")

        self._require_finite_latlon = require_finite_latlon
        self._require_valid_fix_status = require_valid_fix_status
        self._reject_zero_latlon = reject_zero_latlon
        self._last_fix: Optional[NavSatFix] = None

        self._pub = self.create_publisher(NavSatFix, output_topic, 10)
        self._sub = self.create_subscription(NavSatFix, input_topic, self._on_fix, 10)

        period = 1.0 / publish_hz
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"Rate-limiting NavSatFix from '{input_topic}' -> '{output_topic}' at {publish_hz} Hz"
        )

    def _on_fix(self, msg: NavSatFix) -> None:
        self._last_fix = msg

    def _on_timer(self) -> None:
        if self._last_fix is None:
            return

        if self._require_valid_fix_status:
            if self._last_fix.status.status == NavSatStatus.STATUS_NO_FIX:
                return

        if self._require_finite_latlon:
            if not (
                math.isfinite(self._last_fix.latitude)
                and math.isfinite(self._last_fix.longitude)
            ):
                return

        if self._reject_zero_latlon:
            if self._last_fix.latitude == 0.0 and self._last_fix.longitude == 0.0:
                return

        self._pub.publish(self._last_fix)


def main() -> None:
    rclpy.init()
    node = NavSatFixRateLimiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
