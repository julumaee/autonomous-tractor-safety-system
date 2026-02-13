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

import can
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Header

from tractor_safety_system_interfaces.msg import RadarDetection


class RadarNode(Node):

    def __init__(self):
        super().__init__("radar_publisher")
        self.publisher_ = self.create_publisher(RadarDetection, "/radar_detections", 10)
        self.declare_parameter("can_channel", "can0")
        self.declare_parameter("frame_timeout", 0.5)  # seconds
        self.declare_parameter("poll_period", 0.01)  # seconds
        self.declare_parameter("min_confidence_percent", 90)   # 0..100
        self.declare_parameter("min_height_m", 0.6)            # meters
        self.declare_parameter("use_abs_height", True)         # safer default

        self.min_confidence_percent = float(self.get_parameter("min_confidence_percent").value)
        self.min_height_m = float(self.get_parameter("min_height_m").value)
        self.use_abs_height = bool(self.get_parameter("use_abs_height").value)

        self.can_channel = self.get_parameter("can_channel").value

        # Set up the CAN interface with correct can channel and socketcan
        self.bus = None
        self._connect_to_can_bus()

        self.frame_buffer = {}
        self.frame_timestamps = {}  # Track when frames arrive for timeout handling
        self.frame_timeout = float(self.get_parameter("frame_timeout").value)
        poll_period = float(self.get_parameter("poll_period").value)
        self.timer = self.create_timer(poll_period, self.poll_can)

        # Subscribe to parameter updates
        self.add_on_set_parameters_callback(self.on_set_parameters)

    def _connect_to_can_bus(self):
        """Connect to the CAN bus."""
        try:
            if self.bus is not None:
                self.bus.shutdown()
            self.bus = can.interface.Bus(
                channel=self.can_channel, interface="socketcan"
            )
            self.get_logger().info(f"Connected to CAN bus on channel {self.can_channel}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to CAN bus: {e}")
            self.bus = None

    def on_set_parameters(self, params):
        """Set parameters their new values."""
        for param in params:
            if param.name == "can_channel":
                self.can_channel = param.value
                self._connect_to_can_bus()
        return SetParametersResult(successful=True)

    def poll_can(self):
        """Poll CAN bus for incoming frames."""
        if self.bus is None:
            return

        # Drain all currently queued frames. Reading just one frame per timer tick
        # can easily drop frames on high-rate radars, causing incomplete pairs.
        while True:
            frame = self.bus.recv(timeout=0.0)
            if frame is None:
                break
            self.process_radar_data(frame)
        # Clean up stale frame buffer entries
        self._cleanup_stale_frames()

    def _decode_confidence_percent(self, frame1: bytes) -> float:
        # Objects_Confid: start bit 56, len 6 => byte7 bits0..5
        raw = frame1[7] & 0x3F
        conf = raw * 5.0
        # clamp to sane percent range
        return max(0.0, min(100.0, conf))

    def process_radar_data(self, frame):
        """Process CAN frame and add to buffer."""
        message_id = frame.arbitration_id
        data = frame.data

        if message_id != 0x701 or len(data) != 8:
            return

        target_id = data[0] & 0x7F
        frame_number = (data[0] & 0x80) >> 7
        current_time = self.get_clock().now().nanoseconds * 1e-9

        if target_id not in self.frame_buffer:
            self.frame_buffer[target_id] = {}

        # Update timestamp on every relevant frame so actively-updating targets
        # don't get cleaned up just because the first half arrived long ago.
        self.frame_timestamps[target_id] = current_time

        if frame_number == 0:
            # Store first part of the trace
            self.frame_buffer[target_id]["frame0"] = data
        elif frame_number == 1:
            # Store second part of the trace (may arrive before frame0)
            self.frame_buffer[target_id]["frame1"] = data

        # Publish once we have both halves, regardless of arrival order.
        buf = self.frame_buffer.get(target_id, {})
        if "frame0" in buf and "frame1" in buf:
            self.publish_radar_detection(target_id)

    def _cleanup_stale_frames(self):
        """Remove incomplete frame pairs that have timed out."""
        current_time = self.get_clock().now().nanoseconds * 1e-9
        stale_targets = []
        for target_id, timestamp in self.frame_timestamps.items():
            if current_time - timestamp > self.frame_timeout:
                stale_targets.append(target_id)
        for target_id in stale_targets:
            del self.frame_buffer[target_id]
            del self.frame_timestamps[target_id]
            self.get_logger().warn(
                f"Removed stale frame pair for target ID {target_id} (timeout)"
            )

    def publish_radar_detection(self, target_id):
        """Combine the two CAN frames into a radar detection and publish."""
        try:
            frame0 = self.frame_buffer[target_id]["frame0"]
            frame1 = self.frame_buffer[target_id]["frame1"]

            # Clean up buffer/timestamp immediately to avoid repeated publishes.
            if target_id in self.frame_buffer:
                del self.frame_buffer[target_id]
            if target_id in self.frame_timestamps:
                del self.frame_timestamps[target_id]

            # Longitudinal distance
            dist_long = ((frame0[1] << 5) | (frame0[2] >> 3)) & 0x1FFF
            dist_long = dist_long * 0.05 - 100

            # Lateral distance
            dist_lat = (((frame0[2] & 0x07) << 8) | frame0[3]) * 0.05 - 50

            # distance = (dist_long ** 2 + dist_lat ** 2) ** 0.5
            # self.get_logger().info(f'Distance={distance:.2f}')  # Debugging info

            # Longitudinal speed
            vrel_long = ((frame0[4] << 2) | (frame0[5] >> 6)) & 0x3FF
            vrel_long = vrel_long * 0.25 - 128

            # Height from frame1
            height = ((frame1[1] << 2) | (frame1[2] >> 6)) & 0x3FF
            height = height * 0.1 - 30

            confidence = self._decode_confidence_percent(frame1)
            # clamp to 0..100 for safety
            confidence = max(0.0, min(100.0, confidence))

            z_for_filter = abs(height) if self.use_abs_height else height

            if confidence < self.min_confidence_percent:
                return

            if z_for_filter < self.min_height_m:
                return

            # Publish message
            radar_detection_msg = RadarDetection()
            radar_detection_msg.header = Header()
            radar_detection_msg.header.frame_id = "radar_link"
            radar_detection_msg.header.stamp = self.get_clock().now().to_msg()

            radar_detection_msg.position.x = dist_long
            radar_detection_msg.position.y = dist_lat
            radar_detection_msg.position.z = height
            radar_detection_msg.speed = vrel_long
            radar_detection_msg.distance = (dist_long**2 + dist_lat**2) ** 0.5

            self.publisher_.publish(radar_detection_msg)
            # self.get_logger().info(
            #     f"Radar target ID={target_id}, Distance={radar_detection_msg.distance}"
            # )

        except Exception as e:
            self.get_logger().error(f"Error publishing radar detection: {e}")


def main(args=None):
    rclpy.init(args=args)
    radar_node = RadarNode()
    rclpy.spin(radar_node)
    radar_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
