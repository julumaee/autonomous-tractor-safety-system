#!/usr/bin/env python3
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
"""Logger for real-world tests (detections, tracks, and GPS truth) to CSV."""

import csv
from datetime import datetime
import math
import os
from typing import Optional

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped as TWCS
from rclpy.node import Node
from ublox_msgs.msg import NavPVT

from tractor_safety_system_interfaces.msg import (
    CameraDetection,
    FusedDetection,
    FusedDetectionArray,
    RadarDetection,
)


class RealWorldLogger(Node):
    def __init__(self):
        super().__init__("real_world_logger_node")

        # Parameters
        self.declare_parameter("test_name", "test_1")
        self.declare_parameter("navpvt_topic", "/navpvt")
        self.declare_parameter("ego_motion_topic", "/ego_motion")

        self.test_name = (
            self.get_parameter("test_name").get_parameter_value().string_value
        )
        self.navpvt_topic = (
            self.get_parameter("navpvt_topic").get_parameter_value().string_value
        )
        self.ego_motion_topic = (
            self.get_parameter("ego_motion_topic").get_parameter_value().string_value
        )

        # Latest GPS truth (from NavPVT)
        self.last_navpvt_rx_time: Optional[float] = None  # logger wall time (seconds)
        self.last_navpvt_i_tow_ms: Optional[int] = None

        self.gt_lat_deg: Optional[float] = None
        self.gt_lon_deg: Optional[float] = None
        self.gt_alt_m: Optional[float] = None

        self.gt_fix_type: Optional[int] = None
        self.gt_num_sv: Optional[int] = None
        self.gt_h_acc_m: Optional[float] = None
        self.gt_v_acc_m: Optional[float] = None
        self.gt_p_dop: Optional[float] = None
        self.gt_g_speed_mps: Optional[float] = None
        self.gt_s_acc_mps: Optional[float] = None
        self.gt_heading_deg: Optional[float] = None
        self.gt_head_acc_deg: Optional[float] = None

        # Ego motion (from /ego_motion)
        self.vx = 0.0
        self.yaw_rate = 0.0
        self.yaw = 0.0
        self.last_ego_time: Optional[float] = None

        # Directory + timestamp for filenames
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = "real_world_logs"
        os.makedirs(log_dir, exist_ok=True)

        self.get_logger().info(f"Logging to directory: {log_dir}")
        self.get_logger().info(f"Test name: {self.test_name}")
        self.get_logger().info(f"NavPVT topic (ground truth): {self.navpvt_topic}")
        self.get_logger().info(f"Ego motion topic: {self.ego_motion_topic}")

        # -----------------------
        # Open CSV files + headers
        # -----------------------

        # Raw camera / radar detections
        self.raw_file = open(
            os.path.join(log_dir, f"raw_detections_{self.test_name}.csv"),
            "w",
            newline="",
        )
        self.raw_writer = csv.writer(self.raw_file)
        self.raw_writer.writerow(
            [
                "sensor_type",    # "radar" or "camera"
                "logger_stamp",   # arrival to logger (seconds)
                "header_stamp",   # detection header stamp (seconds)
                "x", "y", "z",
                f"Test was run at: {timestamp}",
            ]
        )

        # Fused detections
        self.fused_file = open(
            os.path.join(log_dir, f"fused_detections_{self.test_name}.csv"),
            "w",
            newline="",
        )
        self.fused_writer = csv.writer(self.fused_file)
        self.fused_writer.writerow(
            [
                "logger_stamp",
                "header_stamp",
                "detection_type",  # "fused" / "camera" / "radar"
                "x", "y", "z",
                "distance",
                "speed",
                f"Test was run at: {timestamp}",
            ]
        )

        # Tracks
        self.tracks_file = open(
            os.path.join(log_dir, f"tracks_{self.test_name}.csv"),
            "w",
            newline="",
        )
        self.tracks_writer = csv.writer(self.tracks_file)
        self.tracks_writer.writerow(
            [
                "logger_stamp",
                "header_stamp",
                "tracking_id",
                "x", "y", "z",
                "distance",
                "speed",
                "age",
                "consecutive_misses",
                f"Test was run at: {timestamp}",
            ]
        )

        # GPS truth (NavPVT)
        self.gps_truth_file = open(
            os.path.join(log_dir, f"gps_truth_{self.test_name}.csv"),
            "w",
            newline="",
        )
        self.gps_truth_writer = csv.writer(self.gps_truth_file)
        self.gps_truth_writer.writerow(
            [
                "logger_stamp",
                "i_tow_ms",
                "fix_type",
                "flags",
                "flags2",
                "num_sv",
                "p_dop",
                "lat_deg",
                "lon_deg",
                "alt_m",
                "h_acc_m",
                "v_acc_m",
                "vel_n_mps",
                "vel_e_mps",
                "vel_d_mps",
                "g_speed_mps",
                "s_acc_mps",
                "heading_deg",
                "head_acc_deg",
                f"Test was run at: {timestamp}",
            ]
        )

        # Ego motion + GPS truth snapshot
        self.ego_file = open(
            os.path.join(log_dir, f"ego_motion_{self.test_name}.csv"),
            "w",
            newline="",
        )
        self.ego_writer = csv.writer(self.ego_file)
        self.ego_writer.writerow(
            [
                "logger_stamp",
                "header_stamp",
                "v_mps",
                "yaw_rate_radps",
                "yaw_integrated_rad",
                # Latest GNSS truth snapshot (from NavPVT at time of logging)
                "gps_logger_stamp",
                "gps_i_tow_ms",
                "gps_fix_type",
                "gps_num_sv",
                "gps_p_dop",
                "gps_lat_deg",
                "gps_lon_deg",
                "gps_alt_m",
                "gps_h_acc_m",
                "gps_v_acc_m",
                "gps_g_speed_mps",
                "gps_s_acc_mps",
                f"Test was run at: {timestamp}",
            ]
        )

        # -----------------------
        # Subscriptions
        # -----------------------
        self.create_subscription(
            RadarDetection, "/radar_detections", self.handle_radar, 10
        )
        self.create_subscription(
            CameraDetection, "/camera_detections", self.handle_camera, 10
        )
        self.create_subscription(
            FusedDetection, "/fused_detections", self.handle_fused, 10
        )
        self.create_subscription(
            FusedDetectionArray, "/tracked_detections", self.handle_track, 50
        )

        # Ego motion (speed/yaw_rate) from your ego motion node
        self.create_subscription(
            TWCS, self.ego_motion_topic, self.handle_ego_motion, 20
        )

        # GPS truth from NavPVT (u-blox)
        self.create_subscription(
            NavPVT, self.navpvt_topic, self.handle_navpvt, 20
        )

        self.get_logger().info(
            f'RealWorldLogger started, logging to "{log_dir}" for test "{self.test_name}"'
        )

    # -----------------------
    # Time helpers
    # -----------------------
    def _stamp_to_seconds(self, time_msg) -> float:
        return time_msg.sec + time_msg.nanosec * 1e-9

    def _get_current_time(self) -> float:
        now = self.get_clock().now().to_msg()
        return self._stamp_to_seconds(now)

    # -----------------------
    # Callbacks
    # -----------------------
    def handle_radar(self, msg: RadarDetection):
        header_time = self._stamp_to_seconds(msg.header.stamp)
        logger_time = self._get_current_time()
        self.raw_writer.writerow(
            [
                "radar",
                logger_time,
                header_time,
                msg.position.x,
                msg.position.y,
                msg.position.z,
            ]
        )

    def handle_camera(self, msg: CameraDetection):
        header_time = self._stamp_to_seconds(msg.header.stamp)
        logger_time = self._get_current_time()
        self.raw_writer.writerow(
            [
                "camera",
                logger_time,
                header_time,
                msg.position.x,
                msg.position.y,
                msg.position.z,
            ]
        )

    def handle_fused(self, msg: FusedDetection):
        header_time = self._stamp_to_seconds(msg.header.stamp)
        logger_time = self._get_current_time()
        self.fused_writer.writerow(
            [
                logger_time,
                header_time,
                msg.detection_type,
                msg.position.x,
                msg.position.y,
                msg.position.z,
                msg.distance,
                msg.speed,
            ]
        )

    def handle_track(self, msg: FusedDetectionArray):
        for detection in msg.detections:
            header_time = self._stamp_to_seconds(detection.header.stamp)
            logger_time = self._get_current_time()
            self.tracks_writer.writerow(
                [
                    logger_time,
                    header_time,
                    detection.tracking_id,
                    detection.position.x,
                    detection.position.y,
                    detection.position.z,
                    detection.distance,
                    detection.speed,
                    detection.age,
                    detection.consecutive_misses,
                ]
            )

    def handle_navpvt(self, msg: NavPVT):
        """
        Log u-blox NavPVT as ground truth + quality.

        Typical NavPVT units:
          lat/lon: 1e-7 deg
          height: mm
          hAcc/vAcc: mm
          vel_* and gSpeed: mm/s
          heading/headAcc: 1e-5 deg
          pDOP: 0.01
        """
        logger_time = self._get_current_time()

        lat_deg = msg.lat * 1e-7
        lon_deg = msg.lon * 1e-7
        alt_m = msg.height * 1e-3

        h_acc_m = msg.h_acc * 1e-3
        v_acc_m = msg.v_acc * 1e-3

        vel_n_mps = msg.vel_n * 1e-3
        vel_e_mps = msg.vel_e * 1e-3
        vel_d_mps = msg.vel_d * 1e-3
        g_speed_mps = msg.g_speed * 1e-3
        s_acc_mps = msg.s_acc * 1e-3

        heading_deg = msg.heading * 1e-5
        head_acc_deg = msg.head_acc * 1e-5

        p_dop = msg.p_dop * 0.01

        # Update cached "latest truth snapshot"
        self.last_navpvt_rx_time = logger_time
        self.last_navpvt_i_tow_ms = int(msg.i_tow)

        self.gt_lat_deg = float(lat_deg)
        self.gt_lon_deg = float(lon_deg)
        self.gt_alt_m = float(alt_m)

        self.gt_fix_type = int(msg.fix_type)
        self.gt_num_sv = int(msg.num_sv)
        self.gt_h_acc_m = float(h_acc_m)
        self.gt_v_acc_m = float(v_acc_m)
        self.gt_p_dop = float(p_dop)
        self.gt_g_speed_mps = float(g_speed_mps)
        self.gt_s_acc_mps = float(s_acc_mps)
        self.gt_heading_deg = float(heading_deg)
        self.gt_head_acc_deg = float(head_acc_deg)

        # Write GPS truth row
        self.gps_truth_writer.writerow(
            [
                logger_time,
                msg.i_tow,
                msg.fix_type,
                msg.flags,
                msg.flags2,
                msg.num_sv,
                p_dop,
                lat_deg,
                lon_deg,
                alt_m,
                h_acc_m,
                v_acc_m,
                vel_n_mps,
                vel_e_mps,
                vel_d_mps,
                g_speed_mps,
                s_acc_mps,
                heading_deg,
                head_acc_deg,
            ]
        )

    def handle_ego_motion(self, msg: TWCS):
        """Log ego motion and include the latest GNSS truth snapshot from NavPVT."""
        header_time = self._stamp_to_seconds(msg.header.stamp)
        logger_time = self._get_current_time()

        self.vx = msg.twist.twist.linear.x
        self.yaw_rate = msg.twist.twist.angular.z

        # Integrate yaw rate to estimate heading (best-effort; for straight-line
        # tests yaw should be ~0)
        if self.last_ego_time is not None:
            dt = header_time - self.last_ego_time
            if 0.001 < dt < 1.0:
                self.yaw += self.yaw_rate * dt
                self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        self.last_ego_time = header_time
        self.ego_writer.writerow(
            [
                logger_time,
                header_time,
                self.vx,
                self.yaw_rate,
                self.yaw,
            ]
        )

    # -----------------------
    # Cleanup
    # -----------------------
    def destroy_node(self):
        for f in [
            getattr(self, "raw_file", None),
            getattr(self, "fused_file", None),
            getattr(self, "tracks_file", None),
            getattr(self, "gps_truth_file", None),
            getattr(self, "ego_file", None),
        ]:
            if f is not None:
                try:
                    f.close()
                except Exception:
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealWorldLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Logger stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
