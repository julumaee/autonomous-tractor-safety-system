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

"""
Sensor Calibration Script for Autonomous Tractor Safety System

This script helps calibrate radar and camera sensors by collecting data
from known reference positions and calculating optimal TF transform parameters.

Usage:
    1. Place calibration targets at known distances (5m, 10m, 20m)
    2. Run this script to collect sensor data
    3. Script calculates optimal sensor pose transforms (x, y, z, roll, pitch, yaw)
    
    # Run calibration data collection
    python3 sensor_calibration.py --mode collect --duration 30
    
    # Analyze collected data and calculate transforms
    python3 sensor_calibration.py --mode analyze --data calibration_data.csv
    
    # Run full calibration (collect + analyze)
    python3 sensor_calibration.py --mode full --duration 30    
    # Sequential calibration (one target at a time)
    python3 sensor_calibration.py --mode collect --duration 10 --target-name target1
    python3 sensor_calibration.py --mode collect --duration 10 --target-name target2 --append
    python3 sensor_calibration.py --mode analyze --data calibration_log/calibration_raw_combined.csv"""

import argparse
import csv
import json
import os
import time
from datetime import datetime
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from tractor_safety_system_interfaces.msg import CameraDetection, RadarDetection

try:
    from ublox_msgs.msg import NavPVT
except Exception:  # pragma: no cover
    NavPVT = None


class CalibrationDataCollector(Node):
    """Collects sensor data for calibration."""

    def __init__(
        self,
        duration: int = 30,
        target_name: str = None,
        append_mode: bool = False,
        start_delay_s: float = 0.0,
        gps_topic: str = "/navpvt",
        gps_origin_out: str = None,
        gps_log_out: str = None,
        write_gps_origin: bool = True,
        log_gps_samples: bool = True,
    ):
        super().__init__("calibration_data_collector")

        self.duration = duration
        self.start_delay_s = max(0.0, float(start_delay_s))
        self._warmup_end_wall_time = time.time() + self.start_delay_s
        self.start_time: float | None = None
        self.target_name = target_name or "unknown"

        # Create output directory
        self.output_dir = "calibration_log"
        os.makedirs(self.output_dir, exist_ok=True)

        # GPS origin + optional GPS log
        self.gps_topic = str(gps_topic or "/navpvt")
        self.gps_origin_out = gps_origin_out or os.path.join(
            self.output_dir, "gps_origin.json"
        )
        self.gps_log_out = gps_log_out or os.path.join(
            self.output_dir, "navpvt_log.csv"
        )
        self.write_gps_origin = bool(write_gps_origin)
        self.log_gps_samples = bool(log_gps_samples)
        self._latest_navpvt = None
        self._gps_origin_written = False
        self._gps_log_file = None
        self._gps_log_writer = None

        # Determine filename based on mode
        if append_mode:
            # Append to combined file
            self.filename = os.path.join(
                self.output_dir, "calibration_raw_combined.csv"
            )
            file_exists = os.path.exists(self.filename)
            self.file = open(self.filename, mode="a", newline="")
            self.writer = csv.writer(self.file)
            # Only write header if file is new
            if not file_exists:
                self.writer.writerow([
                    "timestamp", "sensor_type", "x", "y", "z", "frame_id", "target_name"
                ])
        else:
            # Create new file with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.filename = os.path.join(
                self.output_dir, f"calibration_raw_{timestamp}.csv"
            )
            self.file = open(self.filename, mode="w", newline="")
            self.writer = csv.writer(self.file)
            self.writer.writerow([
                "timestamp", "sensor_type", "x", "y", "z", "frame_id", "target_name"
            ])

        # Subscribers
        self.radar_sub = self.create_subscription(
            RadarDetection, "/radar_detections", self.radar_callback, 10
        )
        self.camera_sub = self.create_subscription(
            CameraDetection, "/camera_detections", self.camera_callback, 10
        )

        if NavPVT is not None:
            self.navpvt_sub = self.create_subscription(
                NavPVT, self.gps_topic, self.navpvt_callback, 10
            )
        else:
            self.navpvt_sub = None
            self.get_logger().warn(
                "ublox_msgs/NavPVT is not available; GPS origin will not be logged"
            )

        # Counters
        self.radar_count = 0
        self.camera_count = 0

        # Timer to check duration
        self.timer = self.create_timer(1.0, self.check_duration)

        if self.start_delay_s > 0.0:
            self.get_logger().info(
                f"Warmup: logging will start in {self.start_delay_s:.1f}s"
            )
        self.get_logger().info(f"Logging duration: {duration} seconds")
        self.get_logger().info(f"Target: {self.target_name}")
        self.get_logger().info(f"Output file: {self.filename}")
        if self.navpvt_sub is not None:
            self.get_logger().info(
                f"GPS: subscribing to {self.gps_topic}; origin -> {self.gps_origin_out}"
            )
            if self.log_gps_samples:
                self.get_logger().info(f"GPS log -> {self.gps_log_out}")
        if append_mode:
            self.get_logger().info("APPEND MODE: Data will be added to existing file")
        self.get_logger().info(
            "Keep target stationary at the designated position"
        )

        if self.navpvt_sub is not None and self.log_gps_samples:
            self._open_gps_log()

    def _open_gps_log(self) -> None:
        os.makedirs(os.path.dirname(self.gps_log_out) or ".", exist_ok=True)
        file_exists = os.path.exists(self.gps_log_out)
        self._gps_log_file = open(self.gps_log_out, mode="a", newline="")
        self._gps_log_writer = csv.writer(self._gps_log_file)
        if not file_exists:
            self._gps_log_writer.writerow(
                [
                    "timestamp",
                    "i_tow_ms",
                    "lat_deg",
                    "lon_deg",
                    "height_m",
                    "h_msl_m",
                    "fix_type",
                    "flags",
                    "num_sv",
                    "h_acc_m",
                    "v_acc_m",
                ]
            )

    @staticmethod
    def _navpvt_to_origin_dict(msg) -> dict:
        lat_deg = float(msg.lat) * 1e-7
        lon_deg = float(msg.lon) * 1e-7
        return {
            "source": "ublox_msgs/NavPVT",
            "lat_deg": lat_deg,
            "lon_deg": lon_deg,
            "height_m": float(msg.height) * 1e-3,
            "h_msl_m": float(msg.h_msl) * 1e-3,
            "fix_type": int(msg.fix_type),
            "flags": int(msg.flags),
            "num_sv": int(msg.num_sv),
            "h_acc_m": float(msg.h_acc) * 1e-3,
            "v_acc_m": float(msg.v_acc) * 1e-3,
            "i_tow_ms": int(msg.i_tow),
        }

    def _maybe_write_gps_origin(self) -> None:
        if (
            not self.write_gps_origin
            or self._gps_origin_written
            or self.navpvt_sub is None
        ):
            return
        if self._latest_navpvt is None:
            self.get_logger().warn(
                "No /navpvt received yet; GPS origin not written"
            )
            return

        origin = self._navpvt_to_origin_dict(self._latest_navpvt)
        origin["topic"] = self.gps_topic
        origin["generated"] = datetime.now().isoformat(timespec="seconds")
        origin["ros_time_sec"] = float(self.get_clock().now().nanoseconds) * 1e-9

        os.makedirs(os.path.dirname(self.gps_origin_out) or ".", exist_ok=True)
        with open(self.gps_origin_out, "w", encoding="utf-8") as f:
            json.dump(origin, f, indent=2, sort_keys=True)
            f.write("\n")

        self._gps_origin_written = True
        self.get_logger().info(
            "GPS origin saved: "
            f"lat={origin['lat_deg']:.7f}, lon={origin['lon_deg']:.7f}, "
            f"h_msl={origin['h_msl_m']:.3f}m (fix_type={origin['fix_type']})"
        )

    def navpvt_callback(self, msg):
        self._latest_navpvt = msg
        if self._gps_log_writer is None:
            return
        timestamp = self.get_clock().now().nanoseconds / 1e9
        row = self._navpvt_to_origin_dict(msg)
        self._gps_log_writer.writerow(
            [
                timestamp,
                row["i_tow_ms"],
                row["lat_deg"],
                row["lon_deg"],
                row["height_m"],
                row["h_msl_m"],
                row["fix_type"],
                row["flags"],
                row["num_sv"],
                row["h_acc_m"],
                row["v_acc_m"],
            ]
        )

    def destroy_node(self):
        data_file = getattr(self, "file", None)
        if data_file is not None:
            try:
                if not data_file.closed:
                    data_file.close()
            except Exception:
                pass

        gps_file = getattr(self, "_gps_log_file", None)
        if gps_file is not None:
            try:
                if not gps_file.closed:
                    gps_file.close()
            except Exception:
                pass

        return super().destroy_node()

    def _logging_active(self) -> bool:
        return self.start_time is not None

    def _maybe_start_logging(self) -> None:
        if self._logging_active():
            return

        if time.time() < self._warmup_end_wall_time:
            return

        self.start_time = time.time()
        self._maybe_write_gps_origin()
        self.get_logger().info(
            f"Logging started. Collecting for {self.duration}s..."
        )

    def radar_callback(self, msg: RadarDetection):
        """Handle radar detections."""
        self._maybe_start_logging()
        if not self._logging_active():
            return
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.writer.writerow([
            timestamp,
            "radar",
            msg.position.x,
            msg.position.y,
            msg.position.z,
            msg.header.frame_id,
            self.target_name,
        ])
        self.radar_count += 1

    def camera_callback(self, msg: CameraDetection):
        """Handle camera detections."""
        self._maybe_start_logging()
        if not self._logging_active():
            return
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.writer.writerow([
            timestamp,
            "camera",
            msg.position.x,
            msg.position.y,
            msg.position.z,
            msg.header.frame_id,
            self.target_name,
        ])
        self.camera_count += 1

    def check_duration(self):
        """Check if collection duration has elapsed."""
        if not self._logging_active():
            remaining_warmup = self._warmup_end_wall_time - time.time()
            if remaining_warmup > 0:
                self.get_logger().info(
                    f"Warmup... {remaining_warmup:.0f}s until logging starts | "
                    f"Radar: {self.radar_count} | Camera: {self.camera_count}"
                )
                return

            self._maybe_start_logging()
            return

        elapsed = time.time() - self.start_time
        remaining = self.duration - elapsed

        if remaining > 0:
            self.get_logger().info(
                f"Collecting... {remaining:.0f}s remaining | "
                f"Radar: {self.radar_count} | Camera: {self.camera_count}"
            )
        else:
            self.get_logger().info(
                f"Collection complete! Radar: {self.radar_count}, "
                f"Camera: {self.camera_count}"
            )
            self.file.close()
            if self._gps_log_file is not None:
                self._gps_log_file.close()
            self.get_logger().info(f"Data saved to: {self.filename}")
            raise SystemExit


class CalibrationAnalyzer:
    """Analyzes calibration data and calculates optimal transforms."""

    def __init__(
        self,
        data_file: str,
        ground_truth_file: str = None,
        *,
        radar_fit_mode: str = "6dof",
        radar_gate_factor: float = 0.2,
        camera_gate_factor: float = 0.3,
        radar_gate_min_m: float = 0.0,
        camera_gate_min_m: float = 0.0,
        exclude_camera_targets: list[str] | None = None,
        radar_y_sign: float = 1.0,
        radar_ground_truth_y_sign: float = 1.0,
    ):
        self.data_file = data_file
        self.ground_truth_file = ground_truth_file

        radar_fit_mode = str(radar_fit_mode).strip().lower()
        if radar_fit_mode not in ("6dof", "planar"):
            raise ValueError(f"radar_fit_mode must be '6dof' or 'planar' (got '{radar_fit_mode}')")
        self.radar_fit_mode = radar_fit_mode

        self.radar_gate_factor = float(radar_gate_factor)
        self.camera_gate_factor = float(camera_gate_factor)
        self.radar_gate_min_m = float(radar_gate_min_m)
        self.camera_gate_min_m = float(camera_gate_min_m)

        exclude_camera_targets = exclude_camera_targets or []
        self.exclude_camera_targets_raw = [str(x).strip() for x in exclude_camera_targets if str(x).strip()]
        self.exclude_camera_targets_norm = {
            self._normalize_label(x) for x in self.exclude_camera_targets_raw
        }
        self.radar_y_sign = float(radar_y_sign)
        self.radar_ground_truth_y_sign = float(radar_ground_truth_y_sign)

        # If the CSV contains a 'target_name' column (sequential calibration),
        # we keep points grouped by target for deterministic analysis.
        self.target_order: List[str] = []
        self.radar_points_by_target: dict[str, list[np.ndarray]] = {}
        self.camera_points_by_target: dict[str, list[np.ndarray]] = {}

        self.ground_truth_by_name: dict[str, np.ndarray] = {}
        self.ground_truth_order: list[str] = []

        # Optional initial guesses for transforms (can be set externally)
        self.radar_init_translation: np.ndarray | None = None
        self.radar_init_rotation: np.ndarray | None = None
        self.camera_init_translation: np.ndarray | None = None
        self.camera_init_rotation: np.ndarray | None = None

        # Load ground truth positions if provided (needed for camera frame auto-detection)
        self.ground_truth = []
        if ground_truth_file and os.path.exists(ground_truth_file):
            self.load_ground_truth()

        # Load sensor data
        self.radar_data = []
        self.camera_data = []
        self._camera_frame_ids: set[str] = set()
        self._camera_points_were_optical: bool | None = None
        self.load_sensor_data()

    def _is_excluded_camera_target(self, target_name: str) -> bool:
        if not target_name:
            return False
        t = str(target_name).strip()
        if not t:
            return False
        if t in self.exclude_camera_targets_raw:
            return True
        return self._normalize_label(t) in self.exclude_camera_targets_norm

    def load_sensor_data(self):
        """Load sensor data from CSV file."""
        print(f"Loading sensor data from {self.data_file}...")

        if self.exclude_camera_targets_raw:
            pretty = ", ".join(self.exclude_camera_targets_raw)
            print(f"[INFO] Excluding CAMERA detections for target(s): {pretty}")

        has_target_name = False
        camera_raw_rows: list[tuple[np.ndarray, str, str]] = []
        camera_raw_by_target: dict[str, list[np.ndarray]] = {}
        skipped_camera = 0
        with open(self.data_file, 'r') as f:
            reader = csv.DictReader(f)
            if reader.fieldnames and 'target_name' in reader.fieldnames:
                has_target_name = True

            for row in reader:
                point = np.array([
                    float(row['x']),
                    float(row['y']),
                    float(row['z'])
                ])
                sensor_type = row.get('sensor_type', '')
                frame_id = (row.get('frame_id') or '').strip()
                target_name = (row.get('target_name') or '').strip() if has_target_name else ''
                if has_target_name and target_name and target_name not in self.target_order:
                    self.target_order.append(target_name)

                if sensor_type == 'radar':
                    # Some logs may have radar lateral axis opposite of ROS base convention.
                    # Use radar_y_sign=-1 to flip y for historical logs without modifying the CSV.
                    point_r = point.copy()
                    point_r[1] = self.radar_y_sign * point_r[1]
                    self.radar_data.append(point_r)
                    if has_target_name and target_name:
                        self.radar_points_by_target.setdefault(target_name, []).append(point_r)
                elif sensor_type == 'camera':
                    if has_target_name and target_name and self._is_excluded_camera_target(target_name):
                        skipped_camera += 1
                        continue
                    self._camera_frame_ids.add(frame_id or 'UNKNOWN')
                    camera_raw_rows.append((point, frame_id, target_name))
                    if has_target_name and target_name:
                        camera_raw_by_target.setdefault(target_name, []).append(point)

        # Post-process camera rows.
        # Current convention: camera detections are logged in camera_link
        # (x forward, y left, z up). The optical->camera_link axis swap is handled
        # in camera_interface/camera_node.py.
        #
        # However, older logs may contain optical-frame coordinates even if the frame_id
        # column says camera_link (e.g., collected after a frame rename but before the
        # axis swap was implemented). We auto-detect and convert those logs.
        if len(camera_raw_rows) > 0:
            raw_points = np.asarray([p for (p, _fid, _tn) in camera_raw_rows], dtype=float)
            med_abs = np.median(np.abs(raw_points), axis=0)

            camera_frame_ids = {(_fid or '').strip().lower() for (_p, _fid, _tn) in camera_raw_rows}
            mentions_optical = any('optical' in fid for fid in camera_frame_ids)

            # Heuristic:
            # - Optical: z (forward range) dominates, x (right) is smaller.
            # - camera_link: x (forward range) dominates.
            looks_optical = bool(med_abs[2] > 1.5 * med_abs[0])

            self._camera_points_were_optical = mentions_optical or looks_optical
            if self._camera_points_were_optical:
                print(
                    "[INFO] Camera CSV points appear to be optical-frame; converting to camera_link axes for fitting"
                )
            else:
                print("[INFO] Camera CSV points treated as camera_link axes")

            for raw_point, _frame_id, target_name in camera_raw_rows:
                if self._camera_points_were_optical:
                    x_c, y_c, z_c = raw_point
                    point_to_store = np.array([z_c, -x_c, -y_c], dtype=float)
                else:
                    point_to_store = raw_point.astype(float)

                self.camera_data.append(point_to_store)
                if has_target_name and target_name:
                    self.camera_points_by_target.setdefault(target_name, []).append(point_to_store)

        print(f"Loaded {len(self.radar_data)} radar points")
        print(f"Loaded {len(self.camera_data)} camera points")
        if skipped_camera > 0:
            print(f"[INFO] Skipped {skipped_camera} camera row(s) due to --exclude-camera-target")

        if len(self._camera_frame_ids) > 0:
            pretty = ", ".join(sorted(self._camera_frame_ids))
            print(f"Camera frame_id(s) in CSV: {pretty}")

        if has_target_name and len(self.target_order) > 0:
            print("Sequential calibration data detected (target_name column present)")
            print(f"Detected targets in collection order: {self.target_order}")
        else:
            print("Single-session calibration data (no target_name column)")

    def is_sequential(self) -> bool:
        return len(self.target_order) > 0

    def save_tf_results(self, radar_result, camera_result, out_path: str) -> None:
        os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)

        lines = []
        lines.append("# Sensor calibration TF output")
        lines.append(f"# data_file: {self.data_file}")
        lines.append(f"# generated: {datetime.now().isoformat(timespec='seconds')}")
        lines.append("")

        def _invert_transform(translation: np.ndarray, rotation_rpy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
            """Invert transform defined as: p_base = R * p_sensor + t.

            Returns (t_inv, rpy_inv) for: p_sensor = R_inv * p_base + t_inv.
            """
            R = Rotation.from_euler('xyz', rotation_rpy).as_matrix()
            R_inv = R.T
            t_inv = -(R_inv @ np.asarray(translation, dtype=float))
            rpy_inv = Rotation.from_matrix(R_inv).as_euler('xyz')
            return t_inv, rpy_inv

        if radar_result and "translation" in radar_result and "rotation" in radar_result:
            # radar_result is radar_link -> base_link (sensor->base). For runtime TF publishing,
            # launch_test.sh expects base_link -> radar_link.
            t_rb = np.asarray(radar_result["translation"], dtype=float)
            r_rb = np.asarray(radar_result["rotation"], dtype=float)
            t_br, r_br = _invert_transform(t_rb, r_rb)

            # Legacy format (consumed by launch_test.sh): base_link -> radar_link
            lines.append(
                "radar_tf: {:.6f} {:.6f} {:.6f} {:.8f} {:.8f} {:.8f}".format(
                    t_br[0], t_br[1], t_br[2], r_br[0], r_br[1], r_br[2]
                )
            )

        if camera_result and "translation" in camera_result and "rotation" in camera_result:
            # camera_result is camera_link -> base_link (sensor->base).
            # For runtime TF publishing, launch_test.sh expects base_link -> camera_link.
            t_cb = np.asarray(camera_result["translation"], dtype=float)
            r_cb = np.asarray(camera_result["rotation"], dtype=float)
            t_bc, r_bc = _invert_transform(t_cb, r_cb)

            # Legacy format (consumed by launch_test.sh): base_link -> camera_link
            lines.append(
                "camera_tf: {:.6f} {:.6f} {:.6f} {:.8f} {:.8f} {:.8f}".format(
                    t_bc[0], t_bc[1], t_bc[2], r_bc[0], r_bc[1], r_bc[2]
                )
            )

        with open(out_path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")

        print(f"\n[INFO] Saved TF results to: {out_path}")

    @staticmethod
    def _normalize_label(label: str) -> str:
        return "_".join(label.strip().lower().split())

    def _lookup_ground_truth(self, target_name: str) -> np.ndarray | None:
        if not target_name:
            return None
        direct = self.ground_truth_by_name.get(target_name)
        if direct is not None:
            return direct
        normalized = self._normalize_label(target_name)
        for key, value in self.ground_truth_by_name.items():
            if self._normalize_label(key) == normalized:
                return value
        return None

    def _gate_detections_by_distance(
        self,
        target_name: str,
        points: list[np.ndarray],
        *,
        gate_factor: float,
        gate_min_m: float,
        ground_truth_y_sign: float = 1.0,
        initial_translation: np.ndarray | None = None,
        initial_rotation: np.ndarray | None = None,
    ) -> list[np.ndarray]:
        """Distance-adaptive gating against ground truth.

        Keeps detections where:
          ||point - expected|| <= gate_factor * ||expected||

        This provides distance-adaptive gating (tighter at close range, looser at far range).
        """
        if len(points) == 0:
            return []
        
        # Get expected ground truth position for this target
        expected_pos = self._lookup_ground_truth(target_name)
        if expected_pos is None:
            # No ground truth for this target, return all points
            return points

        ground_truth_y_sign = float(ground_truth_y_sign)
        if ground_truth_y_sign not in (-1.0, 1.0):
            # Allow any float, but keep it explicit in case someone passes e.g. 0
            # which would collapse the y dimension.
            raise ValueError(f"ground_truth_y_sign must be +/-1 (got {ground_truth_y_sign})")

        expected_pos = expected_pos.copy()
        expected_pos[1] = ground_truth_y_sign * expected_pos[1]
        
        # Calculate distance from origin to ground truth
        distance_to_truth = np.linalg.norm(expected_pos)
        
        gate_factor = float(gate_factor)
        gate_min_m = float(gate_min_m)
        if gate_factor <= 0.0:
            return points

        # Gating threshold: gate_factor * distance to ground truth
        threshold = gate_factor * distance_to_truth
        if gate_min_m > 0.0:
            threshold = max(threshold, gate_min_m)
        
        # Filter points
        gated_points = []
        min_dist = None
        use_init = initial_translation is not None or initial_rotation is not None
        if use_init:
            t0 = (
                np.array(initial_translation, dtype=float)
                if initial_translation is not None
                else np.array([0.0, 0.0, 0.0], dtype=float)
            )
            r0 = (
                np.array(initial_rotation, dtype=float)
                if initial_rotation is not None
                else np.array([0.0, 0.0, 0.0], dtype=float)
            )
            R0 = Rotation.from_euler('xyz', r0).as_matrix()

        for point in points:
            # Gate in base_link space if an initial guess is available.
            # This avoids accidentally gating out the true target cluster when the
            # sensor origin is offset/rotated from base_link.
            if use_init:
                point_for_gate = (R0 @ point) + t0
            else:
                point_for_gate = point

            distance_from_expected = np.linalg.norm(point_for_gate - expected_pos)
            if min_dist is None or distance_from_expected < min_dist:
                min_dist = float(distance_from_expected)
            if distance_from_expected <= threshold:
                gated_points.append(point)

        if len(gated_points) == 0 and min_dist is not None:
            print(
                f"  [gate] '{target_name}' gated out: min_dist={min_dist:.2f}m "
                f"> threshold={threshold:.2f}m (factor={gate_factor:.3f}, min_m={gate_min_m:.2f}, range={distance_to_truth:.2f}m)"
            )
        
        return gated_points

    def _centroids_from_targets_with_names(
        self,
        points_by_target: dict[str, list[np.ndarray]],
        *,
        gate_factor: float,
        gate_min_m: float,
        ground_truth_y_sign: float = 1.0,
        initial_translation: np.ndarray | None = None,
        initial_rotation: np.ndarray | None = None,
    ) -> tuple[list[np.ndarray], list[str]]:
        """Compute one centroid per target using distance-based gating."""
        centroids: list[np.ndarray] = []
        names: list[str] = []
        for target_name in self.target_order:
            points = points_by_target.get(target_name, [])
            if len(points) == 0:
                continue
            
            # Apply distance-based gating
            gated_points = self._gate_detections_by_distance(
                target_name,
                points,
                gate_factor=float(gate_factor),
                gate_min_m=float(gate_min_m),
                ground_truth_y_sign=float(ground_truth_y_sign),
                initial_translation=initial_translation,
                initial_rotation=initial_rotation,
            )
            
            if len(gated_points) == 0:
                print(f"Warning: '{target_name}' has no detections after gating")
                continue
            
            # Calculate centroid of gated points
            centroid = np.mean(np.array(gated_points), axis=0)
            
            # Report gating effectiveness
            filtered_ratio = 100 * len(gated_points) / len(points)
            print(f"  {target_name}: {len(gated_points)}/{len(points)} detections kept ({filtered_ratio:.1f}%)")
            
            centroids.append(centroid)
            names.append(target_name)
        return centroids, names

    def load_ground_truth(self):
        """Load ground truth target positions from CSV file."""
        print(f"\nLoading ground truth from {self.ground_truth_file}...")

        with open(self.ground_truth_file, 'r') as f:
            reader = csv.DictReader(f)
            for idx, row in enumerate(reader):
                point = np.array([
                    float(row['x']),
                    float(row['y']),
                    float(row['z'])
                ])
                self.ground_truth.append(point)

                label = (row.get('target_name') or row.get('description') or '').strip()
                if not label:
                    label = f"target_{idx+1}"
                self.ground_truth_order.append(label)
                # Keep the first occurrence if duplicates exist.
                self.ground_truth_by_name.setdefault(label, point)

        print(f"Loaded {len(self.ground_truth)} ground truth points")
        if len(self.ground_truth_by_name) > 0:
            print(f"Ground truth labels: {self.ground_truth_order}")

    def calculate_sensor_offset(
        self, 
        sensor_points: List[np.ndarray],
        reference_points: List[np.ndarray],
        initial_translation: np.ndarray = None,
        initial_rotation: np.ndarray = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Calculate sensor offset and orientation using point matching.

        This uses ICP-like approach to find the best transformation that
        aligns sensor measurements with reference positions.

        Args:
            sensor_points: Points detected by sensor (in sensor frame)
            reference_points: Known reference positions (in base_link frame)
            initial_translation: Optional initial guess for [x, y, z] in meters
            initial_rotation: Optional initial guess for [roll, pitch, yaw] in radians

        Returns:
            translation: [x, y, z] offset in meters
            rotation: [roll, pitch, yaw] in radians
        """
        if len(sensor_points) != len(reference_points):
            print(f"Warning: Point count mismatch - sensor: {len(sensor_points)}, "
                  f"reference: {len(reference_points)}")
            # Match the smaller set
            min_len = min(len(sensor_points), len(reference_points))
            sensor_points = sensor_points[:min_len]
            reference_points = reference_points[:min_len]

        sensor_array = np.array(sensor_points)
        reference_array = np.array(reference_points)

        def transform_error(params):
            """Calculate error for given transform parameters."""
            tx, ty, tz, roll, pitch, yaw = params

            # Create transformation matrix
            R = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
            t = np.array([tx, ty, tz])

            # Transform sensor points to base frame
            transformed = (R @ sensor_array.T).T + t

            # Calculate error
            error = transformed - reference_array
            return error.flatten()

        # Initial guess: [x, y, z, roll, pitch, yaw]
        # Use provided translation if given, otherwise defaults to zero
        if initial_translation is not None:
            x0_trans = initial_translation
        else:
            x0_trans = np.array([0.0, 0.0, 0.0])
        
        # Use provided rotation if given, otherwise defaults to zero
        if initial_rotation is not None:
            x0_rot = initial_rotation
        else:
            x0_rot = np.array([0.0, 0.0, 0.0])
        
        x0 = np.concatenate([x0_trans, x0_rot])

        # Optimize
        result = least_squares(transform_error, x0, method='lm')

        translation = result.x[:3]
        rotation = result.x[3:]

        # Calculate final error
        final_error = np.linalg.norm(transform_error(result.x))
        avg_error = final_error / len(sensor_points)

        print(f"  Optimization converged: {result.success}")
        print(f"  Average error: {avg_error:.3f} m")

        return translation, rotation

    @staticmethod
    def calculate_sensor_offset_planar(
        sensor_points: List[np.ndarray],
        reference_points: List[np.ndarray],
        *,
        initial_translation: np.ndarray | None = None,
        initial_rotation: np.ndarray | None = None,
        robust: bool = True,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Planar calibration: solve only (x, y, yaw).

        Fits a transform of the form:
          p_base_xy = Rz(yaw) * p_sensor_xy + [tx, ty]

        Returns a 6-DOF-shaped output compatible with existing code:
          translation = [tx, ty, tz]
          rotation_rpy = [0.0, 0.0, yaw]

        Notes:
        - This is often more stable than a full 6-DOF fit when targets are
          mostly coplanar / ground-truth z is unreliable.
        """
        if len(sensor_points) != len(reference_points):
            min_len = min(len(sensor_points), len(reference_points))
            sensor_points = sensor_points[:min_len]
            reference_points = reference_points[:min_len]

        sensor_array = np.asarray(sensor_points, dtype=float)
        reference_array = np.asarray(reference_points, dtype=float)
        sensor_xy = sensor_array[:, :2]
        ref_xy = reference_array[:, :2]

        if initial_translation is not None:
            t0 = np.asarray(initial_translation, dtype=float)
            tx0, ty0, tz0 = float(t0[0]), float(t0[1]), float(t0[2])
        else:
            tx0, ty0, tz0 = 0.0, 0.0, 0.0

        yaw0 = 0.0
        if initial_rotation is not None:
            r0 = np.asarray(initial_rotation, dtype=float)
            if r0.shape[0] >= 3:
                yaw0 = float(r0[2])

        def transform_error(params):
            tx, ty, yaw = params
            c = np.cos(yaw)
            s = np.sin(yaw)
            R = np.array([[c, -s], [s, c]], dtype=float)
            transformed_xy = (R @ sensor_xy.T).T + np.array([tx, ty], dtype=float)
            err_xy = transformed_xy - ref_xy
            return err_xy.flatten()

        x0 = np.array([tx0, ty0, yaw0], dtype=float)
        loss = "soft_l1" if robust else "linear"
        result = least_squares(transform_error, x0, method="trf", loss=loss)

        tx, ty, yaw = result.x
        translation = np.array([tx, ty, tz0], dtype=float)
        rotation = np.array([0.0, 0.0, yaw], dtype=float)

        final_error = np.linalg.norm(transform_error(result.x))
        avg_error = final_error / max(1, len(sensor_points))
        print(f"  Optimization converged: {result.success}")
        print(f"  Average XY error: {avg_error:.3f} m")

        return translation, rotation

    @staticmethod
    def invert_transform(
        translation: np.ndarray,
        rotation_rpy: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Invert transform defined as: p_base = R * p_sensor + t.

        Returns (t_inv, rpy_inv) for: p_sensor = R_inv * p_base + t_inv.
        """
        R = Rotation.from_euler('xyz', rotation_rpy).as_matrix()
        R_inv = R.T
        t_inv = -(R_inv @ np.asarray(translation, dtype=float))
        rpy_inv = Rotation.from_matrix(R_inv).as_euler('xyz')
        return t_inv, rpy_inv

    # NOTE: Prior versions of this tool assumed camera points were in the
    # camera optical frame and performed an optical->camera_link conversion.
    # That conversion is now done online in camera_interface/camera_node.py,
    # so no fixed optical correction is applied here.

    def analyze_radar(self):
        """Analyze radar data and calculate transform."""
        print("\n" + "="*60)
        print("RADAR CALIBRATION ANALYSIS")
        print("="*60)

        if len(self.radar_data) == 0:
            print("No radar data found!")
            return None

        # Require sequential mode (target_name column) for proper gating
        if not self.is_sequential():
            print("ERROR: Radar analysis requires sequential calibration data (target_name column)")
            print("Please collect data using: python3 sensor_calibration.py --mode collect --target-name <name> --append")
            return None

        if len(self.ground_truth) == 0:
            print("ERROR: Ground truth file required for radar analysis")
            return None

        if self.radar_gate_min_m <= 0.0 and self.radar_init_translation is None and self.radar_init_rotation is None:
            print(
                "\n[WARN] Radar gating uses base_link ground truth against raw radar-frame points. "
                "If your radar is offset/rotated from base_link, close targets may be gated out. "
                "Consider providing --radar-init=x,y,z[,roll,pitch,yaw] and/or --radar-gate-min-m (e.g. 2.0)."
            )

        radar_clusters, radar_cluster_names = self._centroids_from_targets_with_names(
            self.radar_points_by_target,
            gate_factor=self.radar_gate_factor,
            gate_min_m=self.radar_gate_min_m,
            ground_truth_y_sign=self.radar_ground_truth_y_sign,
            initial_translation=self.radar_init_translation,
            initial_rotation=self.radar_init_rotation,
        )
        print(f"\nIdentified {len(radar_clusters)} radar targets (after distance-based gating)")

        for i, centroid in enumerate(radar_clusters):
            label = f"Target {i+1}" if not radar_cluster_names else radar_cluster_names[i]
            print(f"  {label}: x={centroid[0]:.2f}m, y={centroid[1]:.2f}m, "
                  f"z={centroid[2]:.2f}m")

        if len(radar_clusters) == 0:
            print("No valid radar targets after gating!")
            return None

        if len(radar_clusters) != len(self.ground_truth):
            print(
                "\nWarning: radar target count does not match ground truth count. "
                f"targets={len(radar_clusters)} ground_truth={len(self.ground_truth)}"
            )
            print("Tip: collect targets in the same order as rows in ground_truth.csv")

        # Calculate optimal transform
        print("\nCalculating radar transform...")
        radar_centroids = radar_clusters
        centroid_names = radar_cluster_names or []
        sensor_points: list[np.ndarray] = []
        reference_points: list[np.ndarray] = []
        
        for name, centroid in zip(centroid_names, radar_centroids):
            ref = self._lookup_ground_truth(name)
            if ref is None:
                print(f"Warning: no ground truth label matches '{name}'")
                continue
            sensor_points.append(centroid)
            reference_points.append(ref)

        if len(sensor_points) < 3:
            print("ERROR: Need at least 3 matched targets for transform calculation")
            return None

        if self.radar_fit_mode == "planar":
            print("[INFO] Radar fit mode: planar (x,y,yaw)")
            translation, rotation = self.calculate_sensor_offset_planar(
                sensor_points,
                reference_points,
                initial_translation=self.radar_init_translation,
                initial_rotation=self.radar_init_rotation,
                robust=True,
            )
        else:
            print("[INFO] Radar fit mode: 6dof (x,y,z,roll,pitch,yaw)")
            translation, rotation = self.calculate_sensor_offset(
                sensor_points,
                reference_points,
                initial_translation=self.radar_init_translation,
                initial_rotation=self.radar_init_rotation,
            )

        # The optimizer solves: p_base = R * p_radar + t
        print("\nOptimal Radar Transform (radar_link -> base_link):")
        print(
            f"  Translation: x={translation[0]:.3f}, y={translation[1]:.3f}, "
            f"z={translation[2]:.3f} (meters)"
        )
        print(
            f"  Rotation: roll={rotation[0]:.4f}, pitch={rotation[1]:.4f}, "
            f"yaw={rotation[2]:.4f} (radians)"
        )
        print(
            f"  Rotation: roll={np.rad2deg(rotation[0]):.2f}°, "
            f"pitch={np.rad2deg(rotation[1]):.2f}°, "
            f"yaw={np.rad2deg(rotation[2]):.2f}°"
        )

        # For static_transform_publisher we typically publish base_link -> radar_link.
        t_br, r_br = CalibrationAnalyzer.invert_transform(translation, rotation)
        print(f"\nFor TF publisher (base_link -> radar_link) use INVERTED values:")
        print(
            f"  Translation: x={t_br[0]:.3f}, y={t_br[1]:.3f}, "
            f"z={t_br[2]:.3f} (meters)"
        )
        print(
            f"  Rotation: roll={r_br[0]:.4f}, pitch={r_br[1]:.4f}, "
            f"yaw={r_br[2]:.4f} (radians)"
        )

        return {
            'translation': translation,
            'rotation': rotation,
            'clusters': radar_clusters
        }

    def analyze_camera(self):
        """Analyze camera data and calculate transform."""
        print("\n" + "="*60)
        print("CAMERA CALIBRATION ANALYSIS")
        print("="*60)

        if len(self.camera_data) == 0:
            print("No camera data found!")
            return None

        # Require sequential mode (target_name column) for proper gating
        if not self.is_sequential():
            print("ERROR: Camera analysis requires sequential calibration data (target_name column)")
            print("Please collect data using: python3 sensor_calibration.py --mode collect --target-name <name> --append")
            return None

        if len(self.ground_truth) == 0:
            print("ERROR: Ground truth file required for camera analysis")
            return None

        if self.camera_gate_min_m <= 0.0 and self.camera_init_translation is None and self.camera_init_rotation is None:
            print(
                "\n[WARN] Camera gating uses base_link ground truth against camera-frame points. "
                "If your camera is significantly offset from base_link, close targets may be gated out. "
                "Consider providing --camera-init=x,y,z[,roll,pitch,yaw] and/or --camera-gate-min-m (e.g. 2.0)."
            )

        camera_clusters, camera_cluster_names = self._centroids_from_targets_with_names(
            self.camera_points_by_target,
            gate_factor=self.camera_gate_factor,
            gate_min_m=self.camera_gate_min_m,
            ground_truth_y_sign=1.0,
            initial_translation=self.camera_init_translation,
            initial_rotation=self.camera_init_rotation,
        )
        print(f"\nIdentified {len(camera_clusters)} camera targets (after distance-based gating)")

        for i, centroid in enumerate(camera_clusters):
            label = f"Target {i+1}" if not camera_cluster_names else camera_cluster_names[i]
            print(f"  {label}: x={centroid[0]:.2f}m, y={centroid[1]:.2f}m, "
                  f"z={centroid[2]:.2f}m")

        if len(camera_clusters) == 0:
            print("No valid camera targets after gating!")
            return None

        if len(camera_clusters) != len(self.ground_truth):
            print(
                "\nWarning: camera target count does not match ground truth count. "
                f"targets={len(camera_clusters)} ground_truth={len(self.ground_truth)}"
            )
            print("Tip: collect targets in the same order as rows in ground_truth.csv")

        # Calculate optimal transform
        print("\nCalculating camera transform...")

        camera_centroids = camera_clusters
        centroid_names = camera_cluster_names or []
        sensor_points: list[np.ndarray] = []
        reference_points: list[np.ndarray] = []
        
        for name, centroid in zip(centroid_names, camera_centroids):
            ref = self._lookup_ground_truth(name)
            if ref is None:
                print(f"Warning: no ground truth label matches '{name}'")
                continue
            sensor_points.append(centroid)
            reference_points.append(ref)

        if len(sensor_points) < 3:
            print("ERROR: Need at least 3 matched targets for transform calculation")
            return None

        translation, rotation = self.calculate_sensor_offset(
            sensor_points,
            reference_points,
            initial_translation=self.camera_init_translation,
            initial_rotation=self.camera_init_rotation
        )

        # The optimizer solves: p_base = R * p_cam + t, where p_cam are camera_link points.
        print("\nOptimal Camera Transform (camera_link -> base_link):")
        print(
            f"  Translation: x={translation[0]:.3f}, y={translation[1]:.3f}, "
            f"z={translation[2]:.3f} (meters)"
        )
        print(
            f"  Rotation: roll={rotation[0]:.4f}, pitch={rotation[1]:.4f}, "
            f"yaw={rotation[2]:.4f} (radians)"
        )
        print(
            f"  Rotation: roll={np.rad2deg(rotation[0]):.2f}°, "
            f"pitch={np.rad2deg(rotation[1]):.2f}°, "
            f"yaw={np.rad2deg(rotation[2]):.2f}°"
        )

        return {
            'translation': translation,
            'rotation': rotation,
            'rotation_rep103': rotation,
            'clusters': camera_clusters
        }

    def generate_launch_arguments(self, radar_result, camera_result):
        """Generate launch file arguments for the calculated transforms."""
        print("\n" + "="*60)
        print("LAUNCH FILE ARGUMENTS")
        print("="*60)

        print("\nAdd these arguments to your launch command:")
        print("```bash")

        def _invert_transform(translation: np.ndarray, rotation_rpy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
            R = Rotation.from_euler('xyz', rotation_rpy).as_matrix()
            R_inv = R.T
            t_inv = -(R_inv @ np.asarray(translation, dtype=float))
            rpy_inv = Rotation.from_matrix(R_inv).as_euler('xyz')
            return t_inv, rpy_inv

        if radar_result and 'translation' in radar_result:
            # radar_result is radar_link -> base_link. Launch needs base_link -> radar_link.
            t_rb = np.asarray(radar_result['translation'], dtype=float)
            r_rb = np.asarray(radar_result['rotation'], dtype=float)
            t, r = _invert_transform(t_rb, r_rb)
            print(f"radar_tf_x:={t[0]:.3f} \\")
            print(f"radar_tf_y:={t[1]:.3f} \\")
            print(f"radar_tf_z:={t[2]:.3f} \\")
            print(f"radar_tf_roll:={r[0]:.4f} \\")
            print(f"radar_tf_pitch:={r[1]:.4f} \\")
            print(f"radar_tf_yaw:={r[2]:.4f} \\")

        if camera_result and 'translation' in camera_result:
            # camera_result is camera_link -> base_link. Launch needs base_link -> camera_link.
            t_cb = np.asarray(camera_result['translation'], dtype=float)
            r_cb = np.asarray(camera_result['rotation'], dtype=float)
            t, r = _invert_transform(t_cb, r_cb)
            print(f"camera_tf_x:={t[0]:.3f} \\")
            print(f"camera_tf_y:={t[1]:.3f} \\")
            print(f"camera_tf_z:={t[2]:.3f} \\")
            print(f"camera_tf_roll:={r[0]:.4f} \\")
            print(f"camera_tf_pitch:={r[1]:.4f} \\")
            print(f"camera_tf_yaw:={r[2]:.4f}")

        print("```")

    def run_analysis(self):
        """Run complete analysis."""
        radar_result = self.analyze_radar()
        camera_result = self.analyze_camera()

        if radar_result or camera_result:
            self.generate_launch_arguments(radar_result, camera_result)

        return radar_result, camera_result


def main():
    parser = argparse.ArgumentParser(
        description="Sensor calibration tool for radar and camera"
    )
    parser.add_argument(
        "--mode",
        choices=["collect", "analyze", "full"],
        required=True,
        help="Calibration mode: collect data, analyze data, or run full calibration"
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=30,
        help="Data collection duration in seconds (default: 30)"
    )
    parser.add_argument(
        "--start-delay",
        type=float,
        default=0.0,
        help=(
            "Seconds to wait before starting to log samples (default: 0). "
            "Useful to walk into position before logging begins."
        ),
    )
    parser.add_argument(
        "--data",
        type=str,
        help="Path to collected data CSV file (for analyze mode)"
    )
    parser.add_argument(
        "--ground-truth",
        type=str,
        default="calibration_log/ground_truth.csv",
        help="Path to ground truth CSV file with known target positions"
    )

    parser.add_argument(
        "--gps-topic",
        type=str,
        default="/navpvt",
        help="GPS topic to subscribe for origin logging (default: /navpvt)",
    )
    parser.add_argument(
        "--gps-origin-out",
        type=str,
        default="calibration_log/gps_origin.json",
        help=(
            "Output path for GPS origin JSON (default: calibration_log/gps_origin.json). "
            "Written once when logging begins."
        ),
    )
    parser.add_argument(
        "--disable-gps-origin",
        action="store_true",
        help="Disable writing GPS origin JSON",
    )
    parser.add_argument(
        "--gps-log-out",
        type=str,
        default="calibration_log/navpvt_log.csv",
        help=(
            "Output path for GPS NavPVT log CSV (default: calibration_log/navpvt_log.csv). "
            "Samples are appended while node is running."
        ),
    )
    parser.add_argument(
        "--disable-gps-log",
        action="store_true",
        help="Disable logging GPS NavPVT samples to CSV",
    )

    parser.add_argument(
        "--radar-fit-mode",
        choices=["6dof", "planar"],
        default="6dof",
        help=(
            "Radar calibration fit mode. '6dof' fits x,y,z,roll,pitch,yaw (default). "
            "'planar' fits only x,y,yaw (more stable with coplanar targets)."
        ),
    )

    parser.add_argument(
        "--radar-gate-factor",
        type=float,
        default=0.2,
        help=(
            "Distance-adaptive gating factor for radar centroids. "
            "Keeps points within factor * range_to_truth (default: 0.2)."
        ),
    )
    parser.add_argument(
        "--camera-gate-factor",
        type=float,
        default=0.3,
        help=(
            "Distance-adaptive gating factor for camera centroids. "
            "Keeps points within factor * range_to_truth (default: 0.3)."
        ),
    )
    parser.add_argument(
        "--radar-gate-min-m",
        type=float,
        default=0.0,
        help=(
            "Minimum absolute gating radius for radar points in meters (default: 0). "
            "Effective threshold is max(factor * range_to_truth, min_m)."
        ),
    )
    parser.add_argument(
        "--camera-gate-min-m",
        type=float,
        default=0.0,
        help=(
            "Minimum absolute gating radius for camera points in meters (default: 0). "
            "Effective threshold is max(factor * range_to_truth, min_m)."
        ),
    )

    parser.add_argument(
        "--exclude-camera-target",
        action="append",
        default=[],
        help=(
            "Exclude camera detections for a specific target_name during analysis. "
            "Repeatable. Example: --exclude-camera-target pos_10m"
        ),
    )
    parser.add_argument(
        "--radar-y-sign",
        type=float,
        default=1.0,
        help=(
            "Multiply radar y by this sign when loading calibration CSV (default: 1). "
            "Use -1 to flip left/right for historical logs if radar lateral axis was opposite of base_link." 
        ),
    )
    parser.add_argument(
        "--radar-ground-truth-y-sign",
        type=float,
        default=1.0,
        help=(
            "Multiply ground truth y by this sign for radar-only gating/matching (default: 1). "
            "Use -1 if your ground_truth.csv uses +y left (ROS), but your radar detections are consistent with -y for left."
        ),
    )
    parser.add_argument(
        "--target-name",
        type=str,
        help="Name/identifier for this target (for sequential calibration)"
    )
    parser.add_argument(
        "--append",
        action="store_true",
        help="Append to existing calibration data (for sequential calibration with multiple targets)"
    )
    parser.add_argument(
        "--save-tf",
        action="store_true",
        help="Save calculated TF values to a text file"
    )
    parser.add_argument(
        "--tf-out",
        type=str,
        default="calibration_log/calibration_tf.txt",
        help="Output path for saved TF values (default: calibration_log/calibration_tf.txt)"
    )   
    parser.add_argument(
        "--radar-init",
        type=str,
        action='store',
        default=None,
        help=(
            "Initial radar transform as comma-separated numbers: "
            "x,y,z or x,y,z,roll,pitch,yaw (radians). Example: --radar-init=0,0,1"
        ),
    )
    parser.add_argument(
        "--camera-init",
        type=str,
        action='store',
        default=None,
        help=(
            "Initial camera transform as comma-separated numbers: "
            "x,y,z or x,y,z,roll,pitch,yaw (radians). Example: --camera-init=-0.8,0,1.3"
        ),
    )

    args = parser.parse_args()
    
    if args.mode == "collect" or args.mode == "full":
        print("\n" + "="*60)
        print("CALIBRATION DATA COLLECTION")
        print("="*60)
        if args.append:
            print("\n⚠️  APPEND MODE - Adding to existing data")
            print(f"Target: {args.target_name or 'unknown'}")
            print("\nInstructions:")
            print("1. Position target/person at the designated location")
            print("2. Ensure target is stationary and clearly visible")
            print("3. Keep tractor stationary during collection")
            print(f"4. Data will be appended to calibration_raw_combined.csv")
        else:
            print("\nInstructions:")
            if args.target_name:
                print(f"Target: {args.target_name}")
                print("TIP: Use --append flag for subsequent targets")
            print("1. Position target/person at the designated location")
            print("2. Ensure target is stationary and clearly visible")
            print("3. Keep tractor stationary during collection")
            print("4. Record target positions in ground_truth.csv")

        if args.start_delay and args.start_delay > 0:
            print(f"\nStarting node now. Logging will begin after {args.start_delay:.1f}s...")
        else:
            print("\nStarting collection in 3 seconds...")
            time.sleep(3)

        rclpy.init()
        collector = CalibrationDataCollector(
            duration=args.duration,
            target_name=args.target_name,
            append_mode=args.append,
            start_delay_s=args.start_delay,
            gps_topic=args.gps_topic,
            gps_origin_out=args.gps_origin_out,
            gps_log_out=args.gps_log_out,
            write_gps_origin=not args.disable_gps_origin,
            log_gps_samples=not args.disable_gps_log,
        )
        try:
            rclpy.spin(collector)
        except SystemExit:
            pass
        except KeyboardInterrupt:
            print("\nCollection interrupted by user")
        finally:
            collector.destroy_node()
            rclpy.shutdown()

        # Get the filename from collector
        data_file = collector.filename

        if args.mode == "full":
            # Continue to analysis
            args.data = data_file
        else:
            print(f"\nData collection complete. To analyze, run:")
            print(f"  python3 sensor_calibration.py --mode analyze --data {data_file}")
            return
    
    if args.mode == "analyze" or args.mode == "full":
        if not args.data:
            print("Error: --data argument required for analyze mode")
            return

        if not os.path.exists(args.data):
            print(f"Error: Data file not found: {args.data}")
            return

        analyzer = CalibrationAnalyzer(
            args.data,
            args.ground_truth,
            radar_fit_mode=str(args.radar_fit_mode),
            radar_gate_factor=float(args.radar_gate_factor),
            camera_gate_factor=float(args.camera_gate_factor),
            radar_gate_min_m=float(args.radar_gate_min_m),
            camera_gate_min_m=float(args.camera_gate_min_m),
            exclude_camera_targets=list(getattr(args, "exclude_camera_target", []) or []),
            radar_y_sign=float(args.radar_y_sign),
            radar_ground_truth_y_sign=float(args.radar_ground_truth_y_sign),
        )
        
        # Parse initial transform arguments
        def _parse_init(s: str):
            """Parse comma-separated init values. Accepts 3 (translation) or 6 (translation+rotation) values."""
            if s is None:
                return None, None
            parts = [float(x.strip()) for x in s.split(",") if x.strip() != ""]
            if len(parts) == 3:
                return np.array(parts), None
            elif len(parts) == 6:
                return np.array(parts[:3]), np.array(parts[3:])
            else:
                raise ValueError(f"Expected 3 or 6 comma-separated numbers, got {len(parts)}")
        
        # Set radar initial transform if provided
        if args.radar_init:
            try:
                radar_trans, radar_rot = _parse_init(args.radar_init)
                analyzer.radar_init_translation = radar_trans
                analyzer.radar_init_rotation = radar_rot
                print(f"\n[INFO] Radar initial transform: translation={radar_trans}, rotation={radar_rot}")
            except Exception as e:
                print(f"\n[ERROR] Failed to parse --radar-init: {e}")
                return
        
        # Set camera initial transform if provided
        if args.camera_init:
            try:
                camera_trans, camera_rot = _parse_init(args.camera_init)
                analyzer.camera_init_translation = camera_trans
                analyzer.camera_init_rotation = camera_rot
                print(f"[INFO] Camera initial transform: translation={camera_trans}, rotation={camera_rot}")
            except Exception as e:
                print(f"[ERROR] Failed to parse --camera-init: {e}")
                return
        
        radar_result, camera_result = analyzer.run_analysis()
        if args.save_tf:
            analyzer.save_tf_results(radar_result, camera_result, args.tf_out)


if __name__ == "__main__":
    main()
