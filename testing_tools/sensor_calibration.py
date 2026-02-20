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


class CalibrationDataCollector(Node):
    """Collects sensor data for calibration."""

    def __init__(
        self,
        duration: int = 30,
        target_name: str = None,
        append_mode: bool = False,
        start_delay_s: float = 0.0,
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
        if append_mode:
            self.get_logger().info("APPEND MODE: Data will be added to existing file")
        self.get_logger().info(
            "Keep target stationary at the designated position"
        )

    def _logging_active(self) -> bool:
        return self.start_time is not None

    def _maybe_start_logging(self) -> None:
        if self._logging_active():
            return

        if time.time() < self._warmup_end_wall_time:
            return

        self.start_time = time.time()
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
            self.get_logger().info(f"Data saved to: {self.filename}")
            raise SystemExit


class CalibrationAnalyzer:
    """Analyzes calibration data and calculates optimal transforms."""

    def __init__(self, data_file: str, ground_truth_file: str = None):
        self.data_file = data_file
        self.ground_truth_file = ground_truth_file

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

        # Load sensor data
        self.radar_data = []
        self.camera_data = []
        self.load_sensor_data()

        # Load ground truth positions if provided
        self.ground_truth = []
        if ground_truth_file and os.path.exists(ground_truth_file):
            self.load_ground_truth()

    def load_sensor_data(self):
        """Load sensor data from CSV file."""
        print(f"Loading sensor data from {self.data_file}...")

        has_target_name = False
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
                target_name = (row.get('target_name') or '').strip() if has_target_name else ''
                if has_target_name and target_name and target_name not in self.target_order:
                    self.target_order.append(target_name)

                if sensor_type == 'radar':
                    self.radar_data.append(point)
                    if has_target_name and target_name:
                        self.radar_points_by_target.setdefault(target_name, []).append(point)
                elif sensor_type == 'camera':
                    # Camera data arrives in optical frame (x right, y down, z forward)
                    # Convert to base_link-like frame for proper calibration
                    # base_x = z_cam, base_y = -x_cam, base_z = -y_cam
                    x_c, y_c, z_c = point
                    point_to_store = np.array([z_c, -x_c, -y_c], dtype=float)

                    self.camera_data.append(point_to_store)
                    if has_target_name and target_name:
                        self.camera_points_by_target.setdefault(target_name, []).append(point_to_store)

        print(f"Loaded {len(self.radar_data)} radar points")
        print(f"Loaded {len(self.camera_data)} camera points")

        if has_target_name and len(self.target_order) > 0:
            print("Sequential calibration data detected (target_name column present)")
            print(f"Detected targets in collection order: {self.target_order}")
        else:
            print("Single-session calibration data (no target_name column)")

    def is_sequential(self) -> bool:
        return len(self.target_order) > 0

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

    def _gate_detections_by_distance(self, target_name: str, points: list[np.ndarray]) -> list[np.ndarray]:
        """Filter detections to keep only those within 20% of ground truth distance.
        
        The gating threshold is: distance from detection to ground truth < 0.2 * distance from origin to ground truth.
        This provides distance-adaptive gating (tighter at close range, looser at far range).
        """
        if len(points) == 0:
            return []
        
        # Get expected ground truth position for this target
        expected_pos = self._lookup_ground_truth(target_name)
        if expected_pos is None:
            # No ground truth for this target, return all points
            return points
        
        # Calculate distance from origin to ground truth
        distance_to_truth = np.linalg.norm(expected_pos)
        
        # Gating threshold: 20% of distance to ground truth
        threshold = 0.2 * distance_to_truth
        
        # Filter points
        gated_points = []
        for point in points:
            distance_from_expected = np.linalg.norm(point - expected_pos)
            if distance_from_expected <= threshold:
                gated_points.append(point)
        
        return gated_points

    def _centroids_from_targets_with_names(
        self,
        points_by_target: dict[str, list[np.ndarray]],
    ) -> tuple[list[np.ndarray], list[str]]:
        """Compute one centroid per target using distance-based gating."""
        centroids: list[np.ndarray] = []
        names: list[str] = []
        for target_name in self.target_order:
            points = points_by_target.get(target_name, [])
            if len(points) == 0:
                continue
            
            # Apply distance-based gating
            gated_points = self._gate_detections_by_distance(target_name, points)
            
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
    def _compose_camera_with_optical_frame(rotation_calibrated: np.ndarray) -> np.ndarray:
        """Compose calibrated camera rotation with fixed optical frame correction.
        
        The camera data is converted during loading using the fixed optical-to-base transform:
        - roll = -π/2, pitch = 0, yaw = -π/2
        
        This function composes the fixed correction with the calibrated rotation using
        proper rotation matrix multiplication (not linear addition).
        
        Args:
            rotation_calibrated: [roll, pitch, yaw] rotation computed by calibration
            
        Returns:
            rotation_total: [roll, pitch, yaw] with optical frame correction applied
        """
        OPTICAL_FRAME_CORRECTION = np.array([-np.pi/2, 0.0, -np.pi/2])
        
        # Compose rotations using matrix multiplication (mathematically correct)
        # R_total = R_fixed @ R_calibrated
        R_fixed = Rotation.from_euler('xyz', OPTICAL_FRAME_CORRECTION).as_matrix()
        R_calibrated = Rotation.from_euler('xyz', rotation_calibrated).as_matrix()
        R_total = R_fixed @ R_calibrated
        
        # Convert back to Euler angles
        rotation_total = Rotation.from_matrix(R_total).as_euler('xyz')
        
        return rotation_total

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

        radar_clusters, radar_cluster_names = self._centroids_from_targets_with_names(
            self.radar_points_by_target
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

        translation, rotation = self.calculate_sensor_offset(
            sensor_points, 
            reference_points,
            initial_translation=self.radar_init_translation,
            initial_rotation=self.radar_init_rotation
        )
        
        print(f"\nOptimal Radar Transform (base_link -> radar_link):")
        print(f"  Translation: x={translation[0]:.3f}, y={translation[1]:.3f}, "
              f"z={translation[2]:.3f} (meters)")
        print(f"  Rotation: roll={rotation[0]:.4f}, pitch={rotation[1]:.4f}, "
              f"yaw={rotation[2]:.4f} (radians)")
        print(f"  Rotation: roll={np.rad2deg(rotation[0]):.2f}°, "
              f"pitch={np.rad2deg(rotation[1]):.2f}°, "
              f"yaw={np.rad2deg(rotation[2]):.2f}°")

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

        camera_clusters, camera_cluster_names = self._centroids_from_targets_with_names(
            self.camera_points_by_target
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

        print(f"\nOptimal Camera Transform (calibrated in base-aligned frame):")
        print(f"  Translation: x={translation[0]:.3f}, y={translation[1]:.3f}, "
              f"z={translation[2]:.3f} (meters)")
        print(f"  Rotation: roll={rotation[0]:.4f}, pitch={rotation[1]:.4f}, "
              f"yaw={rotation[2]:.4f} (radians)")
        
        # Compose with fixed optical frame correction for output
        rotation_with_optical = self._compose_camera_with_optical_frame(rotation)
        print(f"\nOptimal Camera Transform (with optical frame correction - FOR TF):")
        print(f"  Translation: x={translation[0]:.3f}, y={translation[1]:.3f}, "
              f"z={translation[2]:.3f} (meters)")
        print(f"  Rotation: roll={rotation_with_optical[0]:.4f}, pitch={rotation_with_optical[1]:.4f}, "
              f"yaw={rotation_with_optical[2]:.4f} (radians)")
        print(f"  Rotation: roll={np.rad2deg(rotation_with_optical[0]):.2f}°, "
              f"pitch={np.rad2deg(rotation_with_optical[1]):.2f}°, "
              f"yaw={np.rad2deg(rotation_with_optical[2]):.2f}°")
        
        return {
            'translation': translation,
            'rotation': rotation_with_optical,
            'clusters': camera_clusters
        }

    def generate_launch_arguments(self, radar_result, camera_result):
        """Generate launch file arguments for the calculated transforms."""
        print("\n" + "="*60)
        print("LAUNCH FILE ARGUMENTS")
        print("="*60)

        print("\nAdd these arguments to your launch command:")
        print("```bash")

        if radar_result and 'translation' in radar_result:
            t = radar_result['translation']
            r = radar_result['rotation']
            print(f"radar_tf_x:={t[0]:.3f} \\")
            print(f"radar_tf_y:={t[1]:.3f} \\")
            print(f"radar_tf_z:={t[2]:.3f} \\")
            print(f"radar_tf_roll:={r[0]:.4f} \\")
            print(f"radar_tf_pitch:={r[1]:.4f} \\")
            print(f"radar_tf_yaw:={r[2]:.4f} \\")

        if camera_result and 'translation' in camera_result:
            t = camera_result['translation']
            r = camera_result['rotation']
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

        analyzer = CalibrationAnalyzer(args.data, args.ground_truth)
        
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
        
        analyzer.run_analysis()


if __name__ == "__main__":
    main()
