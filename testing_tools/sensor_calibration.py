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
                    self.camera_data.append(point)
                    if has_target_name and target_name:
                        self.camera_points_by_target.setdefault(target_name, []).append(point)

        print(f"Loaded {len(self.radar_data)} radar points")
        print(f"Loaded {len(self.camera_data)} camera points")

        if has_target_name and len(self.target_order) > 0:
            print("Sequential calibration data detected (target_name column present)")
            print(f"Detected targets in collection order: {self.target_order}")
        else:
            print("Single-session calibration data (no target_name column)")

    def is_sequential(self) -> bool:
        return len(self.target_order) > 0

    def _centroids_from_targets(self, points_by_target: dict[str, list[np.ndarray]]) -> list[np.ndarray]:
        """Compute one centroid per target using the CSV target order."""
        centroids, _ = self._centroids_from_targets_with_names(points_by_target)
        return centroids

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

    def _dominant_cluster_centroid(
        self,
        points: list[np.ndarray],
        eps: float = 0.4,
        min_samples: int = 5,
    ) -> tuple[np.ndarray, float]:
        """Return centroid of the dominant spatial cluster and inlier ratio.

        This is designed for sequential calibration where the radar may produce
        false targets while only one true target is present.
        """
        if len(points) == 0:
            raise ValueError("No points")

        data = np.asarray(points)
        try:
            from sklearn.cluster import DBSCAN  # type: ignore

            labels = DBSCAN(eps=eps, min_samples=min_samples).fit_predict(data)
            # choose largest non-noise cluster
            best_label = None
            best_count = 0
            for label in set(labels.tolist()):
                if label == -1:
                    continue
                count = int((labels == label).sum())
                if count > best_count:
                    best_label = label
                    best_count = count
            if best_label is None:
                # fallback to robust centroid
                median = np.median(data, axis=0)
                d = np.linalg.norm(data - median, axis=1)
                keep = d <= np.quantile(d, 0.8)
                kept = data[keep] if keep.any() else data
                return np.mean(kept, axis=0), float(len(kept)) / float(len(data))

            inliers = data[labels == best_label]
            return np.mean(inliers, axis=0), float(len(inliers)) / float(len(data))
        except ModuleNotFoundError:
            # No scikit-learn: robust fallback based on median + trimming.
            median = np.median(data, axis=0)
            d = np.linalg.norm(data - median, axis=1)
            keep = d <= np.quantile(d, 0.8)
            kept = data[keep] if keep.any() else data
            return np.mean(kept, axis=0), float(len(kept)) / float(len(data))

    def _centroids_from_targets_with_names(
        self,
        points_by_target: dict[str, list[np.ndarray]],
    ) -> tuple[list[np.ndarray], list[str]]:
        """Compute one centroid per target (dominant cluster) in target_order."""
        centroids: list[np.ndarray] = []
        names: list[str] = []
        for target_name in self.target_order:
            points = points_by_target.get(target_name, [])
            if len(points) == 0:
                continue
            centroid, inlier_ratio = self._dominant_cluster_centroid(points)
            if inlier_ratio < 0.5:
                print(
                    f"Warning: '{target_name}' inlier ratio is low ({inlier_ratio:.0%}); "
                    "false targets/noise may dominate. Consider recollecting this target."
                )
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

    def cluster_detections(self, data: List[np.ndarray], eps: float = 0.5) -> List[np.ndarray]:
        """
        Cluster detections to identify distinct targets.

        Args:
            data: List of 3D points
            eps: Maximum distance between points in same cluster (meters)
            
        Returns:
            List of cluster centroids
        """
        if len(data) == 0:
            return []

        try:
            from sklearn.cluster import DBSCAN
        except ModuleNotFoundError as exc:
            raise ModuleNotFoundError(
                "scikit-learn is required for non-sequential clustering analysis. "
                "Install it with: pip3 install -r calibration_requirements.txt"
            ) from exc

        data_array = np.array(data)
        clustering = DBSCAN(eps=eps, min_samples=3).fit(data_array)

        centroids = []
        for label in set(clustering.labels_):
            if label == -1:  # Skip noise
                continue
            cluster_points = data_array[clustering.labels_ == label]
            centroid = np.mean(cluster_points, axis=0)
            centroids.append(centroid)

        return centroids

    def calculate_sensor_offset(
        self, 
        sensor_points: List[np.ndarray],
        reference_points: List[np.ndarray],
        initial_rotation: np.ndarray = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Calculate sensor offset and orientation using point matching.

        This uses ICP-like approach to find the best transformation that
        aligns sensor measurements with reference positions.

        Args:
            sensor_points: Points detected by sensor (in sensor frame)
            reference_points: Known reference positions (in base_link frame)
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
        # Use provided rotation if given, otherwise zero
        if initial_rotation is not None:
            x0 = np.array([0.0, 0.0, 0.0, 
                          initial_rotation[0], 
                          initial_rotation[1], 
                          initial_rotation[2]])
        else:
            x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

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

    def analyze_radar(self):
        """Analyze radar data and calculate transform."""
        print("\n" + "="*60)
        print("RADAR CALIBRATION ANALYSIS")
        print("="*60)

        if len(self.radar_data) == 0:
            print("No radar data found!")
            return None

        radar_cluster_names: list[str] | None = None

        # Sequential mode: use per-target centroids (dominant cluster per target)
        if self.is_sequential():
            radar_clusters, radar_cluster_names = self._centroids_from_targets_with_names(
                self.radar_points_by_target
            )
            print(f"\nSequential mode: computed {len(radar_clusters)} radar centroids")
        else:
            radar_clusters = self.cluster_detections(self.radar_data, eps=0.5)
        print(f"\nIdentified {len(radar_clusters)} radar targets")

        for i, centroid in enumerate(radar_clusters):
            label = f"Target {i+1}" if not radar_cluster_names else radar_cluster_names[i]
            print(f"  {label}: x={centroid[0]:.2f}m, y={centroid[1]:.2f}m, "
                  f"z={centroid[2]:.2f}m")

        if len(self.ground_truth) > 0:
            if self.is_sequential() and len(radar_clusters) != len(self.ground_truth):
                print(
                    "\nWarning: sequential target count does not match ground truth count. "
                    f"targets={len(radar_clusters)} ground_truth={len(self.ground_truth)}"
                )
                print("Tip: collect targets in the same order as rows in ground_truth.csv")

            # Calculate optimal transform
            print("\nCalculating radar transform...")
            if self.is_sequential() and len(self.ground_truth_by_name) > 0:
                radar_centroids = radar_clusters
                centroid_names = radar_cluster_names or []
                sensor_points: list[np.ndarray] = []
                reference_points: list[np.ndarray] = []
                for name, centroid in zip(centroid_names, radar_centroids):
                    ref = self._lookup_ground_truth(name)
                    if ref is None:
                        print(
                            f"Warning: no ground truth label matches '{name}'. "
                            "Falling back to order-based matching for this point."
                        )
                        continue
                    sensor_points.append(centroid)
                    reference_points.append(ref)

                if len(sensor_points) >= 3:
                    translation, rotation = self.calculate_sensor_offset(
                        sensor_points, reference_points
                    )
                else:
                    print(
                        "Not enough name-matched targets (need >=3). Falling back to order-based matching."
                    )
                    translation, rotation = self.calculate_sensor_offset(
                        radar_clusters, self.ground_truth
                    )
            else:
                translation, rotation = self.calculate_sensor_offset(
                    radar_clusters, self.ground_truth
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
        else:
            print("\nNo ground truth provided. Showing detected clusters only.")
            return {'clusters': radar_clusters}

    def analyze_camera(self):
        """Analyze camera data and calculate transform."""
        print("\n" + "="*60)
        print("CAMERA CALIBRATION ANALYSIS")
        print("="*60)

        if len(self.camera_data) == 0:
            print("No camera data found!")
            return None

        camera_cluster_names: list[str] | None = None

        # Sequential mode: use per-target centroids (dominant cluster per target)
        if self.is_sequential():
            camera_clusters, camera_cluster_names = self._centroids_from_targets_with_names(
                self.camera_points_by_target
            )
            print(f"\nSequential mode: computed {len(camera_clusters)} camera centroids")
        else:
            camera_clusters = self.cluster_detections(self.camera_data, eps=0.5)
        print(f"\nIdentified {len(camera_clusters)} camera targets")

        for i, centroid in enumerate(camera_clusters):
            label = f"Target {i+1}" if not camera_cluster_names else camera_cluster_names[i]
            print(f"  {label}: x={centroid[0]:.2f}m, y={centroid[1]:.2f}m, "
                  f"z={centroid[2]:.2f}m")

        if len(self.ground_truth) > 0:
            if self.is_sequential() and len(camera_clusters) != len(self.ground_truth):
                print(
                    "\nWarning: sequential target count does not match ground truth count. "
                    f"targets={len(camera_clusters)} ground_truth={len(self.ground_truth)}"
                )
                print("Tip: collect targets in the same order as rows in ground_truth.csv")

            # Calculate optimal transform
            print("\nCalculating camera transform...")

            # OAK-D camera has known frame rotations (not calibrated, they're geometric)
            # Roll=-90°, Yaw=-90° are standard for OAK-D mounting
            initial_camera_rotation = np.array([-np.pi/2, 0.0, -np.pi/2])  # roll, pitch, yaw
            print("  Using known OAK-D frame rotations as initial guess")
            
            if self.is_sequential() and len(self.ground_truth_by_name) > 0:
                camera_centroids = camera_clusters
                centroid_names = camera_cluster_names or []
                sensor_points: list[np.ndarray] = []
                reference_points: list[np.ndarray] = []
                for name, centroid in zip(centroid_names, camera_centroids):
                    ref = self._lookup_ground_truth(name)
                    if ref is None:
                        print(
                            f"Warning: no ground truth label matches '{name}'. "
                            "Falling back to order-based matching for this point."
                        )
                        continue
                    sensor_points.append(centroid)
                    reference_points.append(ref)

                if len(sensor_points) >= 3:
                    translation, rotation = self.calculate_sensor_offset(
                        sensor_points,
                        reference_points,
                        initial_rotation=initial_camera_rotation,
                    )
                else:
                    print(
                        "Not enough name-matched targets (need >=3). Falling back to order-based matching."
                    )
                    translation, rotation = self.calculate_sensor_offset(
                        camera_clusters,
                        self.ground_truth,
                        initial_rotation=initial_camera_rotation,
                    )
            else:
                translation, rotation = self.calculate_sensor_offset(
                    camera_clusters,
                    self.ground_truth,
                    initial_rotation=initial_camera_rotation,
                )

            print(f"\nOptimal Camera Transform (base_link -> camera_link):")
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
                'clusters': camera_clusters
            }
        else:
            print("\nNo ground truth provided. Showing detected clusters only.")
            return {'clusters': camera_clusters}

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


def create_ground_truth_template():
    """Create a ground truth CSV file."""
    os.makedirs("calibration_log", exist_ok=True)
    filename = "calibration_log/ground_truth_template.csv"

    # Check if file already exists
    if os.path.exists(filename):
        print(f"Warning: {filename} already exists!")
        response = input("Overwrite? [y/N]: ")
        if response.lower() != 'y':
            print("Keeping existing file.")
            return

    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["target_name", "x", "y", "z", "description"])
        writer.writerow(["pos_5m", 5.0, 0.0, 0.0, "5m straight ahead"])
        writer.writerow(["pos_10m", 10.0, 0.0, 0.0, "10m straight ahead"])
        writer.writerow(["pos_15m", 15.0, 0.0, 0.0, "15m straight ahead"])
        writer.writerow(["pos_5m_left", 5.0, 2.0, 0.0, "5m, 2m to the left"])

    print(f"Created: {filename}")
    print("Edit this file with your actual target positions:")
    print(f"  nano {filename}")
    print("Then copy it to calibration_log/ground_truth.csv:")
    print("  cp calibration_log/ground_truth_template.csv calibration_log/ground_truth.csv")


def main():
    parser = argparse.ArgumentParser(
        description="Sensor calibration tool for radar and camera"
    )
    parser.add_argument(
        "--mode",
        choices=["collect", "analyze", "full", "template"],
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

    args = parser.parse_args()
    
    if args.mode == "template":
        create_ground_truth_template()
        return

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
        analyzer.run_analysis()


if __name__ == "__main__":
    main()
