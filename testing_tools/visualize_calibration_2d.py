#!/usr/bin/env python3
"""
Visualize sensor calibration results in 2D (top-down view).

Plots sensor detections on x-y plane before and after applying the calibrated TF transforms.
"""

import argparse
import csv
import numpy as np
import matplotlib.pyplot as plt
import os
from scipy.spatial.transform import Rotation

def load_tf_file(tf_file: str):
    radar_tf = None
    camera_tf = None
    camera_cal_rpy = None

    if not tf_file or not os.path.exists(tf_file):
        return radar_tf, camera_tf, camera_cal_rpy

    def _invert_tf6(vals6):
        """Invert [tx, ty, tz, roll, pitch, yaw] for xyz Euler convention.

        TF file values are treated as base_link -> sensor_link.
        Visualization needs sensor_link -> base_link to transform sensor points into base.
        """
        tx, ty, tz, roll, pitch, yaw = [float(x) for x in vals6]
        R = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
        R_inv = R.T
        t = np.array([tx, ty, tz], dtype=float)
        t_inv = -(R_inv @ t)
        rpy_inv = Rotation.from_matrix(R_inv).as_euler('xyz')
        return [
            float(t_inv[0]),
            float(t_inv[1]),
            float(t_inv[2]),
            float(rpy_inv[0]),
            float(rpy_inv[1]),
            float(rpy_inv[2]),
        ]

    with open(tf_file, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue

            # format: key: numbers...
            if ":" not in line:
                continue
            key, rest = line.split(":", 1)
            vals = [float(x) for x in rest.strip().split()]

            if key == "radar_tf" and len(vals) == 6:
                radar_tf = _invert_tf6(vals)
            elif key == "camera_tf" and len(vals) == 6:
                camera_tf = _invert_tf6(vals)
            elif key == "camera_calibrated_rpy" and len(vals) == 3:
                camera_cal_rpy = vals

    return radar_tf, camera_tf, camera_cal_rpy

def load_calibration_data(data_file, ground_truth=None):
    """Load sensor detections from CSV.

    Convention used in this project:
        - Camera detections are in camera_link: x forward, y left, z up.
            (Optical->camera_link axis swap happens online in camera_node.)
        - TF file values (camera_tf) are base_link -> camera_link.
    """
    radar_points_by_target: dict[str, list[np.ndarray]] = {}
    camera_points_by_target: dict[str, list[np.ndarray]] = {}

    camera_frame_ids: set[str] = set()
    
    with open(data_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            point = np.array([
                float(row['x']),
                float(row['y']),
                float(row['z'])
            ])
            sensor_type = row.get('sensor_type', '')
            target_name = (row.get('target_name') or '').strip()
            frame_id = (row.get('frame_id') or '').strip().lower()
            
            if target_name:
                if sensor_type == 'radar':
                    radar_points_by_target.setdefault(target_name, []).append(point)
                elif sensor_type == 'camera':
                    camera_frame_ids.add(frame_id or 'unknown')
                    camera_points_by_target.setdefault(target_name, []).append(
                        np.array(point, dtype=float)
                    )

    if camera_frame_ids:
        pretty = ", ".join(sorted(camera_frame_ids))
        print(f"[INFO] Camera frame_id(s) in CSV: {pretty}")
        print("[INFO] Camera points loaded as camera_link coordinates")
    
    return radar_points_by_target, camera_points_by_target


def load_ground_truth(gt_file):
    """Load ground truth positions."""
    ground_truth = {}
    with open(gt_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            name = row.get('target_name') or row.get('description')
            if name:
                name = name.strip()
                pos = np.array([float(row['x']), float(row['y']), float(row['z'])])
                ground_truth[name] = pos
    return ground_truth


def gate_detections(points, target_name, ground_truth):
    """Apply distance-based gating."""
    if target_name not in ground_truth:
        return points
    
    expected_pos = ground_truth[target_name]
    distance_to_truth = np.linalg.norm(expected_pos)
    threshold = 0.2 * distance_to_truth
    
    gated_points = []
    for point in points:
        if np.linalg.norm(point - expected_pos) <= threshold:
            gated_points.append(point)
    
    return gated_points


def gate_detections_by_range(points, target_name, ground_truth):
    """Gate points by range-to-target only.

    Useful when points are not in base_link coordinates.
    """
    if target_name not in ground_truth:
        return points

    expected_pos = ground_truth[target_name]
    distance_to_truth = np.linalg.norm(expected_pos)
    threshold = 0.2 * distance_to_truth

    gated_points = []
    for point in points:
        if abs(np.linalg.norm(point) - distance_to_truth) <= threshold:
            gated_points.append(point)
    return gated_points


def apply_transform(points, translation, rotation_euler):
    """Apply rigid transform to points."""
    if points is None:
        return np.array([])

    points_array = np.asarray(points, dtype=float)
    if points_array.size == 0:
        return np.array([])
    if points_array.ndim == 1:
        points_array = points_array.reshape(1, -1)

    r_tf = Rotation.from_euler('xyz', rotation_euler)

    R = r_tf.as_matrix()    
    t = np.array(translation)

    transformed = (R @ points_array.T).T + t
    return transformed


def expected_point_in_camera_link(
    expected_pos_base: np.ndarray,
    camera_tf_trans: list[float],
    camera_tf_rot_xyz: list[float],
) -> np.ndarray:
    """Compute expected target position in camera_link.

    We assume the TF provided to this script is camera_link -> base_link:
        p_base = R * p_cam + t
    so:
        p_cam = R^T * (p_base - t)
    """
    R = Rotation.from_euler('xyz', camera_tf_rot_xyz).as_matrix()
    t = np.array(camera_tf_trans, dtype=float)
    expected_pos_base = np.asarray(expected_pos_base, dtype=float).reshape(3)
    return (R.T @ (expected_pos_base - t)).reshape(3)


def camera_points_as_camera_link(points: list[np.ndarray]) -> np.ndarray:
    """Return points in camera_link axes.

    Logs are expected to be in camera_link, but older logs may contain optical-frame
    coordinates (x right, y down, z forward). We auto-detect and convert those:
      x_link = z_opt, y_link = -x_opt, z_link = -y_opt
    """
    if not points:
        return np.array([])

    pts = np.asarray(points, dtype=float)
    med_abs = np.median(np.abs(pts), axis=0)
    looks_optical = bool(med_abs[2] > 1.5 * med_abs[0])
    if not looks_optical:
        return pts

    x = pts[:, 0]
    y = pts[:, 1]
    z = pts[:, 2]
    return np.stack([z, -x, -y], axis=1)


def main():
    parser = argparse.ArgumentParser(description='Visualize sensor calibration results in 2D')
    parser.add_argument('--data', required=True, help='Calibration data CSV file')
    parser.add_argument('--ground-truth', required=True, help='Ground truth CSV file')
    parser.add_argument('--radar-tf', nargs=6, type=float, 
                       metavar=('X', 'Y', 'Z', 'ROLL', 'PITCH', 'YAW'),
                       help='Radar transform (tx ty tz roll pitch yaw)')
    parser.add_argument('--camera-tf', nargs=6, type=float,
                       metavar=('X', 'Y', 'Z', 'ROLL', 'PITCH', 'YAW'),
                       help='Camera transform (tx ty tz roll pitch yaw) (camera_link -> base_link)')
    parser.add_argument('--camera-calibrated-roll', type=float, default=-0.0770,
                       help='Camera calibrated roll (without optical correction)')
    parser.add_argument('--camera-calibrated-pitch', type=float, default=0.0938,
                       help='Camera calibrated pitch (without optical correction)')
    parser.add_argument('--camera-calibrated-yaw', type=float, default=0.0491,
                       help='Camera calibrated yaw (without optical correction)')
    parser.add_argument('--sensor', choices=['radar', 'camera', 'both'], 
                       default='both', help='Which sensor to visualize')
    parser.add_argument('--tf-file', default='calibration_log/calibration_tf.txt',
                        help='TF results text file (auto-loaded if --radar-tf/--camera-tf not given)')
    parser.add_argument(
        '--no-show',
        action='store_true',
        help='Do not open an interactive window; only save the PNG.',
    )
    
    args = parser.parse_args()
    
    
    if args.radar_tf is None or args.camera_tf is None:
        r_tf, c_tf, c_cal = load_tf_file(args.tf_file)

        if args.radar_tf is None and r_tf is not None:
            args.radar_tf = r_tf
            print(f"[INFO] Loaded radar_tf from {args.tf_file}")

        if args.camera_tf is None and c_tf is not None:
            args.camera_tf = c_tf
            # use the loaded rotation too
            args.camera_calibrated_roll, args.camera_calibrated_pitch, args.camera_calibrated_yaw = args.camera_tf[3:]
            print(f"[INFO] Loaded camera_tf from {args.tf_file}")

        # update calibrated-only rpy for alignment if present
        if c_cal is not None:
            args.camera_calibrated_roll, args.camera_calibrated_pitch, args.camera_calibrated_yaw = c_cal
            print(f"[INFO] Loaded camera_calibrated_rpy from {args.tf_file}")

    ground_truth = load_ground_truth(args.ground_truth)
    print("Loading calibration data...")
    radar_by_tgt, camera_by_tgt = load_calibration_data(args.data, ground_truth=ground_truth)
    
    print(f"Ground truth targets: {list(ground_truth.keys())}")
    
    # Calculate consistent axis limits using only GATED points
    all_gated_points = []
    for target_name, points in radar_by_tgt.items():
        gated = gate_detections(points, target_name, ground_truth)
        all_gated_points.extend(gated)
    for target_name, points in camera_by_tgt.items():
        if args.camera_tf:
            camera_tf_trans = args.camera_tf[:3]
            camera_tf_rot = args.camera_tf[3:]
            points_link = camera_points_as_camera_link(points)
            transformed = apply_transform(points_link, camera_tf_trans, camera_tf_rot)
            gated = gate_detections(transformed, target_name, ground_truth)
            all_gated_points.extend(gated)
        else:
            gated_cam = gate_detections_by_range(points, target_name, ground_truth)
            pts_link = camera_points_as_camera_link(gated_cam)
            if pts_link.size:
                all_gated_points.extend(pts_link)
    for pos in ground_truth.values():
        all_gated_points.append(pos)
    
    all_gated_points = np.array(all_gated_points)
    x_min, y_min = np.min(all_gated_points[:, :2], axis=0)
    x_max, y_max = np.max(all_gated_points[:, :2], axis=0)
    
    # Use independent ranges for each axis to show actual differences
    # Zoom in on y-axis to show small variations
    x_range = x_max - x_min
    y_range = y_max - y_min
    x_padding = 0.2
    y_padding = 0.2  # Much smaller padding for y to zoom in
    x_lim = [x_min - x_padding, x_max + x_padding]
    y_lim = [y_min - y_padding, y_max + y_padding]
    
    # Define colors for each target
    colors = {'pos_5m': '#1f77b4', 'pos_7m': '#ff7f0e', 'pos_10m': '#2ca02c'}
    
    # Create wider figure to show details
    if args.sensor == 'radar':
        fig, axes = plt.subplots(1, 2, figsize=(20, 7))
    elif args.sensor == 'camera':
        fig, axes = plt.subplots(1, 2, figsize=(20, 7))
    else:  # both
        fig, axes = plt.subplots(2, 2, figsize=(20, 14))
    
    axes = np.atleast_1d(axes).flatten()
    plot_idx = 0
    
    # Plot radar
    if args.sensor in ['radar', 'both']:
        # Before transform
        ax1 = axes[plot_idx]
        plot_idx += 1
        
        for target_name, points in radar_by_tgt.items():
            gated = gate_detections(points, target_name, ground_truth)
            if gated:
                gated = np.array(gated)
                ax1.scatter(gated[:, 0], gated[:, 1], 
                           label=f'{target_name} (n={len(gated)})', 
                           alpha=0.5, s=20, color=colors.get(target_name, 'blue'))
        
        # Plot ground truth
        for name, pos in ground_truth.items():
            ax1.scatter(
                pos[0], pos[1],
                marker='X',
                s=100,                    # smaller
                facecolors='none',       # hollow X
                edgecolors='black',
                linewidths=2.0,
                alpha=0.8,
                label=f'{name} (truth)',
                zorder=1                 # behind measurements
            )
        
        ax1.set_xlabel('X (m)', fontsize=12, fontweight='bold')
        ax1.set_ylabel('Y (m)', fontsize=12, fontweight='bold')
        ax1.set_title('Radar Detections (Before Transform)', fontsize=13, fontweight='bold')
        ax1.legend(loc='upper left', bbox_to_anchor=(1.01, 1), fontsize=10, frameon=True)
        ax1.grid(True, alpha=0.3)
        ax1.set_xlim(x_lim)
        ax1.set_ylim(y_lim)
        
        # After transform
        if args.radar_tf:
            ax2 = axes[plot_idx]
            plot_idx += 1
            
            radar_tf_trans = args.radar_tf[:3]
            radar_tf_rot = args.radar_tf[3:]
            
            for target_name, points in radar_by_tgt.items():
                gated = gate_detections(points, target_name, ground_truth)
                if gated:
                    transformed = apply_transform(gated, radar_tf_trans, radar_tf_rot)
                    ax2.scatter(transformed[:, 0], transformed[:, 1],
                               label=f'{target_name} (n={len(gated)})', 
                               alpha=0.5, s=20, color=colors.get(target_name, 'blue'))
            
            # Plot ground truth
            for name, pos in ground_truth.items():
                ax2.scatter(
                    pos[0], pos[1],
                    marker='X',
                    s=100,                    # smaller
                    facecolors='none',       # hollow X
                    edgecolors='black',
                    linewidths=2.0,
                    alpha=0.8,
                    label=f'{name} (truth)',
                    zorder=1                 # behind measurements
                )
            
            ax2.set_xlabel('X (m)', fontsize=12, fontweight='bold')
            ax2.set_ylabel('Y (m)', fontsize=12, fontweight='bold')
            ax2.set_title('Radar Detections (After Transform)', fontsize=13, fontweight='bold')
            ax2.legend(loc='upper left', bbox_to_anchor=(1.01, 1), fontsize=10, frameon=True)
            ax2.grid(True, alpha=0.3)
            ax2.set_xlim(x_lim)
            ax2.set_ylim(y_lim)
    
    # Plot camera
    if args.sensor in ['camera', 'both']:
        # Before transform
        ax3 = axes[plot_idx]
        plot_idx += 1
        
        for target_name, points in camera_by_tgt.items():
            pts_link_all = camera_points_as_camera_link(points)

            # Gating for visualization only.
            # If we have camera_tf, compute the expected target position in camera_link
            # and gate by Euclidean distance in camera_link. This avoids the old
            # range-only heuristic that ignored the camera mounting offset.
            pts_to_plot = pts_link_all
            if target_name in ground_truth:
                expected_base = ground_truth[target_name]
                threshold = 0.2 * float(np.linalg.norm(expected_base))

                if args.camera_tf:
                    camera_tf_trans = args.camera_tf[:3]
                    camera_tf_rot = args.camera_tf[3:]
                    expected_cam = expected_point_in_camera_link(
                        expected_base, camera_tf_trans, camera_tf_rot
                    )
                    if pts_link_all.size:
                        d = np.linalg.norm(pts_link_all - expected_cam.reshape(1, 3), axis=1)
                        pts_to_plot = pts_link_all[d <= threshold]
                else:
                    # Fallback: range-only gate (less accurate if camera is offset from base).
                    if pts_link_all.size:
                        expected_r = float(np.linalg.norm(expected_base))
                        d_r = np.abs(np.linalg.norm(pts_link_all, axis=1) - expected_r)
                        pts_to_plot = pts_link_all[d_r <= threshold]

            if pts_to_plot.size:
                ax3.scatter(
                    pts_to_plot[:, 0],
                    pts_to_plot[:, 1],
                    label=f'{target_name} (n={len(pts_to_plot)})',
                    alpha=0.5,
                    s=20,
                    color=colors.get(target_name, 'blue'),
                )
        
        # Plot ground truth
        for name, pos in ground_truth.items():
            ax3.scatter(
                pos[0], pos[1],
                marker='X',
                s=100,                    # smaller
                facecolors='none',       # hollow X
                edgecolors='black',
                linewidths=2.0,
                alpha=0.8,
                label=f'{name} (truth)',
                zorder=1                 # behind measurements
            )
        
        ax3.set_xlabel('X (m)', fontsize=12, fontweight='bold')
        ax3.set_ylabel('Y (m)', fontsize=12, fontweight='bold')
        ax3.set_title('Camera Detections (Before Transform)', fontsize=13, fontweight='bold')
        ax3.legend(loc='upper left', bbox_to_anchor=(1.01, 1), fontsize=10, frameon=True)
        ax3.grid(True, alpha=0.3)
        ax3.set_xlim(x_lim)
        ax3.set_ylim(y_lim)
        
        # After transform
        if args.camera_tf:
            ax4 = axes[plot_idx]
            plot_idx += 1
            
            camera_tf_trans = args.camera_tf[:3]
            camera_tf_rot = args.camera_tf[3:]
            
            for target_name, points in camera_by_tgt.items():
                points_link = camera_points_as_camera_link(points)
                transformed = apply_transform(points_link, camera_tf_trans, camera_tf_rot)
                gated = gate_detections(transformed, target_name, ground_truth)
                if gated:
                    gated = np.array(gated)
                    ax4.scatter(
                        gated[:, 0],
                        gated[:, 1],
                        label=f'{target_name} (n={len(gated)})',
                        alpha=0.5,
                        s=20,
                        color=colors.get(target_name, 'blue'),
                    )

            # Plot ground truth
            for name, pos in ground_truth.items():
                ax4.scatter(
                    pos[0], pos[1],
                    marker='X',
                    s=100,                    # smaller
                    facecolors='none',       # hollow X
                    edgecolors='black',
                    linewidths=2.0,
                    alpha=0.8,
                    label=f'{name} (truth)',
                    zorder=1                 # behind measurements
                )

            ax4.set_xlabel('X (m)', fontsize=12, fontweight='bold')
            ax4.set_ylabel('Y (m)', fontsize=12, fontweight='bold')
            ax4.set_title('Camera Detections (After Transform)', fontsize=13, fontweight='bold')
            ax4.legend(loc='upper left', bbox_to_anchor=(1.01, 1), fontsize=10, frameon=True)
            ax4.grid(True, alpha=0.3)
            ax4.set_xlim(x_lim)
            ax4.set_ylim(y_lim)

    plt.tight_layout()

    # Save figure
    output_file = 'calibration_log/calibration_visualization_2d.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Saved 2D visualization to {output_file}")

    headless = not os.environ.get('DISPLAY')
    if args.no_show or headless:
        if headless and not args.no_show:
            print("[INFO] DISPLAY not set; skipping interactive window (use the saved PNG)")
        return

    print("Close the plot window to continue...")
    plt.show()


if __name__ == '__main__':
    main()
