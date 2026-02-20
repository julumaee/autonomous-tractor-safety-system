#!/usr/bin/env python3
"""
Visualize sensor calibration results in 2D (top-down view).

Plots sensor detections on x-y plane before and after applying the calibrated TF transforms.
"""

import argparse
import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation


def load_calibration_data(data_file):
    """Load sensor detections from CSV."""
    radar_points_by_target = {}
    camera_points_by_target = {}
    
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
            
            if target_name:
                if sensor_type == 'radar':
                    radar_points_by_target.setdefault(target_name, []).append(point)
                elif sensor_type == 'camera':
                    # Apply the same frame conversion as in calibration
                    x_c, y_c, z_c = point
                    point_converted = np.array([z_c, -x_c, -y_c], dtype=float)
                    camera_points_by_target.setdefault(target_name, []).append(point_converted)
    
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


def apply_transform(points, translation, rotation_euler):
    """Apply rigid transform to points."""
    if not points:
        return np.array([])
    
    R = Rotation.from_euler('xyz', rotation_euler).as_matrix()
    t = np.array(translation)
    
    points_array = np.array(points)
    transformed = (R @ points_array.T).T + t
    return transformed


def main():
    parser = argparse.ArgumentParser(description='Visualize sensor calibration results in 2D')
    parser.add_argument('--data', required=True, help='Calibration data CSV file')
    parser.add_argument('--ground-truth', required=True, help='Ground truth CSV file')
    parser.add_argument('--radar-tf', nargs=6, type=float, 
                       metavar=('X', 'Y', 'Z', 'ROLL', 'PITCH', 'YAW'),
                       help='Radar transform (tx ty tz roll pitch yaw)')
    parser.add_argument('--camera-tf', nargs=6, type=float,
                       metavar=('X', 'Y', 'Z', 'ROLL', 'PITCH', 'YAW'),
                       help='Camera transform (tx ty tz roll pitch yaw - FOR TF, includes optical correction)')
    parser.add_argument('--camera-calibrated-roll', type=float, default=-0.0770,
                       help='Camera calibrated roll (without optical correction)')
    parser.add_argument('--camera-calibrated-pitch', type=float, default=0.0938,
                       help='Camera calibrated pitch (without optical correction)')
    parser.add_argument('--camera-calibrated-yaw', type=float, default=0.0491,
                       help='Camera calibrated yaw (without optical correction)')
    parser.add_argument('--sensor', choices=['radar', 'camera', 'both'], 
                       default='both', help='Which sensor to visualize')
    
    args = parser.parse_args()
    
    print("Loading calibration data...")
    radar_by_tgt, camera_by_tgt = load_calibration_data(args.data)
    ground_truth = load_ground_truth(args.ground_truth)
    
    print(f"Ground truth targets: {list(ground_truth.keys())}")
    
    # Calculate consistent axis limits using only GATED points
    all_gated_points = []
    for target_name, points in radar_by_tgt.items():
        gated = gate_detections(points, target_name, ground_truth)
        all_gated_points.extend(gated)
    for target_name, points in camera_by_tgt.items():
        gated = gate_detections(points, target_name, ground_truth)
        all_gated_points.extend(gated)
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
    y_padding = 0.05  # Much smaller padding for y to zoom in
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
            ax1.scatter(pos[0], pos[1], marker='X', s=300, 
                       edgecolors='black', linewidths=2.5, color=colors.get(name, 'black'),
                       label=f'{name} (truth)', zorder=10)
        
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
                ax2.scatter(pos[0], pos[1], marker='X', s=300,
                           edgecolors='black', linewidths=2.5, color=colors.get(name, 'black'),
                           label=f'{name} (truth)', zorder=10)
            
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
            gated = gate_detections(points, target_name, ground_truth)
            if gated:
                gated = np.array(gated)
                ax3.scatter(gated[:, 0], gated[:, 1],
                           label=f'{target_name} (n={len(gated)})', 
                           alpha=0.5, s=20, color=colors.get(target_name, 'blue'))
        
        # Plot ground truth
        for name, pos in ground_truth.items():
            ax3.scatter(pos[0], pos[1], marker='X', s=300,
                       edgecolors='black', linewidths=2.5, color=colors.get(name, 'black'),
                       label=f'{name} (truth)', zorder=10)
        
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
            # Use calibrated-only rotation (without optical correction) for data alignment
            camera_tf_rot = [args.camera_calibrated_roll, args.camera_calibrated_pitch, args.camera_calibrated_yaw]
            
            for target_name, points in camera_by_tgt.items():
                gated = gate_detections(points, target_name, ground_truth)
                if gated:
                    transformed = apply_transform(gated, camera_tf_trans, camera_tf_rot)
                    ax4.scatter(transformed[:, 0], transformed[:, 1],
                               label=f'{target_name} (n={len(gated)})', 
                               alpha=0.5, s=20, color=colors.get(target_name, 'blue'))
            
            # Plot ground truth
            for name, pos in ground_truth.items():
                ax4.scatter(pos[0], pos[1], marker='X', s=300,
                           edgecolors='black', linewidths=2.5, color=colors.get(name, 'black'),
                           label=f'{name} (truth)', zorder=10)
            
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
    
    plt.show()


if __name__ == '__main__':
    main()
