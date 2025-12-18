#!/usr/bin/env python3
"""
test_metrics_sanity.py

Sanity tests for the metrics functions in analyze_scenario_static.py.

This does NOT touch any CSVs or real data. It builds tiny synthetic
DataFrames where we know the correct answers and checks that:

  - error statistics (RMSE, P(|e| <= r)) are correct
  - FOV "presence times" for GT are correct for a fan-shaped FOV
  - detection_metrics_from_df computes sensible assoc_ratio, recall, etc.
  - track_metrics_from_df computes availability, latency, ID switches, etc.

Run with:
    python3 test_metrics_sanity.py
"""

import numpy as np
import pandas as pd

# Adjust this import to match your actual file name:
# from analyze_scenarios import ...
from analyze_scenario import (
    AnalysisConfig,
    STATIC_GT_WORLD,
    make_ego_interpolator,
    compute_gt_presence_times,
    detection_metrics_from_df,
    track_metrics_from_df,
    compute_error_stats,  # make sure you've added this
)


def assert_close(a, b, tol=1e-6, msg=""):
    if np.isnan(a) and np.isnan(b):
        return
    if abs(a - b) > tol:
        raise AssertionError(f"{msg} | {a} != {b} (tol={tol})")


def test_error_stats_simple():
    print("=== test_error_stats_simple ===")
    # Perfect detections: error = 0 for all samples
    df = pd.DataFrame(
        {
            "header_t": [0.0, 1.0, 2.0],
            "x": [0.0, 1.0, 2.0],
            "y": [0.0, 0.0, 0.0],
            "gt_id": ["P1", "P1", "P1"],
            "gt_x_body": [0.0, 1.0, 2.0],
            "gt_y_body": [0.0, 0.0, 0.0],
            "error": [0.0, 0.0, 0.0],
        }
    )
    stats = compute_error_stats(df, "test", radii=[0.5], quantiles=[0.99])

    assert_close(stats["rmse"], 0.0, msg="RMSE for zero errors")
    assert_close(stats["p_within_0.50m"], 1.0, msg="P(|e|<=0.5) for zero errors")
    assert_close(stats["radius_q99"], 0.0, msg="r_99 for zero errors")
    print("OK\n")


def test_error_stats_mixed():
    print("=== test_error_stats_mixed ===")
    # Known errors: 0, 0.5, 1.0
    df = pd.DataFrame(
        {
            "header_t": [0.0, 1.0, 2.0],
            "x": [0.0, 0.5, 1.0],
            "y": [0.0, 0.0, 0.0],
            "gt_id": ["P1", "P1", "P1"],
            "gt_x_body": [0.0, 0.0, 0.0],  # doesn't matter for this test
            "gt_y_body": [0.0, 0.0, 0.0],
            "error": [0.0, 0.5, 1.0],
        }
    )

    stats = compute_error_stats(df, "test", radii=[0.5], quantiles=[0.5])
    # Manually:
    # RMSE = sqrt((0^2 + 0.5^2 + 1^2)/3) = sqrt(1.25/3)
    rmse_true = np.sqrt((0.0**2 + 0.5**2 + 1.0**2) / 3.0)
    assert_close(stats["rmse"], rmse_true, msg="RMSE for [0,0.5,1.0]")
    # P(|e| <= 0.5) = 2/3
    assert_close(stats["p_within_0.50m"], 2.0 / 3.0, msg="P(|e|<=0.5) for [0,0.5,1.0]")
    print("OK\n")


def test_fov_presence():
    print("=== test_fov_presence ===")
    # Ego at origin, stationary, yaw = 0
    ego_df = pd.DataFrame(
        {
            "header_t": [0.0, 1.0, 2.0],
            "x": [0.0, 0.0, 0.0],
            "y": [0.0, 0.0, 0.0],
            "yaw": [0.0, 0.0, 0.0],
        }
    )
    x_ego_fn, y_ego_fn, yaw_ego_fn = make_ego_interpolator(ego_df)

    cfg = AnalysisConfig(
        max_assoc_dist=1.5,
        fov_x_min=0.0,
        fov_x_max=30.0,
        fov_half_angle_deg=60.0,
    )

    # Case 1: GT straight ahead at (10, 0) -> always in FOV
    STATIC_GT_WORLD["S_fov"] = [{"id": "P1", "x_world": 10.0, "y_world": 0.0}]
    presence = compute_gt_presence_times(
        "S_fov", ego_df, x_ego_fn, y_ego_fn, yaw_ego_fn, cfg
    )
    assert "P1" in presence, "P1 should be in presence dict"
    assert_close(len(presence["P1"]), 3, msg="P1 present at all three times")

    # Case 2: GT very lateral near origin (0.5, 9.5) -> outside fan-shaped FOV
    STATIC_GT_WORLD["S_fov"] = [{"id": "P1", "x_world": 0.5, "y_world": 9.5}]
    presence2 = compute_gt_presence_times(
        "S_fov", ego_df, x_ego_fn, y_ego_fn, yaw_ego_fn, cfg
    )
    # Either P1 not present or presence list is empty
    if "P1" in presence2:
        assert_close(len(presence2["P1"]), 0, msg="P1 should not be in FOV")
    print("OK\n")


def test_detection_metrics_recall():
    print("=== test_detection_metrics_recall ===")
    # GT: P1 at (10,0), ego at origin; present at t=0,1,2
    ego_df = pd.DataFrame(
        {
            "header_t": [0.0, 1.0, 2.0],
            "x": [0.0, 0.0, 0.0],
            "y": [0.0, 0.0, 0.0],
            "yaw": [0.0, 0.0, 0.0],
        }
    )
    x_ego_fn, y_ego_fn, yaw_ego_fn = make_ego_interpolator(ego_df)

    cfg = AnalysisConfig(
        max_assoc_dist=1.5,
        fov_x_min=0.0,
        fov_x_max=30.0,
        fov_half_angle_deg=60.0,
    )

    STATIC_GT_WORLD["S_recall"] = [{"id": "P1", "x_world": 10.0, "y_world": 0.0}]

    # detections at t=0 and t=2 only (perfectly on GT)
    df_fused = pd.DataFrame(
        {
            "header_t": [0.0, 2.0],
            "x": [10.0, 10.0],
            "y": [0.0, 0.0],
            "gt_id": ["P1", "P1"],
            "gt_x_body": [10.0, 10.0],
            "gt_y_body": [0.0, 0.0],
            "error": [0.0, 0.0],
        }
    )

    metrics = detection_metrics_from_df(
        df_fused, "fused", "S_recall", ego_df, x_ego_fn, y_ego_fn, yaw_ego_fn, cfg
    )

    # We expect:
    # n_total = 2, n_associated = 2, assoc_ratio = 1
    assert_close(metrics["n_total"], 2, msg="n_total for detection test")
    assert_close(metrics["n_associated"], 2, msg="n_associated for detection test")
    assert_close(metrics["assoc_ratio"], 1.0, msg="assoc_ratio for detection test")

    # GT present at t=0,1,2. Detections at t=0 and 2 only.
    # So recall ~ 2/3.
    assert_close(metrics["recall_mean"], 2.0 / 3.0, tol=1e-2, msg="recall_mean fused")
    print("OK\n")


def test_track_metrics_simple():
    print("=== test_track_metrics_simple ===")
    # GT: P1 at (10,0), ego at origin; present at t=0,1,2
    ego_df = pd.DataFrame(
        {
            "header_t": [0.0, 1.0, 2.0],
            "x": [0.0, 0.0, 0.0],
            "y": [0.0, 0.0, 0.0],
            "yaw": [0.0, 0.0, 0.0],
        }
    )
    x_ego_fn, y_ego_fn, yaw_ego_fn = make_ego_interpolator(ego_df)

    cfg = AnalysisConfig(
        max_assoc_dist=1.5,
        fov_x_min=0.0,
        fov_x_max=30.0,
        fov_half_angle_deg=60.0,
    )

    STATIC_GT_WORLD["S_track"] = [{"id": "P1", "x_world": 10.0, "y_world": 0.0}]

    # Perfect track: tracking_id=1 at all three times, zero error, zero speed
    tracks_df = pd.DataFrame(
        {
            "header_t": [0.0, 1.0, 2.0],
            "tracking_id": [1, 1, 1],
            "x": [10.0, 10.0, 10.0],
            "y": [0.0, 0.0, 0.0],
            "gt_id": ["P1", "P1", "P1"],
            "gt_x_body": [10.0, 10.0, 10.0],
            "gt_y_body": [0.0, 0.0, 0.0],
            "error": [0.0, 0.0, 0.0],
            "speed": [0.0, 0.0, 0.0],
        }
    )

    metrics = track_metrics_from_df(tracks_df, ego_df, "S_track", cfg)

    # RMSE = 0, availability ~ 1, latency ~ 0, no ID switches, no spurious tracks
    assert_close(metrics["rmse"], 0.0, msg="track RMSE")
    assert_close(metrics["availability_mean"], 1.0, msg="track availability_mean")

    # Accept small tolerance for latency since it should be exactly 0
    assert_close(metrics["latency_mean"], 0.0, tol=1e-6, msg="track latency_mean")

    assert_close(metrics["id_switches_total"], 0, msg="id_switches_total")
    assert_close(metrics["n_spurious_tracks"], 0, msg="n_spurious_tracks")

    # Residual speed
    assert_close(metrics["speed_mean"], 0.0, msg="track speed_mean")
    print("OK\n")


if __name__ == "__main__":
    test_error_stats_simple()
    test_error_stats_mixed()
    test_fov_presence()
    test_detection_metrics_recall()
    test_track_metrics_simple()
    print("All tests passed.")
