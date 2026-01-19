#!/usr/bin/env python3
"""
analyze_scenario.py

Analyze static pedestrian detection/tracking performance for one scenario.

Inputs: 4 CSVs per scenario
 - raw detections:
    columns: sensor_type, logger_t, header_t, x, y, z
 - fused detections:
    columns: logger_t, header_t, detection_type, x, y, z, distance, speed
 - tracks:
    columns: logger_t, header_t, tracking_id, x, y, z, distance, speed, age, consecutive_misses
 - ego odometry:
    columns: logger_t, header_t, x, y, yaw, v, yaw_rate

All positions assumed to be in body frame (x forward, y left) for detections/tracks,
and in world frame for ego x, y, yaw.

Static pedestrian ground truth positions are configured per scenario in world frame
below in STATIC_GT_WORLD and are transformed into body frame using ego odometry.
"""

import argparse
import os
from dataclasses import dataclass
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ------------------------------------------------------------------------------
# Configuration: static ground-truth positions (world frame)
# ------------------------------------------------------------------------------

STATIC_GT_WORLD: Dict[str, List[Dict[str, float]]] = {
    "S1": [
        {"id": "P1", "x_world": 30.0, "y_world": 0.0},
    ],
    "S2": [
        {"id": "P1", "x_world": 22.0, "y_world": 5.0},
        {"id": "P2", "x_world": 30.0, "y_world": 2.0},
        {"id": "P3", "x_world": 17.0, "y_world": -5.0},
    ],
    "S3": [
        # S3 has moving actors; we skip static GT analysis here for now
    ],
    "S4": [
        {"id": "P1", "x_world": 20.0, "y_world": -5.0},
        {"id": "P2", "x_world": 20.0, "y_world": 0.0},
        {"id": "P3", "x_world": 20.0, "y_world": 5.0},
    ],
}


@dataclass
class AnalysisConfig:
    max_assoc_dist: float = (
        1.5  # [m] maximum distance to associate detection to a static GT
    )
    use_header_time: bool = True  # use header_t for analysis timestamps
    fov_x_min: float = 0.0  # [m] in front of tractor
    fov_x_max: float = 30.0  # [m]
    fov_half_angle_deg: float = 60.0  # [deg] half-angle around +x


# ------------------------------------------------------------------------------
# Loading helpers
# ------------------------------------------------------------------------------


def load_raw_detections(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    # Expect at least: sensor_type, logger_t, header_t, x, y, z
    # Rename to consistent internal names
    rename_map = {}
    if "header_stamp" in df.columns:
        rename_map["header_stamp"] = "header_t"
    if "logger_stamp" in df.columns:
        rename_map["logger_stamp"] = "logger_t"
    df = df.rename(columns=rename_map)

    required = ["sensor_type", "logger_t", "header_t", "x", "y"]
    for col in required:
        if col not in df.columns:
            raise ValueError(f"raw detections CSV must have column '{col}'")
    df_radar = df[df["sensor_type"] == "radar"].copy()
    df_camera = df[df["sensor_type"] == "camera"].copy()
    # Transform radar coordinates
    RADAR_TO_BODY_DX = 0.6  # from static TF
    RADAR_TO_BODY_DY = 0.0  # assume centered
    if not df_radar.empty:
        df_radar["x"] = df_radar["x"] + RADAR_TO_BODY_DX
        df_radar["y"] = df_radar["y"] - RADAR_TO_BODY_DY
    # Transform camera coordinates
    CAMERA_TO_BODY_DX = 0.5  # from static TF
    CAMERA_TO_BODY_DY = 0.0  # assume centered

    if not df_camera.empty:
        old_x = df_camera["x"].to_numpy()
        old_y = df_camera["y"].to_numpy()
        old_z = df_camera["z"].to_numpy()

        df_camera["x"] = old_z + CAMERA_TO_BODY_DX
        df_camera["y"] = -old_x - CAMERA_TO_BODY_DY
        df_camera["z"] = -old_y

    return df_radar, df_camera


def load_fused(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    rename_map = {}
    if "header_stamp" in df.columns:
        rename_map["header_stamp"] = "header_t"
    if "logger_stamp" in df.columns:
        rename_map["logger_stamp"] = "logger_t"
    df = df.rename(columns=rename_map)

    required = ["logger_t", "header_t", "x", "y"]
    for col in required:
        if col not in df.columns:
            raise ValueError(f"fused detections CSV must have column '{col}'")
    return df


def load_tracks(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    rename_map = {}
    if "header_stamp" in df.columns:
        rename_map["header_stamp"] = "header_t"
    if "logger_stamp" in df.columns:
        rename_map["logger_stamp"] = "logger_t"
    df = df.rename(columns=rename_map)

    required = ["logger_t", "header_t", "tracking_id", "x", "y"]
    for col in required:
        if col not in df.columns:
            raise ValueError(f"tracks CSV must have column '{col}'")
    return df


def load_ego(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    rename_map = {}
    if "header_stamp" in df.columns:
        rename_map["header_stamp"] = "header_t"
    if "logger_stamp" in df.columns:
        rename_map["logger_stamp"] = "logger_t"
    df = df.rename(columns=rename_map)

    required = ["logger_t", "header_t", "x", "y", "yaw"]
    for col in required:
        if col not in df.columns:
            raise ValueError(f"ego odometry CSV must have column '{col}'")
    # Sort by time
    df = df.sort_values("header_t")
    return df


# ------------------------------------------------------------------------------
# Ego pose interpolation and GT transform
# ------------------------------------------------------------------------------


def make_ego_interpolator(ego_df: pd.DataFrame) -> Tuple:
    """
    Return callables x_ego(t), y_ego(t), yaw_ego(t) using 1D interpolation
    on header_t.
    """
    t = ego_df["header_t"].values
    x = ego_df["x"].values
    y = ego_df["y"].values
    yaw_raw = ego_df["yaw"].values
    yaw_unwrapped = np.unwrap(yaw_raw)

    def x_ego(tq: np.ndarray) -> np.ndarray:
        return np.interp(tq, t, x)

    def y_ego(tq: np.ndarray) -> np.ndarray:
        return np.interp(tq, t, y)

    def yaw_ego(tq: np.ndarray) -> np.ndarray:
        return np.interp(tq, t, yaw_unwrapped)

    return x_ego, y_ego, yaw_ego


def gt_static_body_positions_at_time(
    t: float,
    scenario: str,
    x_ego_fn,
    y_ego_fn,
    yaw_ego_fn,
) -> List[Tuple[str, float, float]]:
    """
    For a given time t, compute static pedestrians' positions in body frame.
    Returns list of (id, x_body, y_body).
    """
    gt_list = STATIC_GT_WORLD.get(scenario, [])
    if not gt_list:
        return []

    x_ego = float(x_ego_fn(np.array([t]))[0])
    y_ego = float(y_ego_fn(np.array([t]))[0])
    yaw_ego = float(yaw_ego_fn(np.array([t]))[0])

    cos_yaw = np.cos(-yaw_ego)
    sin_yaw = np.sin(-yaw_ego)

    result = []
    for gt in gt_list:
        xw = gt["x_world"]
        yw = gt["y_world"]
        dx = xw - x_ego
        dy = yw - y_ego
        xb = cos_yaw * dx - sin_yaw * dy
        yb = sin_yaw * dx + cos_yaw * dy
        result.append((gt["id"], xb, yb))
    return result


# ------------------------------------------------------------------------------
# Metrics: association + MSE
# ------------------------------------------------------------------------------


def compute_gt_presence_on_grid(
    scenario: str,
    time_grid: np.ndarray,
    x_ego_fn,
    y_ego_fn,
    yaw_ego_fn,
    cfg: AnalysisConfig,
) -> Dict[str, np.ndarray]:
    """
    For each static GT pedestrian, compute the times (from time_grid)
    when it is inside the fan-shaped FOV in the body frame.

    FOV is a forward sector:
      x in [fov_x_min, fov_x_max],
      |y| <= x * tan(fov_half_angle)
    """
    presence: Dict[str, List[float]] = {}
    gt_cfg = STATIC_GT_WORLD.get(scenario, [])
    if not gt_cfg:
        return {}

    # precompute tan(theta)
    theta = np.deg2rad(cfg.fov_half_angle_deg)
    tan_theta = np.tan(theta)

    for t in time_grid:
        gt_list = gt_static_body_positions_at_time(
            t, scenario, x_ego_fn, y_ego_fn, yaw_ego_fn
        )
        for gid, xb, yb in gt_list:
            # only consider objects in front of tractor
            if xb < cfg.fov_x_min or xb > cfg.fov_x_max:
                continue
            # fan-shaped lateral limit: |y| <= x * tan(theta)
            y_limit = xb * tan_theta
            if abs(yb) > y_limit:
                continue

            presence.setdefault(gid, []).append(float(t))

    # convert lists to numpy arrays
    return {gid: np.asarray(ts) for gid, ts in presence.items()}


def compute_error_stats(
    df_with_gt: pd.DataFrame,
    label: str,
    radii: List[float] = [0.5, 1.0],
    quantiles: List[float] = [0.9, 0.95, 0.99],
) -> Dict[str, float]:
    """
    Compute probability-of-distance and confidence radii statistics
    for a dataset with 'error' and 'gt_id' columns.

    Returns a dict with:
        - n_associated
        - mse, rmse
        - p_within_<r> for each r in radii
        - radius_q<q> for each q in quantiles
    """
    stats: Dict[str, float] = {}

    valid_mask = df_with_gt["error"].notna()
    errors = df_with_gt.loc[valid_mask, "error"].values
    n_assoc = errors.size
    stats["n_associated"] = int(n_assoc)

    if n_assoc == 0:
        stats["mse"] = float("nan")
        stats["rmse"] = float("nan")
        for r in radii:
            stats[f"p_within_{r:.2f}m"] = float("nan")
        for q in quantiles:
            stats[f"radius_q{int(q*100)}"] = float("nan")
        print(f"[{label}] No associated samples for error stats.")
        return stats

    mse = float(np.mean(errors**2))
    rmse = float(np.sqrt(mse))
    stats["mse"] = mse
    stats["rmse"] = rmse

    # Probability of being within given radii
    for r in radii:
        p = float(np.mean(errors <= r))
        stats[f"p_within_{r:.2f}m"] = p

    # Quantile radii
    for q in quantiles:
        rq = float(np.quantile(errors, q))
        stats[f"radius_q{int(q*100)}"] = rq

    print(
        f"[{label}] RMSE={rmse:.3f} m; "
        + ", ".join([f"P(|e|<={r}m)={stats[f'p_within_{r:.2f}m']:.3f}" for r in radii])
        + ", "
        + ", ".join(
            [
                f"r_{int(q*100)}={stats[f'radius_q{int(q*100)}']:.3f} m"
                for q in quantiles
            ]
        )
    )
    return stats


def detection_metrics_from_df(
    df_with_gt: pd.DataFrame,
    label: str,
    scenario: str,
    x_ego_fn,
    y_ego_fn,
    yaw_ego_fn,
    cfg: AnalysisConfig,
    radii: List[float] = [0.5, 1.0],
    quantiles: List[float] = [0.9, 0.95, 0.99],
) -> Dict[str, float]:
    """
    Compute detection-level metrics for a dataset that already has GT
    association info (gt_id, error), including:
      - n_total, assoc_ratio
      - false detections per second
      - approximate recall over GT presence times
      - error-based stats (RMSE, P(|e|<=r), quantile radii)
    """
    metrics: Dict[str, float] = {}

    n_total = len(df_with_gt)
    metrics["n_total"] = n_total

    # Error-based stats: n_associated, mse, rmse, P(|e|<=r), radius_qXX
    err_stats = compute_error_stats(df_with_gt, label, radii=radii, quantiles=quantiles)
    metrics.update(err_stats)  # adds n_associated, mse, rmse, etc.

    n_assoc = metrics["n_associated"]

    # False detections per second (unmatched samples)
    t_min = df_with_gt["header_t"].min()
    t_max = df_with_gt["header_t"].max()
    duration = float(t_max - t_min) if n_total > 1 else 0.0
    n_unmatched = n_total - n_assoc
    if duration > 0.0:
        metrics["false_per_sec"] = n_unmatched / duration
    else:
        metrics["false_per_sec"] = float("nan")

    # Association ratio
    metrics["assoc_ratio"] = n_assoc / n_total if n_total > 0 else float("nan")

    # Approximate recall: fraction of GT presence times that have at least one associated detection
    # Approximate recall: fraction of fusion time steps (on this stream) where
    # a GT in FOV has at least one associated detection.
    recall_per_gt = []
    # Use the detection timestamps as the grid
    data_times = np.sort(df_with_gt["header_t"].values)
    if data_times.size > 0:
        gt_presence = compute_gt_presence_on_grid(
            scenario, data_times, x_ego_fn, y_ego_fn, yaw_ego_fn, cfg
        )

        for gid, presence_times in gt_presence.items():
            if presence_times.size == 0:
                continue

            # rows associated with this GT
            df_gid = df_with_gt[
                (df_with_gt["gt_id"] == gid) & df_with_gt["error"].notna()
            ]
            if df_gid.empty:
                recall_per_gt.append(0.0)
                continue

            assoc_times = np.sort(df_gid["header_t"].values)

            # count presence timestamps that have an associated detection at the same time
            eps = 1e-6
            covered = 0
            for t in presence_times:
                if np.any(np.abs(assoc_times - t) <= eps):
                    covered += 1
            recall_g = covered / presence_times.size
            recall_per_gt.append(recall_g)

    metrics["recall_mean"] = (
        float(np.mean(recall_per_gt)) if recall_per_gt else float("nan")
    )

    print(
        f"[{label}] assoc_ratio={metrics['assoc_ratio']:.3f}, "
        f"RMSE={metrics['rmse']:.3f} m, false_per_sec={metrics['false_per_sec']:.3f}, "
        f"recall={metrics['recall_mean']:.3f}"
    )
    return metrics


def track_metrics_from_df(
    tracks_with_gt: pd.DataFrame,
    ego_df: pd.DataFrame,
    scenario: str,
    cfg: AnalysisConfig,
    radii: List[float] = [0.5, 1.0],
    quantiles: List[float] = [0.9, 0.95, 0.99],
) -> Dict[str, float]:
    metrics: Dict[str, float] = {}

    # error-based metrics (RMSE, radii, etc.)
    err_stats = compute_error_stats(
        tracks_with_gt, "tracks", radii=radii, quantiles=quantiles
    )
    metrics.update(err_stats)

    # Build interpolators for ego pose
    x_ego_fn, y_ego_fn, yaw_ego_fn = make_ego_interpolator(ego_df)

    # Use the track timestamps as the grid for presence
    track_times = np.sort(tracks_with_gt["header_t"].values)
    gt_presence = compute_gt_presence_on_grid(
        scenario, track_times, x_ego_fn, y_ego_fn, yaw_ego_fn, cfg
    )

    availability_list = []
    latency_list = []
    id_switches_list = []

    for gid, presence_times in gt_presence.items():
        if presence_times.size == 0:
            continue

        # rows associated with this GT
        df_gid = tracks_with_gt[
            (tracks_with_gt["gt_id"] == gid) & tracks_with_gt["error"].notna()
        ]
        if df_gid.empty:
            availability_list.append(0.0)
            latency_list.append(float("nan"))
            id_switches_list.append(0)
            continue

        assoc_times = np.sort(df_gid["header_t"].values)

        # Because we built presence_times from track_times, exact matches
        # should exist when GT is tracked; we can just compare using a tiny epsilon.
        eps = 1e-6
        covered = 0
        for t in presence_times:
            # check if any associated track time is "basically equal" to t
            if np.any(np.abs(assoc_times - t) <= eps):
                covered += 1
        availability = covered / presence_times.size
        availability_list.append(availability)

        # initiation latency: first presence time vs first associated track time
        t0 = float(np.min(presence_times))
        t1 = float(np.min(assoc_times))
        latency = max(0.0, t1 - t0)
        latency_list.append(latency)

        # ID switches: count changes in tracking_id on associated rows
        df_gid_sorted = df_gid.sort_values("header_t")
        track_ids = df_gid_sorted["tracking_id"].values
        switches = int(np.sum(track_ids[1:] != track_ids[:-1]))
        id_switches_list.append(switches)

    if availability_list:
        metrics["availability_mean"] = float(np.mean(availability_list))
    else:
        metrics["availability_mean"] = float("nan")

    if latency_list:
        lat_vals = np.array([v for v in latency_list if not np.isnan(v)])
        metrics["latency_mean"] = (
            float(np.mean(lat_vals)) if lat_vals.size > 0 else float("nan")
        )
    else:
        metrics["latency_mean"] = float("nan")

    if id_switches_list:
        metrics["id_switches_total"] = int(np.sum(id_switches_list))
        metrics["id_switches_per_gt_mean"] = float(np.mean(id_switches_list))
    else:
        metrics["id_switches_total"] = 0
        metrics["id_switches_per_gt_mean"] = 0.0

    # spurious tracks and residual speed as before...
    all_track_ids = set(tracks_with_gt["tracking_id"].unique())
    associated_ids = set(
        tracks_with_gt.loc[tracks_with_gt["gt_id"].notna(), "tracking_id"].unique()
    )
    spurious_ids = all_track_ids - associated_ids
    metrics["n_spurious_tracks"] = len(spurious_ids)

    if "speed" in tracks_with_gt.columns:
        speeds = tracks_with_gt.loc[tracks_with_gt["gt_id"].notna(), "speed"].values
        if speeds.size > 0:
            metrics["speed_mean"] = float(np.mean(speeds))
            metrics["speed_p95"] = float(np.percentile(speeds, 95))
        else:
            metrics["speed_mean"] = float("nan")
            metrics["speed_p95"] = float("nan")
    else:
        metrics["speed_mean"] = float("nan")
        metrics["speed_p95"] = float("nan")

    print(
        f"[tracks] RMSE={metrics['rmse']:.3f} m, "
        f"availability_mean={metrics['availability_mean']:.3f}, "
        f"latency_mean={metrics['latency_mean']:.2f} s, "
        f"id_switches_total={metrics['id_switches_total']}, "
        f"spurious_tracks={metrics['n_spurious_tracks']}, "
        f"speed_mean={metrics['speed_mean']:.3f} m/s"
    )

    return metrics


def write_metrics_log(
    out_prefix: str,
    scenario: str,
    radar_metrics: Dict[str, float],
    camera_metrics: Dict[str, float],
    fused_metrics: Dict[str, float],
    track_metrics: Dict[str, float],
) -> None:
    """
    Write all metrics into a simple text log file.
    """
    log_path = f"{out_prefix}_metrics.txt"
    with open(log_path, "w") as f:
        f.write(f"Scenario: {scenario}\n\n")

        def dump_block(name, m):
            f.write(f"[{name}]\n")
            for k, v in m.items():
                f.write(f"{k}: {v}\n")
            f.write("\n")

        dump_block("radar", radar_metrics)
        dump_block("camera", camera_metrics)
        dump_block("fused", fused_metrics)
        dump_block("tracks", track_metrics)

    print(f"Metrics written to {log_path}")


def compute_mse_for_dataset(
    df: pd.DataFrame,
    scenario: str,
    x_ego_fn,
    y_ego_fn,
    yaw_ego_fn,
    cfg: AnalysisConfig,
    label: str,
) -> Tuple[float, pd.DataFrame]:
    """
    Compute mean squared error for a detection dataset (radar/camera/fused/tracks)
    relative to static ground truth for a given scenario.

    Returns (mse, df_with_gt) where df_with_gt has extra columns:
      gt_id, gt_x_body, gt_y_body, error
    """
    if scenario not in STATIC_GT_WORLD or not STATIC_GT_WORLD[scenario]:
        print(f"[{label}] No static GT defined for scenario {scenario}, skipping MSE.")
        return float("nan"), df

    times = df["header_t"].values
    xs = df["x"].values
    ys = df["y"].values

    gt_ids = []
    gt_xb = []
    gt_yb = []
    errors = []

    for t, xd, yd in zip(times, xs, ys):
        gt_list = gt_static_body_positions_at_time(
            t, scenario, x_ego_fn, y_ego_fn, yaw_ego_fn
        )
        if not gt_list:
            gt_ids.append(None)
            gt_xb.append(np.nan)
            gt_yb.append(np.nan)
            errors.append(np.nan)
            continue

        # Find closest static GT
        dists = []
        for gid, xb, yb in gt_list:
            d = np.hypot(xd - xb, yd - yb)
            dists.append((d, gid, xb, yb))
        d, gid, xb, yb = min(dists, key=lambda x: x[0])

        if d <= cfg.max_assoc_dist:
            gt_ids.append(gid)
            gt_xb.append(xb)
            gt_yb.append(yb)
            errors.append(d)
        else:
            # too far -> treat as unmatched (no GT)
            gt_ids.append(None)
            gt_xb.append(np.nan)
            gt_yb.append(np.nan)
            errors.append(np.nan)

    df_out = df.copy()
    df_out["gt_id"] = gt_ids
    df_out["gt_x_body"] = gt_xb
    df_out["gt_y_body"] = gt_yb
    df_out["error"] = errors

    valid = df_out["error"].notna()
    if valid.sum() == 0:
        print(
            f"[{label}] No associated detections within {cfg.max_assoc_dist} m, MSE is NaN."
        )
        return float("nan"), df_out

    sq_errors = df_out.loc[valid, "error"].values ** 2
    mse = float(np.mean(sq_errors))
    print(
        f"[{label}] MSE (static GT)"
        f" = {mse:.3f} m^2 ({valid.sum()} associated detections)"
    )
    return mse, df_out


def compute_error_stats(
    df_with_gt: pd.DataFrame,
    label: str,
    radii: List[float] = [0.5, 1.0],
    quantiles: List[float] = [0.9, 0.95, 0.99],
) -> Dict[str, float]:
    """
    Compute probability-of-distance and confidence radii statistics
    for a dataset with 'error' and 'gt_id' columns.

    Returns a dict with:
        - n_assoc
        - rmse
        - p_within_<r> for each r in radii
        - radius_q<q> for each q in quantiles
    """
    stats: Dict[str, float] = {}

    # Only consider samples that have a GT association
    valid_mask = df_with_gt["error"].notna()
    errors = df_with_gt.loc[valid_mask, "error"].values

    n_assoc = errors.size
    stats["n_associated"] = int(n_assoc)

    if n_assoc == 0:
        # No associated samples -> everything NaN
        stats["rmse"] = float("nan")
        for r in radii:
            stats[f"p_within_{r:.2f}m"] = float("nan")
        for q in quantiles:
            stats[f"radius_q{int(q*100)}"] = float("nan")
        print(f"[{label}] No associated samples for error stats.")
        return stats

    # RMSE
    mse = float(np.mean(errors**2))
    rmse = float(np.sqrt(mse))
    stats["rmse"] = rmse

    # Probability of being within given radii
    for r in radii:
        p = float(np.mean(errors <= r))
        stats[f"p_within_{r:.2f}m"] = p

    # Quantile radii
    for q in quantiles:
        rq = float(np.quantile(errors, q))
        stats[f"radius_q{int(q*100)}"] = rq

    print(
        f"[{label}] RMSE={rmse:.3f} m; "
        + ", ".join([f"P(|e|<={r}m)={stats[f'p_within_{r:.2f}m']:.3f}" for r in radii])
        + ", "
        + ", ".join(
            [
                f"r_{int(q*100)}={stats[f'radius_q{int(q*100)}']:.3f} m"
                for q in quantiles
            ]
        )
    )
    return stats


# ------------------------------------------------------------------------------
# Plotting
# ------------------------------------------------------------------------------


def plot_xy(
    scenario: str,
    radar_df: pd.DataFrame,
    camera_df: pd.DataFrame,
    fused_df_with_gt: pd.DataFrame,
    tracks_df: pd.DataFrame,
    out_prefix: str,
):
    """
    Make the four requested plots in body frame (x forward, y left).
    gt_samples: array of shape (N_gt, 2) with (x, y) GT centre samples.
    """

    # Ensure output directory exists
    out_dir = os.path.dirname(out_prefix)
    if out_dir and not os.path.exists(out_dir):
        os.makedirs(out_dir, exist_ok=True)

    # 1) raw + fused + GT
    if scenario != "S3":
        plt.figure(figsize=(8, 12))
        plt.scatter(
            fused_df_with_gt["gt_y_body"],
            fused_df_with_gt["gt_x_body"],
            marker=".",
            s=2,
            color="black",
            label="GT static",
        )
    plt.scatter(
        fused_df_with_gt["y"],
        fused_df_with_gt["x"],
        marker=".",
        color="r",
        s=2,
        alpha=0.7,
        label="Fused",
    )
    # Plot lines from fused detections to corresponding GT
    if scenario != "S3":
        for _, row in fused_df_with_gt.iterrows():
            if pd.isna(row["gt_x_body"]) or pd.isna(row["gt_y_body"]):
                continue
            plt.plot(
                [row["gt_y_body"], row["y"]],
                [row["gt_x_body"], row["x"]],
                color="gray",
                linewidth=0.5,
                alpha=0.5,
            )
    plt.scatter(
        radar_df["y"],
        radar_df["x"],
        marker=".",
        color="g",
        s=2,
        alpha=0.7,
        label="Radar detections",
    )
    plt.scatter(
        camera_df["y"],
        camera_df["x"],
        marker=".",
        color="y",
        s=2,
        alpha=0.7,
        label="Camera detections",
    )

    plt.xlabel("y [m]")
    plt.ylabel("x [m]")
    if scenario != "S3":
        plt.title(f"{scenario}: Raw & Fused vs GT")
    else:
        plt.title(f"{scenario}: Raw & Fused")
    plt.legend()
    if scenario == "S1":
        plt.xlim(-0.5, 0.5)
        plt.ylim(-2, 25)
    else:
        plt.ylim(-2, 30)
    plt.tight_layout()
    plt.savefig(f"{out_prefix}_raw_fused_vs_gt.png", dpi=300)
    plt.close()

    # 2) GT + fused + tracks
    if scenario != "S3":
        plt.figure(figsize=(8, 12))
        plt.scatter(
            fused_df_with_gt["gt_y_body"],
            fused_df_with_gt["gt_x_body"],
            marker=".",
            s=2,
            color="black",
            label="GT static",
        )
    if not fused_df_with_gt.empty:
        plt.scatter(
            fused_df_with_gt["y"],
            fused_df_with_gt["x"],
            marker=".",
            color="r",
            s=2,
            alpha=0.7,
            label="Fused",
        )
    # Plot lines from fused detections to corresponding GT
    if scenario != "S3":
        for _, row in fused_df_with_gt.iterrows():
            if pd.isna(row["gt_x_body"]) or pd.isna(row["gt_y_body"]):
                continue
            plt.plot(
                [row["gt_y_body"], row["y"]],
                [row["gt_x_body"], row["x"]],
                color="gray",
                linewidth=0.5,
                alpha=0.5,
            )
    if not tracks_df.empty:
        plt.scatter(
            tracks_df["y"],
            tracks_df["x"],
            marker=".",
            color="m",
            s=2,
            alpha=0.7,
            label="Tracks",
        )
    plt.xlabel("y [m]")
    plt.ylabel("x [m]")
    if scenario != "S3":
        plt.title(f"{scenario}: Fused & Tracks vs GT")
    else:
        plt.title(f"{scenario}: Fused & Tracks")
    plt.legend()
    if scenario == "S1":
        plt.xlim(-0.5, 0.5)
        plt.ylim(-2, 25)
    else:
        plt.ylim(-2, 30)
    plt.tight_layout()
    plt.savefig(f"{out_prefix}_fused_tracks_vs_gt.png", dpi=300)
    plt.close()

    # 3) GT + tracks
    if scenario != "S3":
        plt.figure(figsize=(8, 12))
        plt.scatter(
            tracks_df["gt_y_body"],
            tracks_df["gt_x_body"],
            marker=".",
            s=2,
            color="black",
            label="GT static",
        )
    if not tracks_df.empty:
        plt.scatter(
            tracks_df["y"],
            tracks_df["x"],
            marker=".",
            color="m",
            s=2,
            alpha=0.7,
            label="Tracks",
        )
    # Plot lines from tracks to corresponding GT
    if scenario != "S3":
        for _, row in tracks_df.iterrows():
            if pd.isna(row["gt_x_body"]) or pd.isna(row["gt_y_body"]):
                continue
            plt.plot(
                [row["gt_y_body"], row["y"]],
                [row["gt_x_body"], row["x"]],
                color="gray",
                linewidth=0.5,
                alpha=0.5,
            )
    plt.xlabel("y [m]")
    plt.ylabel("x [m]")
    if scenario != "S3":
        plt.title(f"{scenario}: Tracks vs GT")
    else:
        plt.title(f"{scenario}: Tracks")
    plt.legend()
    if scenario == "S1":
        plt.xlim(-0.5, 0.5)
        plt.ylim(-2, 25)
    else:
        plt.ylim(-2, 30)
    plt.tight_layout()
    plt.savefig(f"{out_prefix}_tracks_vs_gt.png", dpi=300)
    plt.close()


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scenario", required=True, help="Scenario name (e.g. S1, S2, S4)"
    )
    parser.add_argument("--raw", required=True, help="Path to raw_detections CSV")
    parser.add_argument("--fused", required=True, help="Path to fused_detections CSV")
    parser.add_argument("--tracks", required=True, help="Path to tracks CSV")
    parser.add_argument("--ego", required=True, help="Path to ego_odometry CSV")
    parser.add_argument(
        "--out_prefix", required=True, help="Prefix for output figures (e.g. plots/S1)"
    )
    args = parser.parse_args()

    scenario = args.scenario
    cfg = AnalysisConfig()

    if scenario not in STATIC_GT_WORLD or (
        scenario != "S3" and not STATIC_GT_WORLD[scenario]
    ):
        print(
            f"WARNING: No static GT configured for scenario {scenario}. "
            f"Fill STATIC_GT_WORLD in this script before running."
        )

    print(f"Analyzing scenario {scenario}")

    radar_df, camera_df = load_raw_detections(args.raw)
    fused_df = load_fused(args.fused)
    tracks_df = load_tracks(args.tracks)
    ego_df = load_ego(args.ego)

    x_ego_fn, y_ego_fn, yaw_ego_fn = make_ego_interpolator(ego_df)

    # Compute MSEs for each dataset
    mse_radar, radar_with_gt = compute_mse_for_dataset(
        radar_df, scenario, x_ego_fn, y_ego_fn, yaw_ego_fn, cfg, label="radar"
    )
    mse_camera, camera_with_gt = compute_mse_for_dataset(
        camera_df, scenario, x_ego_fn, y_ego_fn, yaw_ego_fn, cfg, label="camera"
    )
    mse_fused, fused_with_gt = compute_mse_for_dataset(
        fused_df, scenario, x_ego_fn, y_ego_fn, yaw_ego_fn, cfg, label="fused"
    )
    mse_tracks, tracks_with_gt = compute_mse_for_dataset(
        tracks_df, scenario, x_ego_fn, y_ego_fn, yaw_ego_fn, cfg, label="tracks"
    )

    # Write a .csv with fused_with_gt and tracks_with_gt for further analysis if needed
    # fused_with_gt.to_csv(f"{args.out_prefix}_fused_with_gt.csv", index=False)
    # tracks_with_gt.to_csv(f"{args.out_prefix}_tracks_with_gt.csv", index=False)

    print(f"\nScenario {scenario} MSE summary (static GT):")
    print(
        f"  Radar detections: {mse_radar:.3f} m^2" f" (RMSE {np.sqrt(mse_radar):.3f} m)"
    )
    print(
        f"  Camera detections: {mse_camera:.3f} m^2"
        f" (RMSE {np.sqrt(mse_camera):.3f} m)"
    )
    print(
        f"  Fused detections: {mse_fused:.3f} m^2" f" (RMSE {np.sqrt(mse_fused):.3f} m)"
    )
    print(f"  Tracks: {mse_tracks:.3f} m^2" f" (RMSE {np.sqrt(mse_tracks):.3f} m)")
    if scenario != "S3":
        # Compute detection-level metrics
        radar_metrics = detection_metrics_from_df(
            radar_with_gt, "radar", scenario, x_ego_fn, y_ego_fn, yaw_ego_fn, cfg
        )
        camera_metrics = detection_metrics_from_df(
            camera_with_gt, "camera", scenario, x_ego_fn, y_ego_fn, yaw_ego_fn, cfg
        )
        fused_metrics = detection_metrics_from_df(
            fused_with_gt, "fused", scenario, x_ego_fn, y_ego_fn, yaw_ego_fn, cfg
        )

        # Track-level metrics
        track_metrics = track_metrics_from_df(tracks_with_gt, ego_df, scenario, cfg)

        # Write everything to a log file
        write_metrics_log(
            args.out_prefix,
            scenario,
            radar_metrics,
            camera_metrics,
            fused_metrics,
            track_metrics,
        )
    # Draw plots
    plot_xy(
        scenario,
        radar_df,
        camera_df,
        fused_with_gt,
        tracks_with_gt,
        out_prefix=args.out_prefix,
    )


if __name__ == "__main__":
    main()
