#!/usr/bin/env python3
"""analyze_real_world.py

Real-world log analysis:
- Load GPS truth, ego_motion, raw detections (radar+camera), fused, tracks
- Apply sensor TF to raw detections to express them in `base_link`
- Build an ego world trajectory from RTK-GPS lat/lon measurements
- Estimate ego heading from GPS movement in each run
- Compute target ground truth in world frame and transform it into ego frame
- Propagate an approximate RTK-GPS / heading-based GT uncertainty estimate
- Optionally keep only rows within a configurable GT distance gate
- Compute RMSE for each dataset separately (radar/camera/fused/tracks)
- Save metrics and generate dedicated real-world plots
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path
from typing import Callable, Dict, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from analyze_scenario import load_fused, load_tracks  # type: ignore


TARGET_X_WORLD: float = 29.0
TARGET_Y_WORLD: float = 0.0
TARGET_Y_BY_SCENARIO: Dict[str, float] = {
    "t1": 0.0,
    "t2": -1.5,
    "t3": 1.5,
    "t4": -3.0,
    "t5": 3.0,
}
TARGET_WORLD_SIGMA_M: float = 0.0
GPS_HEADING_SPEED_MIN_MPS: float = 0.05
GPS_HEADING_BASELINE_S: float = 0.5
GPS_HEADING_BASELINE_MIN_M: float = 0.20


def resolve_target_world_from_test_name(test_name: str) -> Tuple[float, float]:
    """Resolve static target world position from real-world test naming.

    Mapping by scenario prefix:
      t1 -> y=0.0
      t2 -> y=-1.5
      t3 -> y=1.5
      t4 -> y=-3.0
      t5 -> y=3.0
    x is fixed at 29.0 m for all runs.
    """
    m = re.match(r"^(t[1-5])(?:_|$)", str(test_name).strip().lower())
    if not m:
        return TARGET_X_WORLD, TARGET_Y_WORLD
    scenario_key = m.group(1)
    return TARGET_X_WORLD, float(TARGET_Y_BY_SCENARIO.get(scenario_key, TARGET_Y_WORLD))


def _rpy_to_rot_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return 3x3 rotation matrix for intrinsic XYZ (roll, pitch, yaw)."""
    cr = math.cos(float(roll))
    sr = math.sin(float(roll))
    cp = math.cos(float(pitch))
    sp = math.sin(float(pitch))
    cy = math.cos(float(yaw))
    sy = math.sin(float(yaw))

    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def _parse_calibration_tf_file(tf_path: Path) -> Dict[str, np.ndarray]:
    """Parse calibration_log/calibration_tf.txt.

    Expected lines:
      radar_tf:  x y z roll pitch yaw
      camera_tf: x y z roll pitch yaw
    Values are base_link -> sensor_link (as used by runtime TF publishing).
    """
    out: Dict[str, np.ndarray] = {}
    if not tf_path.exists():
        return out

    for raw_line in tf_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("radar_tf:"):
            parts = line.split(":", 1)[1].strip().split()
            if len(parts) >= 6:
                out["radar_tf"] = np.array([float(x) for x in parts[:6]], dtype=float)
        if line.startswith("camera_tf:"):
            parts = line.split(":", 1)[1].strip().split()
            if len(parts) >= 6:
                out["camera_tf"] = np.array([float(x) for x in parts[:6]], dtype=float)
    return out


def _gps_local_xy_m(
    lat_deg: np.ndarray,
    lon_deg: np.ndarray,
    *,
    lat0_deg: float,
    lon0_deg: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Very small-area lat/lon -> local EN (meters) around the first sample."""
    # Equirectangular approximation is sufficient at field-test scale.
    lat0_rad = math.radians(float(lat0_deg))
    m_per_deg_lat = 111_320.0
    m_per_deg_lon = 111_320.0 * math.cos(lat0_rad)
    east_m = (lon_deg - float(lon0_deg)) * m_per_deg_lon
    north_m = (lat_deg - float(lat0_deg)) * m_per_deg_lat
    return east_m, north_m


def _wrap_angle(a: np.ndarray | float) -> np.ndarray | float:
    return (np.asarray(a) + math.pi) % (2.0 * math.pi) - math.pi


def _interp_series(t: np.ndarray, values: np.ndarray) -> Callable[[np.ndarray], np.ndarray]:
    t = np.asarray(t, dtype=float)
    values = np.asarray(values, dtype=float)

    def fn(tq: np.ndarray) -> np.ndarray:
        tq = np.asarray(tq, dtype=float)
        return np.interp(tq, t, values)

    return fn


def _interp_angle_series(t: np.ndarray, yaw_rad: np.ndarray) -> Callable[[np.ndarray], np.ndarray]:
    t = np.asarray(t, dtype=float)
    yaw_unwrapped = np.unwrap(np.asarray(yaw_rad, dtype=float))

    def fn(tq: np.ndarray) -> np.ndarray:
        tq = np.asarray(tq, dtype=float)
        return np.interp(tq, t, yaw_unwrapped)

    return fn


def _smooth_series_time_window(t: np.ndarray, values: np.ndarray, window_s: float) -> np.ndarray:
    """Smooth a 1D series with a centered moving average over time window."""
    t = np.asarray(t, dtype=float)
    values = np.asarray(values, dtype=float)
    if values.size < 5 or window_s <= 0.0:
        return values

    dt = np.diff(t)
    dt = dt[np.isfinite(dt) & (dt > 0.0)]
    if dt.size == 0:
        return values

    median_dt = float(np.median(dt))
    if median_dt <= 0.0:
        return values

    win = int(round(float(window_s) / median_dt))
    win = max(3, win)
    if win % 2 == 0:
        win += 1

    # Edge padding keeps endpoints stable compared to zero padding.
    pad = win // 2
    kernel = np.ones(win, dtype=float) / float(win)
    padded = np.pad(values, (pad, pad), mode="edge")
    smoothed = np.convolve(padded, kernel, mode="valid")
    return smoothed


def _smooth_angle_time_window(t: np.ndarray, angles_rad: np.ndarray, window_s: float) -> np.ndarray:
    """Smooth angle series by unwrapping -> smoothing -> wrapping."""
    unwrapped = np.unwrap(np.asarray(angles_rad, dtype=float))
    unwrapped_s = _smooth_series_time_window(t, unwrapped, window_s)
    return _wrap_angle(unwrapped_s)


def build_real_world_ground_truth_model(
    gps_df: pd.DataFrame,
    *,
    smooth_window_s: float,
) -> Dict[str, object]:
    """Build ego world trajectory, GPS-movement heading, and uncertainty model.

    World frame:
      - origin at the first RTK-GPS sample
      - +x aligned with overall direction of travel for the run
      - +y left of travel direction

    Heading is estimated from RTK-GPS movement, primarily from `vel_e_mps` /
    `vel_n_mps` when available, otherwise from differentiated GPS position.
    """
    required = ["header_t", "lat_deg", "lon_deg"]
    for col in required:
        if col not in gps_df.columns:
            raise ValueError(f"gps_truth CSV must include '{col}'")

    t = pd.to_numeric(gps_df["header_t"], errors="coerce").to_numpy(dtype=float)
    lat = pd.to_numeric(gps_df["lat_deg"], errors="coerce").to_numpy(dtype=float)
    lon = pd.to_numeric(gps_df["lon_deg"], errors="coerce").to_numpy(dtype=float)
    h_acc = (
        pd.to_numeric(gps_df["h_acc_m"], errors="coerce").to_numpy(dtype=float)
        if "h_acc_m" in gps_df.columns
        else np.full_like(t, np.nan)
    )
    vel_n = (
        pd.to_numeric(gps_df["vel_n_mps"], errors="coerce").to_numpy(dtype=float)
        if "vel_n_mps" in gps_df.columns
        else np.full_like(t, np.nan)
    )
    vel_e = (
        pd.to_numeric(gps_df["vel_e_mps"], errors="coerce").to_numpy(dtype=float)
        if "vel_e_mps" in gps_df.columns
        else np.full_like(t, np.nan)
    )

    mask = np.isfinite(t) & np.isfinite(lat) & np.isfinite(lon)
    t = t[mask]
    lat = lat[mask]
    lon = lon[mask]
    h_acc = h_acc[mask]
    vel_n = vel_n[mask]
    vel_e = vel_e[mask]
    if t.size < 2:
        raise ValueError("gps_truth has too few valid RTK-GPS samples")

    order = np.argsort(t)
    t = t[order]
    lat = lat[order]
    lon = lon[order]
    h_acc = h_acc[order]
    vel_n = vel_n[order]
    vel_e = vel_e[order]

    east_m, north_m = _gps_local_xy_m(
        lat,
        lon,
        lat0_deg=float(lat[0]),
        lon0_deg=float(lon[0]),
    )

    speed_from_vel = np.hypot(vel_e, vel_n)
    valid_vel = np.isfinite(vel_e) & np.isfinite(vel_n) & (speed_from_vel >= GPS_HEADING_SPEED_MIN_MPS)
    if np.any(valid_vel):
        dir_e = float(np.nanmedian(vel_e[valid_vel]))
        dir_n = float(np.nanmedian(vel_n[valid_vel]))
    else:
        dir_e = float(east_m[-1] - east_m[0])
        dir_n = float(north_m[-1] - north_m[0])

    norm = math.hypot(dir_e, dir_n)
    if norm < 1e-6:
        ux, uy = 1.0, 0.0
    else:
        ux, uy = dir_e / norm, dir_n / norm

    x_world_raw = east_m * ux + north_m * uy
    y_world_raw = -east_m * uy + north_m * ux

    # Heavy smoothing for RTK wobble suppression in GT trajectory.
    x_world = _smooth_series_time_window(t, x_world_raw, smooth_window_s)
    y_world = _smooth_series_time_window(t, y_world_raw, smooth_window_s)

    if np.any(valid_vel):
        vx_world = vel_e * ux + vel_n * uy
        vy_world = -vel_e * uy + vel_n * ux
    else:
        vx_world = np.gradient(x_world, t)
        vy_world = np.gradient(y_world, t)

    speed_world = np.hypot(vx_world, vy_world)
    heading_valid = np.isfinite(vx_world) & np.isfinite(vy_world) & (speed_world >= GPS_HEADING_SPEED_MIN_MPS)

    if np.any(heading_valid):
        heading_samples = np.arctan2(vy_world[heading_valid], vx_world[heading_valid])
        heading_interp = np.interp(t, t[heading_valid], np.unwrap(heading_samples))
    else:
        heading_interp = np.zeros_like(t)

    heading_interp = _smooth_angle_time_window(t, heading_interp, smooth_window_s)

    default_h_acc = float(np.nanmedian(h_acc[np.isfinite(h_acc)])) if np.any(np.isfinite(h_acc)) else 0.02
    sigma_pos_m = np.where(np.isfinite(h_acc), h_acc, default_h_acc)
    baseline_m = np.maximum(speed_world * GPS_HEADING_BASELINE_S, GPS_HEADING_BASELINE_MIN_M)
    sigma_heading_rad = np.sqrt(2.0) * sigma_pos_m / baseline_m
    sigma_heading_rad = np.clip(sigma_heading_rad, 0.0, math.radians(45.0))

    return {
        "t": t,
        "x_world": x_world,
        "y_world": y_world,
        "heading_world_rad": heading_interp,
        "sigma_pos_m": sigma_pos_m,
        "sigma_heading_rad": sigma_heading_rad,
        "x_world_fn": _interp_series(t, x_world),
        "y_world_fn": _interp_series(t, y_world),
        "heading_world_fn": _interp_angle_series(t, heading_interp),
        "sigma_pos_fn": _interp_series(t, sigma_pos_m),
        "sigma_heading_fn": _interp_series(t, sigma_heading_rad),
        "world_axis_yaw_rad": math.atan2(uy, ux),
        "smooth_window_s": float(smooth_window_s),
    }


def compute_ground_truth_body_frame(
    tq: np.ndarray,
    *,
    gt_model: Dict[str, object],
    target_x_world: float,
    target_y_world: float,
    target_sigma_m: float = TARGET_WORLD_SIGMA_M,
) -> Dict[str, np.ndarray]:
    """Compute GT in ego frame from GPS world trajectory and movement heading."""
    tq = np.asarray(tq, dtype=float)

    x_ego_world = gt_model["x_world_fn"](tq)
    y_ego_world = gt_model["y_world_fn"](tq)
    yaw_ego_world = gt_model["heading_world_fn"](tq)
    sigma_pos_m = gt_model["sigma_pos_fn"](tq)
    sigma_heading_rad = gt_model["sigma_heading_fn"](tq)

    dx_world = float(target_x_world) - x_ego_world
    dy_world = float(target_y_world) - y_ego_world

    c = np.cos(yaw_ego_world)
    s = np.sin(yaw_ego_world)
    gt_x_body = c * dx_world + s * dy_world
    gt_y_body = -s * dx_world + c * dy_world

    sigma_rel_sq = np.square(sigma_pos_m) + float(target_sigma_m) ** 2
    sigma_gt_x_m = np.sqrt(sigma_rel_sq + np.square(gt_y_body) * np.square(sigma_heading_rad))
    sigma_gt_y_m = np.sqrt(sigma_rel_sq + np.square(gt_x_body) * np.square(sigma_heading_rad))
    sigma_gt_pos_m = np.sqrt(2.0 * sigma_rel_sq + (np.square(gt_x_body) + np.square(gt_y_body)) * np.square(sigma_heading_rad))

    return {
        "ego_x_world": x_ego_world,
        "ego_y_world": y_ego_world,
        "ego_heading_world_rad": yaw_ego_world,
        "gt_x_world": np.full_like(tq, float(target_x_world), dtype=float),
        "gt_y_world": np.full_like(tq, float(target_y_world), dtype=float),
        "gt_x_body": gt_x_body,
        "gt_y_body": gt_y_body,
        "gt_sigma_x_m": sigma_gt_x_m,
        "gt_sigma_y_m": sigma_gt_y_m,
        "gt_sigma_pos_m": sigma_gt_pos_m,
        "gt_heading_sigma_rad": sigma_heading_rad,
    }


def gps_along_track_x(gps_df: pd.DataFrame) -> Tuple[np.ndarray, np.ndarray]:
    """Return (t_gps, x_gps_along_track_m) with origin at first GPS sample."""
    cols = gps_df.columns
    if "lat_deg" not in cols or "lon_deg" not in cols:
        raise ValueError("gps_truth CSV must include 'lat_deg' and 'lon_deg'")

    t = pd.to_numeric(gps_df["header_t"], errors="coerce").to_numpy(dtype=float)
    lat = pd.to_numeric(gps_df["lat_deg"], errors="coerce").to_numpy(dtype=float)
    lon = pd.to_numeric(gps_df["lon_deg"], errors="coerce").to_numpy(dtype=float)
    mask = np.isfinite(t) & np.isfinite(lat) & np.isfinite(lon)
    t = t[mask]
    lat = lat[mask]
    lon = lon[mask]
    if t.size < 2:
        return t, np.zeros_like(t)

    order = np.argsort(t)
    t = t[order]
    lat = lat[order]
    lon = lon[order]

    east, north = _gps_local_xy_m(lat, lon, lat0_deg=float(lat[0]), lon0_deg=float(lon[0]))

    # Define world +x as the GPS displacement direction.
    dx = float(east[-1] - east[0])
    dy = float(north[-1] - north[0])
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        ux, uy = 1.0, 0.0
    else:
        ux, uy = dx / norm, dy / norm

    x_along = east * ux + north * uy
    return t, x_along


def estimate_origin_shift_m(
    *,
    t_gps: np.ndarray,
    x_gps: np.ndarray,
    x_ego_speed_fn: Callable[[np.ndarray], np.ndarray],
    min_time_after_origin_s: float = 2.0,
) -> float:
    """Estimate a constant x-offset (meters) between GPS and speed integration."""
    if t_gps.size == 0:
        return 0.0
    t0 = float(t_gps[0])
    valid = np.isfinite(t_gps) & np.isfinite(x_gps) & (t_gps >= t0 + float(min_time_after_origin_s))
    if not np.any(valid):
        return 0.0
    diff = x_gps[valid] - x_ego_speed_fn(t_gps[valid])
    diff = diff[np.isfinite(diff)]
    if diff.size == 0:
        return 0.0
    return float(np.median(diff))


def _rename_time_columns(df: pd.DataFrame) -> pd.DataFrame:
    rename_map = {}
    if "header_stamp" in df.columns:
        rename_map["header_stamp"] = "header_t"
    if "logger_stamp" in df.columns:
        rename_map["logger_stamp"] = "logger_t"
    return df.rename(columns=rename_map)


def load_gps_truth(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    df = _rename_time_columns(df)
    if "header_t" not in df.columns:
        # gps_truth files commonly have only logger_stamp (renamed to logger_t); use it as timebase.
        if "logger_t" in df.columns:
            df = df.rename(columns={"logger_t": "header_t"})
        else:
            raise ValueError(
                "gps_truth CSV must have 'header_stamp', 'header_t', or 'logger_stamp'"
            )

    # We only need time here to define the world origin time.
    df = df.sort_values("header_t")
    return df


def load_ego_motion(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    df = _rename_time_columns(df)
    if "header_t" not in df.columns:
        raise ValueError("ego_motion CSV must have 'header_stamp' or 'header_t'")

    # Speed column naming varies; support the common ones.
    speed_candidates = ["v_mps", "speed_mps", "v", "speed"]
    speed_col = next((c for c in speed_candidates if c in df.columns), None)
    if speed_col is None:
        raise ValueError(
            "ego_motion CSV must have a speed column (one of: "
            + ", ".join(speed_candidates)
            + ")"
        )

    df = df.sort_values("header_t")
    df = df[["header_t", speed_col]].rename(columns={speed_col: "v_mps"}).copy()
    df["v_mps"] = pd.to_numeric(df["v_mps"], errors="coerce")
    df["header_t"] = pd.to_numeric(df["header_t"], errors="coerce")
    df = df.dropna(subset=["header_t", "v_mps"]).copy()
    return df


def build_x_ego_from_speed(
    ego_motion_df: pd.DataFrame,
    *,
    origin_time: float,
) -> Callable[[np.ndarray], np.ndarray]:
    """Return x_ego_world(t) based on integrating speed along world +x.

    x_ego_world is anchored so that x_ego_world(origin_time) = 0.
    """

    t = ego_motion_df["header_t"].to_numpy(dtype=float)
    v = ego_motion_df["v_mps"].to_numpy(dtype=float)
    if t.size < 2:
        raise ValueError("ego_motion has too few samples to integrate")

    # Ensure strictly increasing for interpolation/integration.
    order = np.argsort(t)
    t = t[order]
    v = v[order]

    # Remove duplicate timestamps (keep last).
    _, unique_idx = np.unique(t, return_index=True)
    unique_idx = np.sort(unique_idx)
    t = t[unique_idx]
    v = v[unique_idx]

    dt = np.diff(t)
    dt = np.maximum(dt, 0.0)
    # Trapezoidal integration.
    dx = 0.5 * (v[:-1] + v[1:]) * dt
    x_cum = np.concatenate([[0.0], np.cumsum(dx)])

    # Anchor at origin_time (world origin is GPS start).
    x_at_origin = float(np.interp(origin_time, t, x_cum))
    x_cum = x_cum - x_at_origin

    def x_ego(tq: np.ndarray) -> np.ndarray:
        tq = np.asarray(tq, dtype=float)
        return np.interp(tq, t, x_cum)

    return x_ego


def load_raw_detections(
    path: str,
    *,
    radar_tf_base_to_sensor: np.ndarray,
    camera_tf_base_to_sensor: np.ndarray,
) -> Tuple[pd.DataFrame, pd.DataFrame]:
    """Load raw detections and split into radar/camera in BODY (base_link) frame.

    Raw file contains both sensor types and logs positions in each sensor's frame.
    We:
      - Convert camera optical axes -> camera_link axes if needed
      - Apply base_link -> sensor_link TF (rotation+translation) to map
        p_base = R * p_sensor + t
    """
    df = pd.read_csv(path)
    df = _rename_time_columns(df)
    required = ["sensor_type", "header_t", "x", "y"]
    for col in required:
        if col not in df.columns:
            raise ValueError(f"raw detections CSV must have column '{col}'")

    df_radar = df[df["sensor_type"] == "radar"].copy()
    df_camera = df[df["sensor_type"] == "camera"].copy()

    # TF: base_link -> sensor_link
    radar_tf = np.asarray(radar_tf_base_to_sensor, dtype=float).reshape(6)
    camera_tf = np.asarray(camera_tf_base_to_sensor, dtype=float).reshape(6)
    t_br = radar_tf[:3]
    rpy_br = radar_tf[3:]
    R_br = _rpy_to_rot_matrix(rpy_br[0], rpy_br[1], rpy_br[2])

    t_bc = camera_tf[:3]
    rpy_bc = camera_tf[3:]
    R_bc = _rpy_to_rot_matrix(rpy_bc[0], rpy_bc[1], rpy_bc[2])

    if not df_radar.empty:
        x_s = pd.to_numeric(df_radar["x"], errors="coerce").to_numpy(dtype=float)
        y_s = pd.to_numeric(df_radar["y"], errors="coerce").to_numpy(dtype=float)
        z_s = (
            pd.to_numeric(df_radar["z"], errors="coerce").to_numpy(dtype=float)
            if "z" in df_radar.columns
            else np.zeros_like(x_s)
        )
        pts_s = np.stack([x_s, y_s, z_s], axis=1)
        pts_b = (R_br @ pts_s.T).T + t_br.reshape(1, 3)
        df_radar["x"] = pts_b[:, 0]
        df_radar["y"] = pts_b[:, 1]
        if "z" in df_radar.columns:
            df_radar["z"] = pts_b[:, 2]
        df_radar["header_t"] = pd.to_numeric(df_radar["header_t"], errors="coerce")
        df_radar = df_radar.dropna(subset=["header_t", "x", "y"]).copy()

    if not df_camera.empty:
        if "z" not in df_camera.columns:
            raise ValueError("camera rows require 'z' column in raw detections")

        old_x = pd.to_numeric(df_camera["x"], errors="coerce").to_numpy(dtype=float)
        old_y = pd.to_numeric(df_camera["y"], errors="coerce").to_numpy(dtype=float)
        old_z = pd.to_numeric(df_camera["z"], errors="coerce").to_numpy(dtype=float)

        # Real-world logger logs /camera_detections which are published in camera_link
        # (x forward, y left, z up) by camera_interface/camera_node.py.
        # Older logs may contain optical axes in the raw CSV:
        #   optical: x right, y down, z forward.
        # Auto-detect by checking whether forward axis dominates in x (camera_link)
        # or in z (optical).
        med_abs_x = float(np.nanmedian(np.abs(old_x)))
        med_abs_z = float(np.nanmedian(np.abs(old_z)))
        looks_optical = bool(med_abs_z > 1.5 * med_abs_x)

        if looks_optical:
            # Optical -> camera_link axes
            x_c = old_z
            y_c = -old_x
            z_c = -old_y
        else:
            # Already camera_link axes
            x_c = old_x
            y_c = old_y
            z_c = old_z

        pts_c = np.stack([x_c, y_c, z_c], axis=1)
        pts_b = (R_bc @ pts_c.T).T + t_bc.reshape(1, 3)
        df_camera["x"] = pts_b[:, 0]
        df_camera["y"] = pts_b[:, 1]
        df_camera["z"] = pts_b[:, 2]
        df_camera["header_t"] = pd.to_numeric(df_camera["header_t"], errors="coerce")
        df_camera = df_camera.dropna(subset=["header_t", "x", "y"]).copy()

    return df_radar, df_camera


def filter_radar(df_radar: pd.DataFrame) -> pd.DataFrame:
    return filter_xy_box(df_radar)


def filter_xy_box(df: pd.DataFrame) -> pd.DataFrame:
    """Filter clutter in body frame to x:[0,35], y:[-4,4]."""
    if df.empty:
        return df
    if "x" not in df.columns or "y" not in df.columns:
        return df
    x = pd.to_numeric(df["x"], errors="coerce").to_numpy(dtype=float)
    y = pd.to_numeric(df["y"], errors="coerce").to_numpy(dtype=float)
    mask = np.isfinite(x) & np.isfinite(y) & (x >= 0.0) & (x <= 35.0) & (y >= -4.0) & (y <= 4.0)
    return df.loc[mask].copy()


def add_gt_and_error(
    df: pd.DataFrame,
    *,
    gt_model: Dict[str, object],
    target_x_world: float,
    target_y_world: float = TARGET_Y_WORLD,
) -> pd.DataFrame:
    if df.empty:
        out = df.copy()
        out["gt_x_body"] = np.array([], dtype=float)
        out["gt_y_body"] = np.array([], dtype=float)
        out["err_m"] = np.array([], dtype=float)
        return out

    if "header_t" not in df.columns:
        raise ValueError("dataset missing 'header_t' column")
    if "x" not in df.columns or "y" not in df.columns:
        raise ValueError("dataset missing 'x'/'y' columns")

    out = df.copy()
    t = pd.to_numeric(out["header_t"], errors="coerce").to_numpy(dtype=float)
    x_det = pd.to_numeric(out["x"], errors="coerce").to_numpy(dtype=float)
    y_det = pd.to_numeric(out["y"], errors="coerce").to_numpy(dtype=float)

    gt = compute_ground_truth_body_frame(
        t,
        gt_model=gt_model,
        target_x_world=target_x_world,
        target_y_world=target_y_world,
    )
    gt_x = gt["gt_x_body"]
    gt_y = gt["gt_y_body"]

    dx = x_det - gt_x
    dy = y_det - gt_y
    err = np.sqrt(dx * dx + dy * dy)

    out["ego_x_world"] = gt["ego_x_world"]
    out["ego_y_world"] = gt["ego_y_world"]
    out["ego_heading_world_rad"] = gt["ego_heading_world_rad"]
    out["gt_x_world"] = gt["gt_x_world"]
    out["gt_y_world"] = gt["gt_y_world"]
    out["gt_x_body"] = gt_x
    out["gt_y_body"] = gt_y
    out["gt_sigma_x_m"] = gt["gt_sigma_x_m"]
    out["gt_sigma_y_m"] = gt["gt_sigma_y_m"]
    out["gt_sigma_pos_m"] = gt["gt_sigma_pos_m"]
    out["gt_heading_sigma_rad"] = gt["gt_heading_sigma_rad"]
    out["err_m"] = err
    return out


def summarize_gt_uncertainty(*dfs: pd.DataFrame) -> Dict[str, float]:
    vals = []
    for df in dfs:
        if df is None or df.empty or "gt_sigma_pos_m" not in df.columns:
            continue
        arr = pd.to_numeric(df["gt_sigma_pos_m"], errors="coerce").to_numpy(dtype=float)
        arr = arr[np.isfinite(arr)]
        if arr.size:
            vals.append(arr)
    if not vals:
        return {"mean": float("nan"), "median": float("nan"), "max": float("nan")}
    all_vals = np.concatenate(vals)
    return {
        "mean": float(np.mean(all_vals)),
        "median": float(np.median(all_vals)),
        "max": float(np.max(all_vals)),
    }


def plot_real_world_xy(
    radar_df: pd.DataFrame,
    camera_df: pd.DataFrame,
    fused_df: pd.DataFrame,
    tracks_df: pd.DataFrame,
    out_prefix: str,
) -> None:
    """Make dedicated real-world XY plots in ego/body frame."""

    out_dir = str(Path(out_prefix).parent)
    if out_dir and out_dir != ".":
        Path(out_dir).mkdir(parents=True, exist_ok=True)

    def _plot_gt_trajectory(*dfs: pd.DataFrame):
        for df in dfs:
            if df is None or df.empty:
                continue
            if "gt_x_body" not in df.columns or "gt_y_body" not in df.columns:
                continue
            work = df.copy()
            work["gt_x_body"] = pd.to_numeric(work["gt_x_body"], errors="coerce")
            work["gt_y_body"] = pd.to_numeric(work["gt_y_body"], errors="coerce")
            work = work.dropna(subset=["gt_x_body", "gt_y_body"])
            if work.empty:
                continue
            if "header_t" in work.columns:
                work["header_t"] = pd.to_numeric(work["header_t"], errors="coerce")
                work = work.sort_values("header_t")
            plt.plot(
                work["gt_y_body"],
                work["gt_x_body"],
                color="black",
                linewidth=1.0,
                alpha=0.35,
                label="GT trajectory",
            )
            return

    def _plot_tracks_grouped(tracks: pd.DataFrame):
        if tracks.empty or "tracking_id" not in tracks.columns:
            return
        unique_ids = tracks["tracking_id"].dropna().astype(object).unique().tolist()
        unique_ids = sorted(unique_ids, key=lambda v: str(v))
        cmap = plt.get_cmap("tab20")
        for idx, track_id in enumerate(unique_ids):
            tr = tracks.loc[tracks["tracking_id"] == track_id, ["x", "y"]].dropna()
            if tr.empty:
                continue
            plt.scatter(
                tr["y"],
                tr["x"],
                marker=".",
                color=cmap(idx % cmap.N),
                s=2,
                alpha=0.75,
                label=(f"Track {track_id}" if idx < 12 else None),
            )

    def _legend_if_any():
        handles, labels = plt.gca().get_legend_handles_labels()
        if handles:
            plt.legend()

    # 1) Raw + fused + GT
    plt.figure(figsize=(8, 12))
    _plot_gt_trajectory(fused_df, tracks_df, radar_df, camera_df)
    if not fused_df.empty:
        plt.scatter(fused_df["y"], fused_df["x"], marker=".", color="r", s=2, alpha=0.7, label="Fused")
    if not radar_df.empty:
        plt.scatter(radar_df["y"], radar_df["x"], marker=".", color="g", s=2, alpha=0.7, label="Radar detections")
    if not camera_df.empty:
        plt.scatter(camera_df["y"], camera_df["x"], marker=".", color="y", s=2, alpha=0.7, label="Camera detections")
    plt.xlabel("y [m]")
    plt.ylabel("x [m]")
    plt.title("Real-world: Raw & Fused vs GT")
    _legend_if_any()
    plt.ylim(-2, 35)
    plt.tight_layout()
    plt.savefig(f"{out_prefix}_raw_fused_vs_gt.png", dpi=300)
    plt.close()

    # 2) GT + fused + tracks
    plt.figure(figsize=(8, 12))
    _plot_gt_trajectory(fused_df, tracks_df)
    if not fused_df.empty:
        plt.scatter(fused_df["y"], fused_df["x"], marker=".", color="r", s=2, alpha=0.7, label="Fused")
    _plot_tracks_grouped(tracks_df)
    plt.xlabel("y [m]")
    plt.ylabel("x [m]")
    plt.title("Real-world: Fused & Tracks vs GT")
    _legend_if_any()
    plt.ylim(-2, 35)
    plt.tight_layout()
    plt.savefig(f"{out_prefix}_fused_tracks_vs_gt.png", dpi=300)
    plt.close()

    # 3) GT + tracks
    plt.figure(figsize=(8, 12))
    _plot_gt_trajectory(tracks_df, fused_df)
    _plot_tracks_grouped(tracks_df)
    plt.xlabel("y [m]")
    plt.ylabel("x [m]")
    plt.title("Real-world: Tracks vs GT")
    _legend_if_any()
    plt.ylim(-2, 35)
    plt.tight_layout()
    plt.savefig(f"{out_prefix}_tracks_vs_gt.png", dpi=300)
    plt.close()


def rmse_from_df(df_with_gt: pd.DataFrame) -> float:
    if df_with_gt.empty or "err_m" not in df_with_gt.columns:
        return float("nan")
    err = pd.to_numeric(df_with_gt["err_m"], errors="coerce").to_numpy(dtype=float)
    err = err[np.isfinite(err)]
    if err.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(err * err)))


def filter_by_gt_distance(df_with_gt: pd.DataFrame, max_dist_m: float) -> pd.DataFrame:
    """Keep only rows whose detection/track is within `max_dist_m` of GT.

    This acts as a single-target association gate for cluttered real-world logs.
    Set `max_dist_m <= 0` to disable.
    """
    if df_with_gt.empty or "err_m" not in df_with_gt.columns:
        return df_with_gt
    max_dist_m = float(max_dist_m)
    if max_dist_m <= 0.0:
        return df_with_gt
    err = pd.to_numeric(df_with_gt["err_m"], errors="coerce").to_numpy(dtype=float)
    mask = np.isfinite(err) & (err <= max_dist_m)
    return df_with_gt.loc[mask].copy()


def main() -> None:
    parser = argparse.ArgumentParser(add_help=True)
    parser.add_argument("test_name", help="Test name, e.g. t1_1")
    parser.add_argument(
        "--log-dir",
        default="real_world_logs",
        help="Directory containing the CSV logs (default: real_world_logs)",
    )
    parser.add_argument(
        "--out-prefix",
        default=None,
        help="Output prefix for plots/metrics (default: plots/<test_name>)",
    )
    parser.add_argument(
        "--tf-file",
        default=None,
        help=(
            "Calibration TF file to apply to raw radar/camera detections. "
            "If omitted, auto-selected based on test name: "
            "'*_replay' -> calibration_log/replay_tf.txt, "
            "otherwise -> calibration_log/calibration_tf.txt."
        ),
    )
    parser.add_argument(
        "--gt-gate-m",
        type=float,
        default=3.0,
        help=(
            "Keep only detections/tracks within this distance (meters) from the "
            "instantaneous GT before RMSE. Use <=0 to disable. Default: 3.0"
        ),
    )
    parser.add_argument(
        "--gt-smooth-window-s",
        type=float,
        default=2.5,
        help=(
            "Time window (seconds) for strong moving-average smoothing of RTK-GPS "
            "trajectory and movement-derived heading used in GT. Set <=0 to disable. "
            "Default: 2.5"
        ),
    )
    args = parser.parse_args()

    test_name = str(args.test_name)
    log_dir = Path(str(args.log_dir))
    out_prefix = str(args.out_prefix) if args.out_prefix else f"plots/{test_name}"
    target_x_world, target_y_world = resolve_target_world_from_test_name(test_name)

    if re.match(r"^t[1-5](?:_|$)", test_name.strip().lower()) is None:
        print(
            f"[WARN] Could not infer scenario (t1..t5) from test_name '{test_name}'. "
            f"Using default target y={TARGET_Y_WORLD:.2f} m."
        )

    if args.tf_file is not None:
        tf_path = Path(str(args.tf_file))
    elif test_name.endswith("_replay"):
        tf_path = Path("calibration_log/replay_tf.txt")
    else:
        tf_path = Path("calibration_log/calibration_tf.txt")
    tf = _parse_calibration_tf_file(tf_path)
    radar_tf = tf.get("radar_tf", np.zeros(6, dtype=float))
    camera_tf = tf.get("camera_tf", np.zeros(6, dtype=float))
    if not tf_path.exists():
        print(f"[WARN] TF file not found: {tf_path} (raw RMSE will be uncalibrated)")
    else:
        print(f"  TF file: {tf_path}")
        if "radar_tf" not in tf:
            print(f"[WARN] TF file missing radar_tf: {tf_path} (radar RMSE uncalibrated)")
        if "camera_tf" not in tf:
            print(f"[WARN] TF file missing camera_tf: {tf_path} (camera RMSE uncalibrated)")

    def _resolve_log_path(stem: str) -> Path:
        """Prefer raw real_world_logger filenames; fall back to legacy *_approach.csv."""
        p = log_dir / f"{stem}_{test_name}.csv"
        if p.exists():
            return p
        legacy = log_dir / f"{stem}_{test_name}_approach.csv"
        if legacy.exists():
            print(f"[WARN] Using legacy approach-phase file: {legacy}")
            return legacy
        return p

    raw_path = _resolve_log_path("raw_detections")
    fused_path = _resolve_log_path("fused_detections")
    tracks_path = _resolve_log_path("tracks")
    gps_path = _resolve_log_path("gps_truth")
    ego_motion_path = _resolve_log_path("ego_motion")

    missing = [p for p in [raw_path, fused_path, tracks_path, gps_path, ego_motion_path] if not p.exists()]
    if missing:
        raise FileNotFoundError("Missing required log files:\n  - " + "\n  - ".join(str(p) for p in missing))

    print(f"Analyzing real-world test '{test_name}'")
    print(f"  Log directory: {log_dir}")

    print("\nLoading data files...")
    gps_df = load_gps_truth(str(gps_path))
    ego_motion_df = load_ego_motion(str(ego_motion_path))
    radar_df, camera_df = load_raw_detections(
        str(raw_path),
        radar_tf_base_to_sensor=radar_tf,
        camera_tf_base_to_sensor=camera_tf,
    )
    fused_df = load_fused(str(fused_path))
    tracks_df = load_tracks(str(tracks_path))

    # Fused detections CSV includes both true-fused and pass-through single-sensor detections.
    # For RMSE, keep only real fused pairs.
    if not fused_df.empty and "detection_type" in fused_df.columns:
        fused_df = fused_df[fused_df["detection_type"].astype(str) == "fused"].copy()
    elif not fused_df.empty:
        print("[WARN] fused_detections CSV missing 'detection_type' column; RMSE will include all rows")

    gt_model = build_real_world_ground_truth_model(
        gps_df,
        smooth_window_s=float(args.gt_smooth_window_s),
    )
    origin_time = float(gt_model["t"][0])
    gps_displacement_m = float(
        np.hypot(
            gt_model["x_world"][-1] - gt_model["x_world"][0],
            gt_model["y_world"][-1] - gt_model["y_world"][0],
        )
    )
    print(f"\nWorld origin time (first GPS sample): {origin_time:.3f}")
    print(
        f"Target ground truth (world frame): static at ({target_x_world:.2f}, {target_y_world:.2f}) m"
    )
    print(
        "GT model: RTK-GPS ego world trajectory + heading estimated from GPS movement"
    )
    print(f"GT smoothing window: {float(args.gt_smooth_window_s):.2f} s")
    print(f"GPS world displacement: {gps_displacement_m:.2f} m")

    # Filter radar clutter (camera not filtered per spec).
    radar_before = len(radar_df)
    radar_df = filter_radar(radar_df)
    print(
        f"\nFiltering radar detections: x in [0,35], y in [-6,6] -> kept {len(radar_df)}/{radar_before}"
    )

    fused_before = len(fused_df)
    fused_df = filter_xy_box(fused_df)
    print(
        f"Filtering fused detections: x in [0,35], y in [-6,6] -> kept {len(fused_df)}/{fused_before}"
    )

    tracks_before = len(tracks_df)
    tracks_df = filter_xy_box(tracks_df)
    print(
        f"Filtering tracks: x in [0,35], y in [-6,6] -> kept {len(tracks_df)}/{tracks_before}"
    )

    # Compute GT at each dataset's timestamps and per-row errors.
    radar_w = add_gt_and_error(radar_df, gt_model=gt_model, target_x_world=target_x_world, target_y_world=target_y_world)
    camera_w = add_gt_and_error(camera_df, gt_model=gt_model, target_x_world=target_x_world, target_y_world=target_y_world)
    fused_w = add_gt_and_error(fused_df, gt_model=gt_model, target_x_world=target_x_world, target_y_world=target_y_world)
    tracks_w = add_gt_and_error(tracks_df, gt_model=gt_model, target_x_world=target_x_world, target_y_world=target_y_world)

    if float(args.gt_gate_m) > 0.0:
        radar_before_gate = len(radar_w)
        camera_before_gate = len(camera_w)
        fused_before_gate = len(fused_w)
        tracks_before_gate = len(tracks_w)

        radar_w = filter_by_gt_distance(radar_w, float(args.gt_gate_m))
        camera_w = filter_by_gt_distance(camera_w, float(args.gt_gate_m))
        fused_w = filter_by_gt_distance(fused_w, float(args.gt_gate_m))
        tracks_w = filter_by_gt_distance(tracks_w, float(args.gt_gate_m))

        print(
            f"GT-distance gate ({float(args.gt_gate_m):.2f} m): "
            f"radar {len(radar_w)}/{radar_before_gate}, "
            f"camera {len(camera_w)}/{camera_before_gate}, "
            f"fused {len(fused_w)}/{fused_before_gate}, "
            f"tracks {len(tracks_w)}/{tracks_before_gate}"
        )

    rmse_radar = rmse_from_df(radar_w)
    rmse_camera = rmse_from_df(camera_w)
    rmse_fused = rmse_from_df(fused_w)
    rmse_tracks = rmse_from_df(tracks_w)
    gt_unc = summarize_gt_uncertainty(radar_w, camera_w, fused_w, tracks_w)

    def _fmt(name: str, rmse: float, n: int) -> str:
        if math.isnan(rmse):
            return f"  {name:<18} RMSE NaN (n={n})"
        return f"  {name:<18} RMSE {rmse:.3f} m (n={n})"

    print(f"\n{'='*70}")
    print(f"RMSE Summary for test {test_name} (RTK-GPS world GT + movement heading):")
    print(f"{'='*70}")
    print(_fmt("Radar detections:", rmse_radar, len(radar_w)))
    print(_fmt("Camera detections:", rmse_camera, len(camera_w)))
    print(_fmt("Fused detections:", rmse_fused, len(fused_w)))
    print(_fmt("Tracks:", rmse_tracks, len(tracks_w)))
    if not math.isnan(gt_unc["mean"]):
        print(
            f"  GT uncertainty (1σ): mean {gt_unc['mean']:.3f} m, "
            f"median {gt_unc['median']:.3f} m, max {gt_unc['max']:.3f} m"
        )
    print(f"{'='*70}")

    # Save metrics
    metrics_path = Path(out_prefix + "_metrics.txt")
    metrics_path.parent.mkdir(parents=True, exist_ok=True)
    with open(metrics_path, "w", encoding="utf-8") as f:
        f.write(f"Real-World Test Analysis: {test_name}\n")
        f.write("Ground truth: RTK-GPS ego world trajectory + heading from GPS movement\n")
        f.write(f"World origin time (first GPS sample): {origin_time:.3f}\n")
        f.write(f"Static target world position: ({target_x_world:.2f}, {target_y_world:.2f}) m\n")
        f.write(f"GPS world displacement: {gps_displacement_m:.2f} m\n")
        f.write(f"GT smoothing window: {float(args.gt_smooth_window_s):.2f} s\n")
        f.write("Filter (radar/fused/tracks): x in [0,35], y in [-6,6]\n")
        f.write("Camera: no x/y filter applied\n\n")
        if float(args.gt_gate_m) > 0.0:
            f.write(f"GT-distance gate: <= {float(args.gt_gate_m):.2f} m\n\n")
        else:
            f.write("GT-distance gate: disabled\n\n")
        f.write(_fmt("Radar", rmse_radar, len(radar_w)) + "\n")
        f.write(_fmt("Camera", rmse_camera, len(camera_w)) + "\n")
        f.write(_fmt("Fused", rmse_fused, len(fused_w)) + "\n")
        f.write(_fmt("Tracks", rmse_tracks, len(tracks_w)) + "\n")
        if not math.isnan(gt_unc["mean"]):
            f.write(
                f"GT uncertainty (1σ radial): mean {gt_unc['mean']:.3f} m, "
                f"median {gt_unc['median']:.3f} m, max {gt_unc['max']:.3f} m\n"
            )

    print(f"\nMetrics saved to: {metrics_path}")

    # Plots (same filenames/pattern as before)
    print("\nGenerating plots...")
    try:
        plot_real_world_xy(radar_w, camera_w, fused_w, tracks_w, out_prefix=out_prefix)
        print(f"Plots saved with prefix: {out_prefix}")
    except Exception as exc:
        print(f"Warning: Plot generation failed: {exc}")

    print("\nAnalysis complete!")


if __name__ == "__main__":
    main()
