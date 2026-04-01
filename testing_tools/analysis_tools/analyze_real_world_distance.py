#!/usr/bin/env python3
"""Distance-only real-world analysis for single-target field tests.

This variant keeps the existing `analyze_real_world.py` untouched and evaluates only
range / distance-to-target error. Ground truth is built from RTK-GPS ego position in
an aligned world frame and a fixed target world position per scenario:

- t1 -> target at (29.0, 0.0) m
- t2 -> target at (29.0, -1.5) m
- t3 -> target at (29.0, 1.5) m
- t4 -> target at (29.0, -3.0) m
- t5 -> target at (29.0, 3.0) m

Per-row distance error is:
    | ||measurement_body|| - ||target_world - ego_world|| |

This avoids heading-induced GT jumps in the primary metric while preserving the same
raw/fused/tracks log handling and plot generation workflow.
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path
from typing import Dict, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


TARGET_X_WORLD: float = 29.0
TARGET_Y_BY_SCENARIO: Dict[str, float] = {
    "t1": 0.0,
    "t2": -1.5,
    "t3": 1.5,
    "t4": -3.0,
    "t5": 3.0,
}


def _read_csv_robust(path: str | Path) -> pd.DataFrame:
    """Read CSV while tolerating malformed replay rows."""
    try:
        return pd.read_csv(path)
    except pd.errors.ParserError:
        return pd.read_csv(path, engine="python", on_bad_lines="skip")


def _rename_time_columns(df: pd.DataFrame) -> pd.DataFrame:
    rename_map = {}
    if "header_stamp" in df.columns:
        rename_map["header_stamp"] = "header_t"
    if "logger_stamp" in df.columns:
        rename_map["logger_stamp"] = "logger_t"
    return df.rename(columns=rename_map)


def resolve_target_world_from_test_name(test_name: str) -> Tuple[float, float]:
    m = re.match(r"^(t[1-5])(?:_|$)", str(test_name).strip().lower())
    if not m:
        return TARGET_X_WORLD, 0.0
    scenario = m.group(1)
    return TARGET_X_WORLD, float(TARGET_Y_BY_SCENARIO[scenario])


def _parse_calibration_tf_file(tf_path: Path) -> Dict[str, np.ndarray]:
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
        elif line.startswith("camera_tf:"):
            parts = line.split(":", 1)[1].strip().split()
            if len(parts) >= 6:
                out["camera_tf"] = np.array([float(x) for x in parts[:6]], dtype=float)
    return out


def _rpy_to_rot_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr = math.cos(float(roll))
    sr = math.sin(float(roll))
    cp = math.cos(float(pitch))
    sp = math.sin(float(pitch))
    cy = math.cos(float(yaw))
    sy = math.sin(float(yaw))
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def _gps_local_xy_m(
    lat_deg: np.ndarray,
    lon_deg: np.ndarray,
    *,
    lat0_deg: float,
    lon0_deg: float,
) -> Tuple[np.ndarray, np.ndarray]:
    lat0_rad = math.radians(float(lat0_deg))
    m_per_deg_lat = 111_320.0
    m_per_deg_lon = 111_320.0 * math.cos(lat0_rad)
    east_m = (lon_deg - float(lon0_deg)) * m_per_deg_lon
    north_m = (lat_deg - float(lat0_deg)) * m_per_deg_lat
    return east_m, north_m


def _interp_series(t: np.ndarray, values: np.ndarray):
    t = np.asarray(t, dtype=float)
    values = np.asarray(values, dtype=float)

    def fn(tq: np.ndarray) -> np.ndarray:
        tq = np.asarray(tq, dtype=float)
        return np.interp(tq, t, values)

    return fn


def _smooth_series_time_window(t: np.ndarray, values: np.ndarray, window_s: float) -> np.ndarray:
    t = np.asarray(t, dtype=float)
    values = np.asarray(values, dtype=float)
    if values.size < 5 or float(window_s) <= 0.0:
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

    pad = win // 2
    kernel = np.ones(win, dtype=float) / float(win)
    padded = np.pad(values, (pad, pad), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def load_gps_truth(path: str) -> pd.DataFrame:
    df = _read_csv_robust(path)
    df = _rename_time_columns(df)
    if "header_t" not in df.columns:
        if "logger_t" in df.columns:
            df = df.rename(columns={"logger_t": "header_t"})
        else:
            raise ValueError("gps_truth CSV must have 'header_stamp', 'header_t', or 'logger_stamp'")

    required = ["header_t", "lat_deg", "lon_deg"]
    for col in required:
        if col not in df.columns:
            raise ValueError(f"gps_truth CSV must include '{col}'")

    df = df.sort_values("header_t").copy()
    return df


def load_fused(path: str) -> pd.DataFrame:
    df = _read_csv_robust(path)
    df = _rename_time_columns(df)
    required = ["header_t", "x", "y"]
    for col in required:
        if col not in df.columns:
            raise ValueError(f"fused detections CSV must have column '{col}'")
    return df


def load_tracks(path: str) -> pd.DataFrame:
    df = _read_csv_robust(path)
    df = _rename_time_columns(df)
    required = ["header_t", "tracking_id", "x", "y"]
    for col in required:
        if col not in df.columns:
            raise ValueError(f"tracks CSV must have column '{col}'")
    return df


def load_raw_detections(
    path: str,
    *,
    radar_tf_base_to_sensor: np.ndarray,
    camera_tf_base_to_sensor: np.ndarray,
) -> Tuple[pd.DataFrame, pd.DataFrame]:
    df = _read_csv_robust(path)
    df = _rename_time_columns(df)
    required = ["sensor_type", "header_t", "x", "y"]
    for col in required:
        if col not in df.columns:
            raise ValueError(f"raw detections CSV must have column '{col}'")

    df_radar = df[df["sensor_type"] == "radar"].copy()
    df_camera = df[df["sensor_type"] == "camera"].copy()

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
        df_radar["header_t"] = pd.to_numeric(df_radar["header_t"], errors="coerce")
        df_radar = df_radar.dropna(subset=["header_t", "x", "y"]).copy()

    if not df_camera.empty:
        if "z" not in df_camera.columns:
            raise ValueError("camera rows require 'z' column in raw detections")

        old_x = pd.to_numeric(df_camera["x"], errors="coerce").to_numpy(dtype=float)
        old_y = pd.to_numeric(df_camera["y"], errors="coerce").to_numpy(dtype=float)
        old_z = pd.to_numeric(df_camera["z"], errors="coerce").to_numpy(dtype=float)

        med_abs_x = float(np.nanmedian(np.abs(old_x)))
        med_abs_z = float(np.nanmedian(np.abs(old_z)))
        looks_optical = bool(med_abs_z > 1.5 * med_abs_x)

        if looks_optical:
            x_c = old_z
            y_c = -old_x
            z_c = -old_y
        else:
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


def filter_xy_box(df: pd.DataFrame) -> pd.DataFrame:
    if df.empty or "x" not in df.columns or "y" not in df.columns:
        return df
    x = pd.to_numeric(df["x"], errors="coerce").to_numpy(dtype=float)
    y = pd.to_numeric(df["y"], errors="coerce").to_numpy(dtype=float)
    mask = np.isfinite(x) & np.isfinite(y) & (x >= 0.0) & (x <= 35.0) & (y >= -6.0) & (y <= 6.0)
    return df.loc[mask].copy()


def _axis_direction_from_gps_df(gps_df: pd.DataFrame) -> Tuple[float, float]:
    """Compute (ux, uy) world-frame x-axis unit vector from start→end of a GPS log.

    The returned unit vector points in the direction of travel (first sample toward
    last sample in time).  Used to define a consistent world-frame axis across runs.
    """
    lat = pd.to_numeric(gps_df["lat_deg"], errors="coerce").to_numpy(dtype=float)
    lon = pd.to_numeric(gps_df["lon_deg"], errors="coerce").to_numpy(dtype=float)
    t_axis = pd.to_numeric(gps_df["header_t"], errors="coerce").to_numpy(dtype=float)
    mask_ax = np.isfinite(t_axis) & np.isfinite(lat) & np.isfinite(lon)
    t_axis, lat, lon = t_axis[mask_ax], lat[mask_ax], lon[mask_ax]
    if t_axis.size < 2:
        raise ValueError("axis GPS file has too few valid samples to determine direction")
    order = np.argsort(t_axis)
    lat, lon = lat[order], lon[order]
    east_ax, north_ax = _gps_local_xy_m(lat, lon, lat0_deg=float(lat[0]), lon0_deg=float(lon[0]))
    dir_e = float(east_ax[-1] - east_ax[0])
    dir_n = float(north_ax[-1] - north_ax[0])
    norm = math.hypot(dir_e, dir_n)
    if norm < 1e-6:
        return 1.0, 0.0
    return dir_e / norm, dir_n / norm


def build_distance_ground_truth_model(
    gps_df: pd.DataFrame,
    *,
    smooth_window_s: float,
    axis_gps_df: pd.DataFrame | None = None,
) -> Dict[str, object]:
    """Build the GT distance model for one test run.

    Parameters
    ----------
    gps_df:
        RTK-GPS log for the current test run (provides ego trajectory).
    smooth_window_s:
        Time window for GPS position smoothing.
    axis_gps_df:
        If provided, the world-frame x-axis direction is derived from
        *this* GPS log (start→end) instead of the current run's GPS.
        Use a t1 run log so all tests share the same axis.
    """
    t = pd.to_numeric(gps_df["header_t"], errors="coerce").to_numpy(dtype=float)
    lat = pd.to_numeric(gps_df["lat_deg"], errors="coerce").to_numpy(dtype=float)
    lon = pd.to_numeric(gps_df["lon_deg"], errors="coerce").to_numpy(dtype=float)

    mask = np.isfinite(t) & np.isfinite(lat) & np.isfinite(lon)
    t = t[mask]
    lat = lat[mask]
    lon = lon[mask]
    if t.size < 2:
        raise ValueError("gps_truth has too few valid RTK-GPS samples")

    order = np.argsort(t)
    t = t[order]
    lat = lat[order]
    lon = lon[order]

    east_m, north_m = _gps_local_xy_m(lat, lon, lat0_deg=float(lat[0]), lon0_deg=float(lon[0]))

    # Determine world-frame x-axis direction.
    if axis_gps_df is not None:
        # Use an external GPS log (e.g. t1) so all runs share the same frame.
        ux, uy = _axis_direction_from_gps_df(axis_gps_df)
    else:
        # Fall back: derive axis from this run's own start→end direction.
        dir_e = float(east_m[-1] - east_m[0])
        dir_n = float(north_m[-1] - north_m[0])
        norm = math.hypot(dir_e, dir_n)
        if norm < 1e-6:
            ux, uy = 1.0, 0.0
        else:
            ux, uy = dir_e / norm, dir_n / norm

    x_world_raw = east_m * ux + north_m * uy
    y_world_raw = -east_m * uy + north_m * ux

    x_world = _smooth_series_time_window(t, x_world_raw, smooth_window_s)
    y_world = _smooth_series_time_window(t, y_world_raw, smooth_window_s)

    return {
        "t": t,
        "x_world": x_world,
        "y_world": y_world,
        "x_world_fn": _interp_series(t, x_world),
        "y_world_fn": _interp_series(t, y_world),
        "smooth_window_s": float(smooth_window_s),
    }


def add_distance_gt_and_error(
    df: pd.DataFrame,
    *,
    gt_model: Dict[str, object],
    target_x_world: float,
    target_y_world: float,
) -> pd.DataFrame:
    out = df.copy()
    if out.empty:
        out["distance_m"] = np.array([], dtype=float)
        out["gt_distance_m"] = np.array([], dtype=float)
        out["err_distance_m"] = np.array([], dtype=float)
        out["ego_x_world"] = np.array([], dtype=float)
        out["ego_y_world"] = np.array([], dtype=float)
        return out

    if "header_t" not in out.columns or "x" not in out.columns or "y" not in out.columns:
        raise ValueError("dataset missing required columns: 'header_t', 'x', 'y'")

    t = pd.to_numeric(out["header_t"], errors="coerce").to_numpy(dtype=float)
    x = pd.to_numeric(out["x"], errors="coerce").to_numpy(dtype=float)
    y = pd.to_numeric(out["y"], errors="coerce").to_numpy(dtype=float)

    ego_x_world = gt_model["x_world_fn"](t)
    ego_y_world = gt_model["y_world_fn"](t)
    gt_distance = np.sqrt(np.square(float(target_x_world) - ego_x_world) + np.square(float(target_y_world) - ego_y_world))
    meas_distance = np.sqrt(np.square(x) + np.square(y))
    err_distance = np.abs(meas_distance - gt_distance)

    out["ego_x_world"] = ego_x_world
    out["ego_y_world"] = ego_y_world
    out["target_x_world"] = float(target_x_world)
    out["target_y_world"] = float(target_y_world)
    out["distance_m"] = meas_distance
    out["gt_distance_m"] = gt_distance
    out["err_distance_m"] = err_distance
    return out


def filter_by_distance_gate(df: pd.DataFrame, max_dist_error_m: float) -> pd.DataFrame:
    if df.empty or "err_distance_m" not in df.columns:
        return df
    max_dist_error_m = float(max_dist_error_m)
    if max_dist_error_m <= 0.0:
        return df
    err = pd.to_numeric(df["err_distance_m"], errors="coerce").to_numpy(dtype=float)
    mask = np.isfinite(err) & (err <= max_dist_error_m)
    return df.loc[mask].copy()


def rmse_from_df(df: pd.DataFrame) -> float:
    if df.empty or "err_distance_m" not in df.columns:
        return float("nan")
    err = pd.to_numeric(df["err_distance_m"], errors="coerce").to_numpy(dtype=float)
    err = err[np.isfinite(err)]
    if err.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(err * err)))


def plot_xy_only(
    radar_df: pd.DataFrame,
    camera_df: pd.DataFrame,
    fused_df: pd.DataFrame,
    tracks_df: pd.DataFrame,
    out_prefix: str,
) -> None:
    out_dir = str(Path(out_prefix).parent)
    if out_dir and out_dir != ".":
        Path(out_dir).mkdir(parents=True, exist_ok=True)

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

    plt.figure(figsize=(8, 12))
    if not fused_df.empty:
        plt.scatter(fused_df["y"], fused_df["x"], marker="x", color="r", s=8, alpha=0.7, label="Fused")
    if not radar_df.empty:
        plt.scatter(radar_df["y"], radar_df["x"], marker=".", color="g", s=2, alpha=0.7, label="Radar detections")
    if not camera_df.empty:
        plt.scatter(camera_df["y"], camera_df["x"], marker=".", color="y", s=2, alpha=0.7, label="Camera detections")
    plt.xlabel("y [m]")
    plt.ylabel("x [m]")
    plt.title("Real-world distance analysis: Raw & Fused")
    _legend_if_any()
    plt.ylim(-2, 35)
    plt.tight_layout()
    plt.savefig(f"{out_prefix}_raw_fused.png", dpi=300)
    plt.close()

    plt.figure(figsize=(8, 12))
    if not fused_df.empty:
        plt.scatter(fused_df["y"], fused_df["x"], marker="x", color="r", s=8, alpha=0.7, label="Fused")
    _plot_tracks_grouped(tracks_df)
    plt.xlabel("y [m]")
    plt.ylabel("x [m]")
    plt.title("Real-world distance analysis: Fused & Tracks")
    _legend_if_any()
    plt.ylim(-2, 35)
    plt.tight_layout()
    plt.savefig(f"{out_prefix}_fused_tracks.png", dpi=300)
    plt.close()

    plt.figure(figsize=(8, 12))
    _plot_tracks_grouped(tracks_df)
    plt.xlabel("y [m]")
    plt.ylabel("x [m]")
    plt.title("Real-world distance analysis: Tracks")
    _legend_if_any()
    plt.ylim(-2, 35)
    plt.tight_layout()
    plt.savefig(f"{out_prefix}_tracks.png", dpi=300)
    plt.close()


def plot_distance_vs_time(
    radar_df: pd.DataFrame,
    camera_df: pd.DataFrame,
    fused_df: pd.DataFrame,
    tracks_df: pd.DataFrame,
    gt_model: Dict[str, object],
    *,
    target_x_world: float,
    target_y_world: float,
    out_prefix: str,
) -> None:
    plt.figure(figsize=(12, 6))

    t_gt = np.asarray(gt_model["t"], dtype=float)
    gt_distance = np.sqrt(
        np.square(float(target_x_world) - np.asarray(gt_model["x_world"], dtype=float))
        + np.square(float(target_y_world) - np.asarray(gt_model["y_world"], dtype=float))
    )
    t0 = float(t_gt[0]) if t_gt.size else 0.0

    plt.plot(t_gt - t0, gt_distance, color="black", linewidth=2.0, alpha=0.9, label="Ground truth distance")

    def _scatter_dist(df: pd.DataFrame, label: str, color: str):
        if df.empty or "distance_m" not in df.columns or "header_t" not in df.columns:
            return
        tt = pd.to_numeric(df["header_t"], errors="coerce").to_numpy(dtype=float)
        dd = pd.to_numeric(df["distance_m"], errors="coerce").to_numpy(dtype=float)
        mask = np.isfinite(tt) & np.isfinite(dd)
        if not np.any(mask):
            return
        plt.scatter(tt[mask] - t0, dd[mask], s=6, alpha=0.45, color=color, label=label)

    _scatter_dist(radar_df, "Radar detections", "g")
    _scatter_dist(camera_df, "Camera detections", "gold")
    _scatter_dist(fused_df, "Fused detections", "r")
    _scatter_dist(tracks_df, "Tracks", "b")

    plt.xlabel("time since first GPS sample [s]")
    plt.ylabel("distance to target [m]")
    plt.title("Real-world distance analysis: Distance vs Time")
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.tight_layout()
    plt.savefig(f"{out_prefix}_distance_vs_time.png", dpi=300)
    plt.close()


def plot_distance_error_vs_time(
    radar_df: pd.DataFrame,
    camera_df: pd.DataFrame,
    fused_df: pd.DataFrame,
    tracks_df: pd.DataFrame,
    gt_model: Dict[str, object],
    *,
    out_prefix: str,
) -> None:
    plt.figure(figsize=(12, 6))

    t_gt = np.asarray(gt_model["t"], dtype=float)
    t0 = float(t_gt[0]) if t_gt.size else 0.0

    def _scatter_err(df: pd.DataFrame, label: str, color: str):
        if df.empty or "err_distance_m" not in df.columns or "header_t" not in df.columns:
            return
        tt = pd.to_numeric(df["header_t"], errors="coerce").to_numpy(dtype=float)
        ee = pd.to_numeric(df["err_distance_m"], errors="coerce").to_numpy(dtype=float)
        mask = np.isfinite(tt) & np.isfinite(ee)
        if not np.any(mask):
            return
        plt.scatter(tt[mask] - t0, ee[mask], s=8, alpha=0.5, color=color, label=label)

    _scatter_err(radar_df, "Radar error", "g")
    _scatter_err(camera_df, "Camera error", "gold")
    _scatter_err(fused_df, "Fused error", "r")
    _scatter_err(tracks_df, "Tracks error", "b")

    plt.axhline(0.0, color="black", linewidth=1.0, alpha=0.7, linestyle="--")
    plt.xlabel("time since first GPS sample [s]")
    plt.ylabel("distance error to GT [m]")
    plt.title("Real-world distance analysis: Distance Error vs Time")
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.tight_layout()
    plt.savefig(f"{out_prefix}_distance_error_vs_time.png", dpi=300)
    plt.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Distance-only real-world analysis")
    parser.add_argument("test_name", type=str, help="Test name, e.g. t1_2 or t1_2_replay")
    parser.add_argument("--log-dir", type=str, default="real_world_logs", help="Directory containing real-world logger CSVs")
    parser.add_argument("--out-prefix", type=str, default=None, help="Output path prefix for metrics/plots")
    parser.add_argument("--tf-file", type=str, default=None, help="Optional TF file override")
    parser.add_argument(
        "--distance-gate-m",
        type=float,
        default=3.0,
        help="Keep only detections/tracks within this distance error from GT before RMSE. Use <=0 to disable. Default: 3.0",
    )
    parser.add_argument(
        "--gps-smooth-window-s",
        type=float,
        default=0.0,
        help="Time window in seconds for GPS position smoothing before distance GT generation. Set <=0 to disable. Default: 0 (disabled).",
    )
    parser.add_argument(
        "--axis-gps-file",
        type=str,
        default=None,
        help=(
            "Path to a t1 GPS CSV file used to define the world-frame x-axis direction "
            "(start→end of that run). When set, all runs share the same axis so that "
            "the fixed target coordinates (29, ±1.5 m etc.) remain consistent. "
            "If omitted the current run's own GPS start→end direction is used."
        ),
    )
    args = parser.parse_args()

    test_name = str(args.test_name)
    log_dir = Path(str(args.log_dir))
    out_prefix = str(args.out_prefix) if args.out_prefix else f"plots/{test_name}_distance"
    target_x_world, target_y_world = resolve_target_world_from_test_name(test_name)

    if re.match(r"^t[1-5](?:_|$)", test_name.strip().lower()) is None:
        print(
            f"[WARN] Could not infer scenario (t1..t5) from test_name '{test_name}'. "
            f"Using default target y=0.00 m."
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

    def _resolve_log_path(stem: str) -> Path:
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

    missing = [p for p in [raw_path, fused_path, tracks_path, gps_path] if not p.exists()]
    if missing:
        raise FileNotFoundError("Missing required log files:\n  - " + "\n  - ".join(str(p) for p in missing))

    print(f"Analyzing distance-only real-world test '{test_name}'")
    print(f"  Log directory: {log_dir}")
    print(f"  TF file: {tf_path}")

    axis_gps_df: pd.DataFrame | None = None
    if args.axis_gps_file is not None:
        axis_gps_path = Path(str(args.axis_gps_file))
        if not axis_gps_path.exists():
            raise FileNotFoundError(f"--axis-gps-file not found: {axis_gps_path}")
        axis_gps_df = load_gps_truth(str(axis_gps_path))
        print(f"  Axis GPS file: {axis_gps_path}")
        ux_ax, uy_ax = _axis_direction_from_gps_df(axis_gps_df)
        print(f"  World-frame axis (ux, uy) from axis GPS: ({ux_ax:.6f}, {uy_ax:.6f})")
    else:
        print("  Axis GPS file: (not set — using this run's own GPS start→end direction)")

    gps_df = load_gps_truth(str(gps_path))
    radar_df, camera_df = load_raw_detections(
        str(raw_path),
        radar_tf_base_to_sensor=radar_tf,
        camera_tf_base_to_sensor=camera_tf,
    )
    fused_df = load_fused(str(fused_path))
    tracks_df = load_tracks(str(tracks_path))

    if not fused_df.empty and "detection_type" in fused_df.columns:
        fused_df = fused_df[fused_df["detection_type"].astype(str) == "fused"].copy()

    gt_model = build_distance_ground_truth_model(
        gps_df,
        smooth_window_s=float(args.gps_smooth_window_s),
        axis_gps_df=axis_gps_df,
    )

    origin_time = float(gt_model["t"][0])
    gps_displacement_m = float(
        np.hypot(
            float(gt_model["x_world"][-1] - gt_model["x_world"][0]),
            float(gt_model["y_world"][-1] - gt_model["y_world"][0]),
        )
    )
    print(f"\nWorld origin time (first GPS sample): {origin_time:.3f}")
    print(f"Target world position: ({target_x_world:.2f}, {target_y_world:.2f}) m")
    print("GT metric: distance from ego RTK-GPS position to fixed target world position")
    print(f"GPS smoothing window: {float(args.gps_smooth_window_s):.2f} s")
    print(f"GPS world displacement: {gps_displacement_m:.2f} m")

    radar_before = len(radar_df)
    fused_before = len(fused_df)
    tracks_before = len(tracks_df)
    radar_df = filter_xy_box(radar_df)
    fused_df = filter_xy_box(fused_df)
    tracks_df = filter_xy_box(tracks_df)
    print(f"\nFiltering radar detections: x in [0,35], y in [-6,6] -> kept {len(radar_df)}/{radar_before}")
    print(f"Filtering fused detections: x in [0,35], y in [-6,6] -> kept {len(fused_df)}/{fused_before}")
    print(f"Filtering tracks: x in [0,35], y in [-6,6] -> kept {len(tracks_df)}/{tracks_before}")

    radar_w = add_distance_gt_and_error(radar_df, gt_model=gt_model, target_x_world=target_x_world, target_y_world=target_y_world)
    camera_w = add_distance_gt_and_error(camera_df, gt_model=gt_model, target_x_world=target_x_world, target_y_world=target_y_world)
    fused_w = add_distance_gt_and_error(fused_df, gt_model=gt_model, target_x_world=target_x_world, target_y_world=target_y_world)
    tracks_w = add_distance_gt_and_error(tracks_df, gt_model=gt_model, target_x_world=target_x_world, target_y_world=target_y_world)

    if float(args.distance_gate_m) > 0.0:
        radar_before_gate = len(radar_w)
        camera_before_gate = len(camera_w)
        fused_before_gate = len(fused_w)
        tracks_before_gate = len(tracks_w)
        radar_w = filter_by_distance_gate(radar_w, float(args.distance_gate_m))
        camera_w = filter_by_distance_gate(camera_w, float(args.distance_gate_m))
        fused_w = filter_by_distance_gate(fused_w, float(args.distance_gate_m))
        tracks_w = filter_by_distance_gate(tracks_w, float(args.distance_gate_m))
        print(
            f"Distance gate ({float(args.distance_gate_m):.2f} m): "
            f"radar {len(radar_w)}/{radar_before_gate}, "
            f"camera {len(camera_w)}/{camera_before_gate}, "
            f"fused {len(fused_w)}/{fused_before_gate}, "
            f"tracks {len(tracks_w)}/{tracks_before_gate}"
        )

    rmse_radar = rmse_from_df(radar_w)
    rmse_camera = rmse_from_df(camera_w)
    rmse_fused = rmse_from_df(fused_w)
    rmse_tracks = rmse_from_df(tracks_w)

    def _fmt(name: str, rmse: float, n: int) -> str:
        if math.isnan(rmse):
            return f"  {name:<18} RMSE NaN (n={n})"
        return f"  {name:<18} RMSE {rmse:.3f} m (n={n})"

    print(f"\n{'=' * 70}")
    print(f"Distance-only RMSE Summary for test {test_name}:")
    print(f"{'=' * 70}")
    print(_fmt("Radar detections:", rmse_radar, len(radar_w)))
    print(_fmt("Camera detections:", rmse_camera, len(camera_w)))
    print(_fmt("Fused detections:", rmse_fused, len(fused_w)))
    print(_fmt("Tracks:", rmse_tracks, len(tracks_w)))
    print(f"{'=' * 70}")

    metrics_path = Path(out_prefix + "_metrics.txt")
    metrics_path.parent.mkdir(parents=True, exist_ok=True)
    with open(metrics_path, "w", encoding="utf-8") as f:
        f.write(f"Real-World Distance-Only Test Analysis: {test_name}\n")
        f.write("Ground truth: distance from RTK-GPS ego position to fixed target world position\n")
        f.write(f"World origin time (first GPS sample): {origin_time:.3f}\n")
        f.write(f"Static target world position: ({target_x_world:.2f}, {target_y_world:.2f}) m\n")
        f.write(f"GPS world displacement: {gps_displacement_m:.2f} m\n")
        f.write(f"GPS smoothing window: {float(args.gps_smooth_window_s):.2f} s\n")
        f.write("Filter (radar/fused/tracks): x in [0,35], y in [-6,6]\n")
        f.write("Camera: no x/y filter applied\n\n")
        if float(args.distance_gate_m) > 0.0:
            f.write(f"Distance gate: <= {float(args.distance_gate_m):.2f} m\n\n")
        else:
            f.write("Distance gate: disabled\n\n")
        f.write(_fmt("Radar", rmse_radar, len(radar_w)) + "\n")
        f.write(_fmt("Camera", rmse_camera, len(camera_w)) + "\n")
        f.write(_fmt("Fused", rmse_fused, len(fused_w)) + "\n")
        f.write(_fmt("Tracks", rmse_tracks, len(tracks_w)) + "\n")

    print(f"\nMetrics saved to: {metrics_path}")

    print("\nGenerating plots...")
    plot_xy_only(radar_w, camera_w, fused_w, tracks_w, out_prefix=out_prefix)
    plot_distance_vs_time(
        radar_w,
        camera_w,
        fused_w,
        tracks_w,
        gt_model,
        target_x_world=target_x_world,
        target_y_world=target_y_world,
        out_prefix=out_prefix,
    )
    plot_distance_error_vs_time(
        radar_w,
        camera_w,
        fused_w,
        tracks_w,
        gt_model,
        out_prefix=out_prefix,
    )
    print(f"Plots saved with prefix: {out_prefix}")
    print("\nAnalysis complete!")


if __name__ == "__main__":
    main()
