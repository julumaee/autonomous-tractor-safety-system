#!/usr/bin/env python3
"""Batch distance-only analysis over replay runs (t1..t5, _1.._7).

Produces paper-ready metrics using the same distance-only error model as
analyze_real_world_distance.py, but aggregated by scenario across all 7 runs.

Outputs:
- <out_prefix>_run_level.csv           (one row per run x detection_type)
- <out_prefix>_scenario_pooled.csv     (pooled across 7 runs per scenario)
- <out_prefix>_scenario_run_stats.csv  (mean/std across per-run metrics)
- <out_prefix>_summary.txt             (method + compact text summary)
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import pandas as pd

import analyze_real_world_distance as ard


SCENARIOS: Tuple[str, ...] = ("t1", "t2", "t3", "t4", "t5")
RUN_IDS: Tuple[int, ...] = (1, 2, 3, 4, 5, 6, 7)
DET_TYPES: Tuple[str, ...] = ("radar", "camera", "fused", "tracks")
TRACK_HZ: float = 20.0


def _safe_array(df: pd.DataFrame) -> np.ndarray:
    if df.empty or "err_distance_m" not in df.columns:
        return np.array([], dtype=float)
    arr = pd.to_numeric(df["err_distance_m"], errors="coerce").to_numpy(dtype=float)
    return arr[np.isfinite(arr)]


def _safe_column_array(df: pd.DataFrame, col: str) -> np.ndarray:
    if df.empty or col not in df.columns:
        return np.array([], dtype=float)
    arr = pd.to_numeric(df[col], errors="coerce").to_numpy(dtype=float)
    return arr[np.isfinite(arr)]


def _rmse_from_err(err: np.ndarray) -> float:
    if err.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(err * err)))


def _r95_from_err(err: np.ndarray) -> float:
    if err.size == 0:
        return float("nan")
    return float(np.percentile(err, 95.0))


def _min_from_arr(arr: np.ndarray) -> float:
    if arr.size == 0:
        return float("nan")
    return float(np.min(arr))


def _max_from_arr(arr: np.ndarray) -> float:
    if arr.size == 0:
        return float("nan")
    return float(np.max(arr))


def _d05_from_arr(arr: np.ndarray) -> float:
    """5th percentile of detection distances: robust lower range bound."""
    if arr.size == 0:
        return float("nan")
    return float(np.percentile(arr, 5.0))


def _compute_track_continuity_metrics(
    tracks_df: pd.DataFrame,
    *,
    eval_end_time_s: float,
    time_col: str,
    hz: float = TRACK_HZ,
) -> Dict[str, float]:
    out = {
        "track_availability_after_lockon": float("nan"),
        "track_dropout_count": float("nan"),
        "track_longest_miss_gap_s": float("nan"),
        "track_reacquisition_time_s": float("nan"),
    }
    if tracks_df.empty or time_col not in tracks_df.columns:
        return out

    t = pd.to_numeric(tracks_df[time_col], errors="coerce").to_numpy(dtype=float)
    t = t[np.isfinite(t)]
    if t.size == 0:
        return out

    dt = 1.0 / float(hz)
    t_start = float(np.min(t))  # first formed target track
    t_end = float(eval_end_time_s)
    if not np.isfinite(t_end) or t_end < t_start:
        t_end = float(np.max(t))
    if t_end < t_start:
        return out

    n_steps = int(np.floor((t_end - t_start) / dt + 0.5)) + 1
    if n_steps <= 0:
        return out

    present = np.zeros(n_steps, dtype=bool)
    idx = np.rint((t - t_start) / dt).astype(int)
    valid = (idx >= 0) & (idx < n_steps)
    present[idx[valid]] = True

    miss = ~present
    present_count = int(np.count_nonzero(present))
    eval_count = int(n_steps)

    miss_runs = 0
    longest_run = 0
    run_len = 0
    reacq_times_s: List[float] = []
    for is_miss in miss:
        if is_miss:
            run_len += 1
            if run_len == 1:
                miss_runs += 1
            if run_len > longest_run:
                longest_run = run_len
        else:
            if run_len > 0:
                reacq_times_s.append(float(run_len * dt))
            run_len = 0

    out["track_availability_after_lockon"] = float(present_count / eval_count) if eval_count > 0 else float("nan")
    out["track_dropout_count"] = float(miss_runs)
    out["track_longest_miss_gap_s"] = float(longest_run * dt)
    if len(reacq_times_s) == 0:
        out["track_reacquisition_time_s"] = 0.0 if miss_runs == 0 else float("nan")
    else:
        out["track_reacquisition_time_s"] = float(np.mean(reacq_times_s))
    return out


def _resolve_log_path(log_dir: Path, stem: str, test_name: str) -> Path:
    p = log_dir / f"{stem}_{test_name}.csv"
    if p.exists():
        return p
    legacy = log_dir / f"{stem}_{test_name}_approach.csv"
    if legacy.exists():
        return legacy
    return p


def _process_one_run(
    *,
    test_name: str,
    log_dir: Path,
    radar_tf: np.ndarray,
    camera_tf: np.ndarray,
    distance_gate_m: float,
    gps_smooth_window_s: float = 0.0,
    axis_gps_df: pd.DataFrame | None,
) -> Dict[str, object]:
    target_x_world, target_y_world = ard.resolve_target_world_from_test_name(test_name)

    raw_path = _resolve_log_path(log_dir, "raw_detections", test_name)
    fused_path = _resolve_log_path(log_dir, "fused_detections", test_name)
    tracks_path = _resolve_log_path(log_dir, "tracks", test_name)
    gps_path = _resolve_log_path(log_dir, "gps_truth", test_name)

    required = [raw_path, fused_path, tracks_path, gps_path]
    missing = [str(p) for p in required if not p.exists()]
    if missing:
        raise FileNotFoundError("Missing required files:\n  - " + "\n  - ".join(missing))

    gps_df = ard.load_gps_truth(str(gps_path))
    radar_df, camera_df = ard.load_raw_detections(
        str(raw_path),
        radar_tf_base_to_sensor=radar_tf,
        camera_tf_base_to_sensor=camera_tf,
    )
    fused_df = ard.load_fused(str(fused_path))
    tracks_df = ard.load_tracks(str(tracks_path))

    if not fused_df.empty and "detection_type" in fused_df.columns:
        fused_df = fused_df[fused_df["detection_type"].astype(str) == "fused"].copy()

    radar_df = ard.filter_xy_box(radar_df)
    fused_df = ard.filter_xy_box(fused_df)
    tracks_df = ard.filter_xy_box(tracks_df)

    gt_model = ard.build_distance_ground_truth_model(
        gps_df,
        smooth_window_s=float(gps_smooth_window_s),
        axis_gps_df=axis_gps_df,
    )

    radar_w = ard.add_distance_gt_and_error(
        radar_df,
        gt_model=gt_model,
        target_x_world=target_x_world,
        target_y_world=target_y_world,
    )
    camera_w = ard.add_distance_gt_and_error(
        camera_df,
        gt_model=gt_model,
        target_x_world=target_x_world,
        target_y_world=target_y_world,
    )
    fused_w = ard.add_distance_gt_and_error(
        fused_df,
        gt_model=gt_model,
        target_x_world=target_x_world,
        target_y_world=target_y_world,
    )
    tracks_w = ard.add_distance_gt_and_error(
        tracks_df,
        gt_model=gt_model,
        target_x_world=target_x_world,
        target_y_world=target_y_world,
    )

    if float(distance_gate_m) > 0.0:
        radar_w = ard.filter_by_distance_gate(radar_w, float(distance_gate_m))
        camera_w = ard.filter_by_distance_gate(camera_w, float(distance_gate_m))
        fused_w = ard.filter_by_distance_gate(fused_w, float(distance_gate_m))
        tracks_w = ard.filter_by_distance_gate(tracks_w, float(distance_gate_m))

    def _last_used_detection_time(df: pd.DataFrame) -> float:
        if df.empty:
            return float("nan")
        t_col = "logger_t" if "logger_t" in df.columns else "header_t"
        if t_col not in df.columns:
            return float("nan")
        t = pd.to_numeric(df[t_col], errors="coerce").to_numpy(dtype=float)
        t = t[np.isfinite(t)]
        if t.size == 0:
            return float("nan")
        return float(np.max(t))

    det_end_candidates = [
        _last_used_detection_time(radar_w),
        _last_used_detection_time(camera_w),
        _last_used_detection_time(fused_w),
    ]
    det_end_candidates = [x for x in det_end_candidates if np.isfinite(x)]

    track_time_col = "logger_t" if "logger_t" in tracks_w.columns else "header_t"
    tracks_end = _last_used_detection_time(tracks_w)
    if det_end_candidates:
        eval_end_time_s = float(max(det_end_candidates))
    else:
        eval_end_time_s = tracks_end

    track_metrics = _compute_track_continuity_metrics(
        tracks_w,
        eval_end_time_s=eval_end_time_s,
        time_col=track_time_col,
        hz=TRACK_HZ,
    )

    return {
        "err_by_type": {
            "radar": _safe_array(radar_w),
            "camera": _safe_array(camera_w),
            "fused": _safe_array(fused_w),
            "tracks": _safe_array(tracks_w),
        },
        "distance_by_type": {
            "radar": _safe_column_array(radar_w, "distance_m"),
            "camera": _safe_column_array(camera_w, "distance_m"),
            "fused": _safe_column_array(fused_w, "distance_m"),
            "tracks": _safe_column_array(tracks_w, "distance_m"),
        },
        "track_metrics": track_metrics,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Batch distance-only real-world analysis for replay runs")
    parser.add_argument("--log-dir", type=str, default="real_world_logs", help="Directory containing replay CSV logs")
    parser.add_argument(
        "--out-prefix",
        type=str,
        default="plots/replay_distance_batch",
        help="Output path prefix for batch metric tables",
    )
    parser.add_argument("--tf-file", type=str, default="calibration_log/replay_tf.txt", help="TF file used for replay logs")
    parser.add_argument(
        "--distance-gate-m",
        type=float,
        default=3.0,
        help="Keep only detections/tracks within this distance error to GT before metrics (<=0 disables)",
    )
    parser.add_argument(
        "--gps-smooth-window-s",
        type=float,
        default=0.0,
        help="Time window in seconds for GPS smoothing in distance GT model. Default: 0 (disabled).",
    )
    parser.add_argument(
        "--axis-gps-file",
        type=str,
        default=None,
        help="Optional GPS CSV (typically t1 replay) used to define world x-axis for all runs",
    )
    parser.add_argument(
        "--gps-sanity-file",
        type=str,
        default=None,
        help=(
            "Optional path to the replay_gps_sanity_run_level.csv produced by "
            "check_replay_gps_sanity. Runs with is_flagged==True are automatically excluded."
        ),
    )
    parser.add_argument(
        "--exclude-runs",
        type=str,
        default="",
        help="Comma-separated list of test names to exclude (e.g. t5_2_replay,t5_6_replay).",
    )
    args = parser.parse_args()

    log_dir = Path(str(args.log_dir))
    out_prefix = Path(str(args.out_prefix))
    out_prefix.parent.mkdir(parents=True, exist_ok=True)

    tf_path = Path(str(args.tf_file))
    tf = ard._parse_calibration_tf_file(tf_path)
    radar_tf = tf.get("radar_tf", np.zeros(6, dtype=float))
    camera_tf = tf.get("camera_tf", np.zeros(6, dtype=float))

    axis_gps_df: pd.DataFrame | None = None
    axis_gps_file_used = ""
    if args.axis_gps_file:
        axis_path = Path(str(args.axis_gps_file))
        if not axis_path.exists():
            raise FileNotFoundError(f"--axis-gps-file not found: {axis_path}")
        axis_gps_df = ard.load_gps_truth(str(axis_path))
        axis_gps_file_used = str(axis_path)

    # Build the set of test names to exclude.
    excluded_runs: set[str] = set()
    excluded_runs_source: List[str] = []
    if args.gps_sanity_file:
        sanity_path = Path(str(args.gps_sanity_file))
        if not sanity_path.exists():
            raise FileNotFoundError(f"--gps-sanity-file not found: {sanity_path}")
        sanity_df = pd.read_csv(sanity_path)
        if "test_name" not in sanity_df.columns or "is_flagged" not in sanity_df.columns:
            raise ValueError("--gps-sanity-file must have 'test_name' and 'is_flagged' columns")
        flagged_names = sanity_df.loc[sanity_df["is_flagged"] == True, "test_name"].tolist()
        for name in flagged_names:
            excluded_runs.add(str(name))
            excluded_runs_source.append(f"{name} (gps-sanity flagged)")
    if args.exclude_runs:
        for name in args.exclude_runs.split(","):
            name = name.strip()
            if name:
                excluded_runs.add(name)
                excluded_runs_source.append(f"{name} (--exclude-runs)")
    if excluded_runs_source:
        print(f"Excluded runs ({len(excluded_runs)}):")
        for s in excluded_runs_source:
            print(f"  - {s}")

    run_rows: List[Dict[str, object]] = []
    pooled_errors: Dict[Tuple[str, str], List[np.ndarray]] = {
        (scenario, det_type): [] for scenario in SCENARIOS for det_type in DET_TYPES
    }
    pooled_distances: Dict[Tuple[str, str], List[np.ndarray]] = {
        (scenario, det_type): [] for scenario in SCENARIOS for det_type in DET_TYPES
    }

    missing_runs: List[str] = []
    failed_runs: List[Tuple[str, str]] = []

    excluded_list: List[str] = []

    for scenario in SCENARIOS:
        for run_id in RUN_IDS:
            test_name = f"{scenario}_{run_id}_replay"
            if test_name in excluded_runs:
                excluded_list.append(test_name)
                continue
            raw_path = _resolve_log_path(log_dir, "raw_detections", test_name)
            fused_path = _resolve_log_path(log_dir, "fused_detections", test_name)
            tracks_path = _resolve_log_path(log_dir, "tracks", test_name)
            gps_path = _resolve_log_path(log_dir, "gps_truth", test_name)
            if not (raw_path.exists() and fused_path.exists() and tracks_path.exists() and gps_path.exists()):
                missing_runs.append(test_name)
                continue

            try:
                run_result = _process_one_run(
                    test_name=test_name,
                    log_dir=log_dir,
                    radar_tf=radar_tf,
                    camera_tf=camera_tf,
                    distance_gate_m=float(args.distance_gate_m),
                    gps_smooth_window_s=float(args.gps_smooth_window_s),
                    axis_gps_df=axis_gps_df,
                )
            except Exception as exc:
                failed_runs.append((test_name, str(exc)))
                continue

            err_by_type = run_result["err_by_type"]
            distance_by_type = run_result["distance_by_type"]
            track_metrics = run_result["track_metrics"]

            for det_type in DET_TYPES:
                err = err_by_type[det_type]
                distance_arr = distance_by_type[det_type]
                n = int(err.size)
                distance_min_m = _min_from_arr(distance_arr) if det_type in ("radar", "camera") else float("nan")
                distance_max_m = _max_from_arr(distance_arr) if det_type in ("radar", "camera") else float("nan")
                distance_d05_m = _d05_from_arr(distance_arr) if det_type in ("radar", "camera") else float("nan")

                row = {
                    "scenario": scenario,
                    "run_id": run_id,
                    "test_name": test_name,
                    "detection_type": det_type,
                    "n": n,
                    "rmse_m": _rmse_from_err(err),
                    "r95_m": _r95_from_err(err),
                    "distance_min_m": distance_min_m,
                    "distance_max_m": distance_max_m,
                    "distance_d05_m": distance_d05_m,
                    "distance_gate_m": float(args.distance_gate_m),
                    "gps_smooth_window_s": float(args.gps_smooth_window_s),
                    "tf_file": str(tf_path),
                    "axis_gps_file": axis_gps_file_used,
                }

                if det_type == "tracks":
                    row["distance_d05_m"] = float("nan")
                    row.update(track_metrics)
                else:
                    row.update(
                        {
                            "track_availability_after_lockon": float("nan"),
                            "track_dropout_count": float("nan"),
                            "track_longest_miss_gap_s": float("nan"),
                            "track_reacquisition_time_s": float("nan"),
                        }
                    )

                run_rows.append(
                    row
                )
                if n > 0:
                    pooled_errors[(scenario, det_type)].append(err)
                if distance_arr.size > 0:
                    pooled_distances[(scenario, det_type)].append(distance_arr)

    run_df = pd.DataFrame(run_rows)
    if run_df.empty:
        raise RuntimeError("No runs were processed successfully. Check --log-dir and replay file names.")

    pooled_rows: List[Dict[str, object]] = []
    for scenario in SCENARIOS:
        for det_type in DET_TYPES:
            pieces = pooled_errors[(scenario, det_type)]
            if pieces:
                err = np.concatenate(pieces)
            else:
                err = np.array([], dtype=float)

            dist_pieces = pooled_distances[(scenario, det_type)]
            dist = np.concatenate(dist_pieces) if dist_pieces else np.array([], dtype=float)

            pooled_rows.append(
                {
                    "scenario": scenario,
                    "detection_type": det_type,
                    "n": int(err.size),
                    "rmse_m": _rmse_from_err(err),
                    "r95_m": _r95_from_err(err),
                    "distance_min_m": _min_from_arr(dist) if det_type in ("radar", "camera") else float("nan"),
                    "distance_max_m": _max_from_arr(dist) if det_type in ("radar", "camera") else float("nan"),
                    "distance_d05_m": _d05_from_arr(dist) if det_type in ("radar", "camera") else float("nan"),
                    "track_availability_after_lockon": float("nan"),
                    "track_dropout_count": float("nan"),
                    "track_longest_miss_gap_s": float("nan"),
                    "track_reacquisition_time_s": float("nan"),
                }
            )
    pooled_df = pd.DataFrame(pooled_rows)

    valid_run_df = run_df.copy()
    valid_run_df = valid_run_df[pd.to_numeric(valid_run_df["n"], errors="coerce").fillna(0) > 0].copy()
    if valid_run_df.empty:
        run_stats_df = pd.DataFrame(
            columns=[
                "scenario",
                "detection_type",
                "run_count",
                "rmse_mean_m",
                "rmse_std_m",
                "r95_mean_m",
                "r95_std_m",
            ]
        )
    else:
        grouped = valid_run_df.groupby(["scenario", "detection_type"], dropna=False)
        run_stats_df = grouped.agg(
            run_count=("test_name", "count"),
            rmse_mean_m=("rmse_m", "mean"),
            rmse_std_m=("rmse_m", "std"),
            r95_mean_m=("r95_m", "mean"),
            r95_std_m=("r95_m", "std"),
            distance_min_mean_m=("distance_min_m", "mean"),
            distance_max_mean_m=("distance_max_m", "mean"),
            distance_d05_mean_m=("distance_d05_m", "mean"),
            track_availability_after_lockon_mean=("track_availability_after_lockon", "mean"),
            track_dropout_count_mean=("track_dropout_count", "mean"),
            track_longest_miss_gap_s_mean=("track_longest_miss_gap_s", "mean"),
            track_reacquisition_time_s_mean=("track_reacquisition_time_s", "mean"),
        ).reset_index()

    run_csv = Path(str(out_prefix) + "_run_level.csv")
    pooled_csv = Path(str(out_prefix) + "_scenario_pooled.csv")
    pooled_pivot_csv = Path(str(out_prefix) + "_scenario_pooled_pivot.csv")
    run_stats_csv = Path(str(out_prefix) + "_scenario_run_stats.csv")
    run_stats_pivot_csv = Path(str(out_prefix) + "_scenario_run_stats_pivot.csv")
    summary_txt = Path(str(out_prefix) + "_summary.txt")

    run_df.to_csv(run_csv, index=False)
    pooled_df.to_csv(pooled_csv, index=False)
    run_stats_df.to_csv(run_stats_csv, index=False)

    # Compact paper-friendly pivots.
    pooled_pivot = pooled_df.pivot(index="scenario", columns="detection_type", values=["n", "rmse_m", "r95_m"])
    pooled_pivot.columns = [f"{metric}_{det_type}" for metric, det_type in pooled_pivot.columns]
    pooled_pivot = pooled_pivot.reset_index()
    pooled_pivot.to_csv(pooled_pivot_csv, index=False)

    if not run_stats_df.empty:
        run_stats_pivot = run_stats_df.pivot(
            index="scenario",
            columns="detection_type",
            values=[
                "run_count",
                "rmse_mean_m",
                "rmse_std_m",
                "r95_mean_m",
                "r95_std_m",
                "distance_min_mean_m",
                "distance_max_mean_m",
                "distance_d05_mean_m",
                "track_availability_after_lockon_mean",
                "track_dropout_count_mean",
                "track_longest_miss_gap_s_mean",
                "track_reacquisition_time_s_mean",
            ],
        )
        run_stats_pivot.columns = [f"{metric}_{det_type}" for metric, det_type in run_stats_pivot.columns]
        run_stats_pivot = run_stats_pivot.reset_index()
    else:
        run_stats_pivot = pd.DataFrame()
    run_stats_pivot.to_csv(run_stats_pivot_csv, index=False)

    # Concise paper-ready table with only requested added metrics (scenario means over runs).
    concise_rows: List[Dict[str, object]] = []
    for scenario in SCENARIOS:
        radar_sub = run_df[(run_df["scenario"] == scenario) & (run_df["detection_type"] == "radar")]
        camera_sub = run_df[(run_df["scenario"] == scenario) & (run_df["detection_type"] == "camera")]
        tracks_sub = run_df[(run_df["scenario"] == scenario) & (run_df["detection_type"] == "tracks")]
        concise_rows.append(
            {
                "scenario": scenario,
                "radar_min_m": float(pd.to_numeric(radar_sub["distance_min_m"], errors="coerce").mean()),
                "radar_max_m": float(pd.to_numeric(radar_sub["distance_max_m"], errors="coerce").mean()),
                "radar_d05_m": float(pd.to_numeric(radar_sub["distance_d05_m"], errors="coerce").mean()),
                "camera_min_m": float(pd.to_numeric(camera_sub["distance_min_m"], errors="coerce").mean()),
                "camera_max_m": float(pd.to_numeric(camera_sub["distance_max_m"], errors="coerce").mean()),
                "camera_d05_m": float(pd.to_numeric(camera_sub["distance_d05_m"], errors="coerce").mean()),
                "track_availability_after_lockon": float(pd.to_numeric(tracks_sub["track_availability_after_lockon"], errors="coerce").mean()),
                "track_longest_miss_gap_s": float(pd.to_numeric(tracks_sub["track_longest_miss_gap_s"], errors="coerce").mean()),
                "track_dropout_count": float(pd.to_numeric(tracks_sub["track_dropout_count"], errors="coerce").mean()),
                "track_reacquisition_time_s": float(pd.to_numeric(tracks_sub["track_reacquisition_time_s"], errors="coerce").mean()),
            }
        )
    concise_df = pd.DataFrame(concise_rows)
    concise_csv = Path(str(out_prefix) + "_scenario_added_metrics.csv")
    concise_df.to_csv(concise_csv, index=False)

    with open(summary_txt, "w", encoding="utf-8") as f:
        f.write("Distance-only replay batch analysis\n")
        f.write("================================\n\n")
        f.write("Error definition per detection/track sample:\n")
        f.write("  e = | sqrt(x^2 + y^2) - sqrt((tx-ex)^2 + (ty-ey)^2) |\n")
        f.write("where (x,y) is measured in base_link and (ex,ey) is ego world position from RTK-GPS.\n\n")
        f.write("RMSE definition:\n")
        f.write("  RMSE = sqrt(mean(e^2))\n\n")
        f.write("r95 definition:\n")
        f.write("  r95 = empirical 95th percentile of e (non-parametric)\n\n")
        f.write("Additional metrics:\n")
        f.write("  radar/camera min/max distance and d05 (5th percentile of distances; scenario means over valid runs)\n")
        f.write("  tracks continuity on 20 Hz timeline from first formed track to run end:\n")
        f.write("    availability_after_lockon, longest_miss_gap_s, dropout_count, reacquisition_time_s\n\n")
        f.write(f"Distance gate: {float(args.distance_gate_m):.3f} m\n")
        f.write(f"GPS smoothing window: {float(args.gps_smooth_window_s):.3f} s\n")
        f.write(f"TF file: {tf_path}\n")
        f.write(f"Axis GPS file: {axis_gps_file_used or '(not set)'}\n")
        if excluded_list:
            f.write(f"Excluded runs ({len(excluded_list)}): {', '.join(excluded_list)}\n")
        f.write("\n")
        f.write("Pooled scenario metrics:\n")
        for scenario in SCENARIOS:
            f.write(f"\n[{scenario}]\n")
            sub = pooled_df[pooled_df["scenario"] == scenario]
            for det_type in DET_TYPES:
                row = sub[sub["detection_type"] == det_type]
                if row.empty:
                    f.write(f"  {det_type:<8} n=0, RMSE=NaN, r95=NaN\n")
                    continue
                r = row.iloc[0]
                rmse = r["rmse_m"]
                r95 = r["r95_m"]
                rmse_s = "NaN" if not np.isfinite(rmse) else f"{float(rmse):.3f}"
                r95_s = "NaN" if not np.isfinite(r95) else f"{float(r95):.3f}"
                if det_type in ("radar", "camera"):
                    dmin = r["distance_min_m"]
                    dmax = r["distance_max_m"]
                    d05 = r["distance_d05_m"]
                    dmin_s = "NaN" if not np.isfinite(dmin) else f"{float(dmin):.3f}"
                    dmax_s = "NaN" if not np.isfinite(dmax) else f"{float(dmax):.3f}"
                    d05_s = "NaN" if not np.isfinite(d05) else f"{float(d05):.3f}"
                    f.write(
                        f"  {det_type:<8} n={int(r['n'])}, RMSE={rmse_s} m, r95={r95_s} m, "
                        f"dmin={dmin_s} m, d05={d05_s} m, dmax={dmax_s} m\n"
                    )
                elif det_type == "tracks":
                    availability = r["track_availability_after_lockon"]
                    dropouts = r["track_dropout_count"]
                    long_gap = r["track_longest_miss_gap_s"]
                    reacq = r["track_reacquisition_time_s"]
                    availability_s = "NaN" if not np.isfinite(availability) else f"{float(availability):.3f}"
                    dropouts_s = "NaN" if not np.isfinite(dropouts) else f"{int(round(float(dropouts)))}"
                    long_gap_s = "NaN" if not np.isfinite(long_gap) else f"{float(long_gap):.3f}"
                    reacq_s = "NaN" if not np.isfinite(reacq) else f"{float(reacq):.3f}"
                    f.write(
                        f"  {det_type:<8} n={int(r['n'])}, RMSE={rmse_s} m, r95={r95_s} m, "
                        f"availability={availability_s}, dropouts={dropouts_s}, "
                        f"longest_gap={long_gap_s} s, reacq={reacq_s} s\n"
                    )
                else:
                    f.write(f"  {det_type:<8} n={int(r['n'])}, RMSE={rmse_s} m, r95={r95_s} m\n")

        f.write("\nScenario means of requested added metrics:\n")
        for _, row in concise_df.iterrows():
            f.write(
                f"  {row['scenario']}: "
                f"radar_min={row['radar_min_m']:.3f}, radar_d05={row['radar_d05_m']:.3f}, radar_max={row['radar_max_m']:.3f}, "
                f"camera_min={row['camera_min_m']:.3f}, camera_d05={row['camera_d05_m']:.3f}, camera_max={row['camera_max_m']:.3f}, "
                f"availability={row['track_availability_after_lockon']:.3f}, "
                f"longest_gap={row['track_longest_miss_gap_s']:.3f} s, "
                f"dropouts={row['track_dropout_count']:.3f}, "
                f"reacq={row['track_reacquisition_time_s']:.3f} s\n"
            )

        if excluded_list:
            f.write("\nExcluded runs (GPS sanity / --exclude-runs):\n")
            for name in excluded_list:
                f.write(f"  - {name}\n")
        if missing_runs:
            f.write("\nMissing runs (skipped):\n")
            for name in missing_runs:
                f.write(f"  - {name}\n")
        if failed_runs:
            f.write("\nFailed runs (skipped):\n")
            for name, reason in failed_runs:
                f.write(f"  - {name}: {reason}\n")

    print("Batch distance analysis complete.")
    print(f"  Run-level CSV:          {run_csv}")
    print(f"  Scenario pooled CSV:    {pooled_csv}")
    print(f"  Scenario pooled pivot:  {pooled_pivot_csv}")
    print(f"  Scenario run-stats CSV: {run_stats_csv}")
    print(f"  Run-stats pivot CSV:    {run_stats_pivot_csv}")
    print(f"  Added metrics CSV:      {concise_csv}")
    print(f"  Summary text:           {summary_txt}")

    if missing_runs:
        print(f"[WARN] Missing runs skipped: {len(missing_runs)}")
    if failed_runs:
        print(f"[WARN] Failed runs skipped: {len(failed_runs)}")


if __name__ == "__main__":
    main()
