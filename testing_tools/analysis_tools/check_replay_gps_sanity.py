#!/usr/bin/env python3
"""Sanity-check replay GPS logs by combining quality indicators and repeat consistency.

This script is intended to catch runs whose RTK-GPS logs look superficially valid
(good fix type / accuracy fields) but are spatially inconsistent with the rest of
the repeated runs in the same scenario.

Outputs:
- <out_prefix>_run_level.csv
- <out_prefix>_scenario_summary.csv
- <out_prefix>_summary.txt
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import pandas as pd

import analyze_real_world_distance as ard


GPS_GLOB = "gps_truth_t*_replay.csv"
DEFAULT_AXIS_FILE = "gps_truth_t1_1_replay.csv"


def _scenario_key(stem: str) -> Tuple[str, int]:
    m = re.match(r"^gps_truth_(t[1-5])_(\d+)_replay$", stem)
    if not m:
        raise ValueError(f"Unrecognized GPS replay filename: {stem}")
    return m.group(1), int(m.group(2))


def _build_axis_frame(axis_gps_df: pd.DataFrame) -> Dict[str, float]:
    lat = pd.to_numeric(axis_gps_df["lat_deg"], errors="coerce").to_numpy(dtype=float)
    lon = pd.to_numeric(axis_gps_df["lon_deg"], errors="coerce").to_numpy(dtype=float)
    mask = np.isfinite(lat) & np.isfinite(lon)
    lat = lat[mask]
    lon = lon[mask]
    if lat.size < 2:
        raise ValueError("axis GPS file has too few valid samples")
    ux, uy = ard._axis_direction_from_gps_df(axis_gps_df)
    return {
        "lat0_deg": float(lat[0]),
        "lon0_deg": float(lon[0]),
        "ux": float(ux),
        "uy": float(uy),
    }


def _world_xy_from_gps(
    gps_df: pd.DataFrame,
    *,
    axis_frame: Dict[str, float],
    gps_smooth_window_s: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
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

    east_m, north_m = ard._gps_local_xy_m(
        lat,
        lon,
        lat0_deg=float(axis_frame["lat0_deg"]),
        lon0_deg=float(axis_frame["lon0_deg"]),
    )
    ux = float(axis_frame["ux"])
    uy = float(axis_frame["uy"])
    x = east_m * ux + north_m * uy
    y = -east_m * uy + north_m * ux

    if float(gps_smooth_window_s) > 0.0:
        x = ard._smooth_series_time_window(t, x, float(gps_smooth_window_s))
        y = ard._smooth_series_time_window(t, y, float(gps_smooth_window_s))
    return t, x, y


def _median(arr: pd.Series) -> float:
    vals = pd.to_numeric(arr, errors="coerce").to_numpy(dtype=float)
    vals = vals[np.isfinite(vals)]
    if vals.size == 0:
        return float("nan")
    return float(np.median(vals))


def _max(arr: pd.Series) -> float:
    vals = pd.to_numeric(arr, errors="coerce").to_numpy(dtype=float)
    vals = vals[np.isfinite(vals)]
    if vals.size == 0:
        return float("nan")
    return float(np.max(vals))


def _min(arr: pd.Series) -> float:
    vals = pd.to_numeric(arr, errors="coerce").to_numpy(dtype=float)
    vals = vals[np.isfinite(vals)]
    if vals.size == 0:
        return float("nan")
    return float(np.min(vals))


def _build_row(
    gps_path: Path,
    *,
    axis_frame: Dict[str, float],
    gps_smooth_window_s: float,
) -> Dict[str, float | int | str]:
    scenario, run_id = _scenario_key(gps_path.stem)
    test_name = gps_path.stem.replace("gps_truth_", "")

    gps_df = ard.load_gps_truth(str(gps_path))
    t, x, y = _world_xy_from_gps(
        gps_df,
        axis_frame=axis_frame,
        gps_smooth_window_s=float(gps_smooth_window_s),
    )

    displacement_m = float(np.hypot(x[-1] - x[0], y[-1] - y[0])) if x.size >= 2 else float("nan")
    duration_s = float(t[-1] - t[0]) if t.size >= 2 else float("nan")

    row: Dict[str, float | int | str] = {
        "scenario": scenario,
        "run_id": int(run_id),
        "test_name": test_name,
        "gps_file": str(gps_path),
        "gps_sample_count": int(len(gps_df)),
        "start_x_world_m": float(x[0]),
        "start_y_world_m": float(y[0]),
        "end_x_world_m": float(x[-1]),
        "end_y_world_m": float(y[-1]),
        "displacement_world_m": displacement_m,
        "duration_s": duration_s,
        "fix_type_min": _min(gps_df["fix_type"]) if "fix_type" in gps_df.columns else float("nan"),
        "fix_type_max": _max(gps_df["fix_type"]) if "fix_type" in gps_df.columns else float("nan"),
        "h_acc_m_median": _median(gps_df["h_acc_m"]) if "h_acc_m" in gps_df.columns else float("nan"),
        "h_acc_m_max": _max(gps_df["h_acc_m"]) if "h_acc_m" in gps_df.columns else float("nan"),
        "v_acc_m_median": _median(gps_df["v_acc_m"]) if "v_acc_m" in gps_df.columns else float("nan"),
        "v_acc_m_max": _max(gps_df["v_acc_m"]) if "v_acc_m" in gps_df.columns else float("nan"),
        "p_dop_median": _median(gps_df["p_dop"]) if "p_dop" in gps_df.columns else float("nan"),
        "p_dop_max": _max(gps_df["p_dop"]) if "p_dop" in gps_df.columns else float("nan"),
        "num_sv_median": _median(gps_df["num_sv"]) if "num_sv" in gps_df.columns else float("nan"),
        "num_sv_min": _min(gps_df["num_sv"]) if "num_sv" in gps_df.columns else float("nan"),
        "head_acc_deg_median": _median(gps_df["head_acc_deg"]) if "head_acc_deg" in gps_df.columns else float("nan"),
        "head_acc_deg_max": _max(gps_df["head_acc_deg"]) if "head_acc_deg" in gps_df.columns else float("nan"),
    }
    return row


def _add_scenario_consistency_columns(
    run_df: pd.DataFrame,
    *,
    start_x_thresh_m: float,
    start_y_thresh_m: float,
    end_x_thresh_m: float,
    end_y_thresh_m: float,
    displacement_thresh_m: float,
    h_acc_thresh_m: float,
    pdop_thresh: float,
    min_fix_type: int,
    min_satellites: int,
) -> Tuple[pd.DataFrame, pd.DataFrame]:
    df = run_df.copy()

    scenario_summary_rows: List[Dict[str, float | int | str]] = []
    for scenario, grp in df.groupby("scenario", sort=True):
        start_x_med = float(np.median(grp["start_x_world_m"]))
        start_y_med = float(np.median(grp["start_y_world_m"]))
        end_x_med = float(np.median(grp["end_x_world_m"]))
        end_y_med = float(np.median(grp["end_y_world_m"]))
        disp_med = float(np.median(grp["displacement_world_m"]))

        mask = df["scenario"] == scenario
        df.loc[mask, "scenario_start_x_median_m"] = start_x_med
        df.loc[mask, "scenario_start_y_median_m"] = start_y_med
        df.loc[mask, "scenario_end_x_median_m"] = end_x_med
        df.loc[mask, "scenario_end_y_median_m"] = end_y_med
        df.loc[mask, "scenario_displacement_median_m"] = disp_med
        df.loc[mask, "start_x_delta_m"] = df.loc[mask, "start_x_world_m"] - start_x_med
        df.loc[mask, "start_y_delta_m"] = df.loc[mask, "start_y_world_m"] - start_y_med
        df.loc[mask, "end_x_delta_m"] = df.loc[mask, "end_x_world_m"] - end_x_med
        df.loc[mask, "end_y_delta_m"] = df.loc[mask, "end_y_world_m"] - end_y_med
        df.loc[mask, "displacement_delta_m"] = df.loc[mask, "displacement_world_m"] - disp_med

        scenario_summary_rows.append(
            {
                "scenario": scenario,
                "run_count": int(len(grp)),
                "start_x_median_m": start_x_med,
                "start_y_median_m": start_y_med,
                "end_x_median_m": end_x_med,
                "end_y_median_m": end_y_med,
                "displacement_median_m": disp_med,
            }
        )

    flags: List[List[str]] = []
    for row in df.itertuples(index=False):
        row_flags: List[str] = []
        if np.isfinite(row.fix_type_min) and float(row.fix_type_min) < float(min_fix_type):
            row_flags.append(f"fix_type_min<{min_fix_type}")
        if np.isfinite(row.h_acc_m_max) and float(row.h_acc_m_max) > float(h_acc_thresh_m):
            row_flags.append(f"h_acc_max>{h_acc_thresh_m:.3f}m")
        if np.isfinite(row.p_dop_max) and float(row.p_dop_max) > float(pdop_thresh):
            row_flags.append(f"p_dop_max>{pdop_thresh:.2f}")
        if np.isfinite(row.num_sv_min) and float(row.num_sv_min) < float(min_satellites):
            row_flags.append(f"num_sv_min<{min_satellites}")
        if np.isfinite(row.start_x_delta_m) and abs(float(row.start_x_delta_m)) > float(start_x_thresh_m):
            row_flags.append(f"start_x_delta>{start_x_thresh_m:.2f}m")
        if np.isfinite(row.start_y_delta_m) and abs(float(row.start_y_delta_m)) > float(start_y_thresh_m):
            row_flags.append(f"start_y_delta>{start_y_thresh_m:.2f}m")
        if np.isfinite(row.end_x_delta_m) and abs(float(row.end_x_delta_m)) > float(end_x_thresh_m):
            row_flags.append(f"end_x_delta>{end_x_thresh_m:.2f}m")
        if np.isfinite(row.end_y_delta_m) and abs(float(row.end_y_delta_m)) > float(end_y_thresh_m):
            row_flags.append(f"end_y_delta>{end_y_thresh_m:.2f}m")
        if np.isfinite(row.displacement_delta_m) and abs(float(row.displacement_delta_m)) > float(displacement_thresh_m):
            row_flags.append(f"displacement_delta>{displacement_thresh_m:.2f}m")
        flags.append(row_flags)

    df["flag_count"] = [len(x) for x in flags]
    df["flag_reasons"] = [";".join(x) for x in flags]
    df["is_flagged"] = df["flag_count"] > 0

    scenario_summary_df = pd.DataFrame(scenario_summary_rows).sort_values("scenario").reset_index(drop=True)
    return df.sort_values(["scenario", "run_id"]).reset_index(drop=True), scenario_summary_df


def _build_summary_text(
    run_df: pd.DataFrame,
    *,
    axis_gps_path: Path | None,
    out_prefix: Path,
    start_x_thresh_m: float,
    start_y_thresh_m: float,
    end_x_thresh_m: float,
    end_y_thresh_m: float,
    displacement_thresh_m: float,
    h_acc_thresh_m: float,
    pdop_thresh: float,
    min_fix_type: int,
    min_satellites: int,
) -> str:
    lines: List[str] = []
    lines.append("Replay GPS sanity check")
    lines.append("=======================")
    lines.append("")
    lines.append("Flagging rules:")
    lines.append(f"  fix_type_min < {min_fix_type}")
    lines.append(f"  h_acc_m_max > {h_acc_thresh_m:.3f} m")
    lines.append(f"  p_dop_max > {pdop_thresh:.2f}")
    lines.append(f"  num_sv_min < {min_satellites}")
    lines.append(f"  |start_x_delta| > {start_x_thresh_m:.2f} m")
    lines.append(f"  |start_y_delta| > {start_y_thresh_m:.2f} m")
    lines.append(f"  |end_x_delta| > {end_x_thresh_m:.2f} m")
    lines.append(f"  |end_y_delta| > {end_y_thresh_m:.2f} m")
    lines.append(f"  |displacement_delta| > {displacement_thresh_m:.2f} m")
    lines.append("")
    lines.append(f"Axis GPS file: {axis_gps_path if axis_gps_path is not None else '(run-local axis)'}")
    lines.append(f"Run-level CSV: {out_prefix}_run_level.csv")
    lines.append(f"Scenario CSV: {out_prefix}_scenario_summary.csv")
    lines.append("")

    flagged = run_df[run_df["is_flagged"]].copy()
    lines.append(f"Flagged runs: {len(flagged)}/{len(run_df)}")
    lines.append("")

    for scenario, grp in run_df.groupby("scenario", sort=True):
        lines.append(f"[{scenario}]")
        lines.append(
            "  scenario medians: "
            f"start=({grp['scenario_start_x_median_m'].iloc[0]:.2f}, {grp['scenario_start_y_median_m'].iloc[0]:.2f}) m, "
            f"end=({grp['scenario_end_x_median_m'].iloc[0]:.2f}, {grp['scenario_end_y_median_m'].iloc[0]:.2f}) m, "
            f"disp={grp['scenario_displacement_median_m'].iloc[0]:.2f} m"
        )
        flagged_grp = grp[grp["is_flagged"]]
        if flagged_grp.empty:
            lines.append("  no flagged runs")
        else:
            for row in flagged_grp.itertuples(index=False):
                lines.append(
                    "  "
                    f"{row.test_name}: start=({row.start_x_world_m:.2f}, {row.start_y_world_m:.2f}) m, "
                    f"end=({row.end_x_world_m:.2f}, {row.end_y_world_m:.2f}) m, "
                    f"disp={row.displacement_world_m:.2f} m, reasons={row.flag_reasons}"
                )
        lines.append("")
    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-dir", default="real_world_logs", help="Directory containing gps_truth_*.csv files")
    parser.add_argument(
        "--axis-gps-file",
        default=None,
        help=(
            "Optional GPS file used to define a shared world-frame axis. "
            f"Default: <log-dir>/{DEFAULT_AXIS_FILE} if it exists."
        ),
    )
    parser.add_argument("--gps-smooth-window-s", type=float, default=0.0, help="Optional smoothing window before consistency checks")
    parser.add_argument("--out-prefix", default="plots/replay_gps_sanity", help="Output prefix for CSV/TXT results")
    parser.add_argument("--start-x-threshold-m", type=float, default=0.75, help="Max allowed deviation from scenario-median start x")
    parser.add_argument("--start-y-threshold-m", type=float, default=0.50, help="Max allowed deviation from scenario-median start y")
    parser.add_argument("--end-x-threshold-m", type=float, default=1.00, help="Max allowed deviation from scenario-median end x")
    parser.add_argument("--end-y-threshold-m", type=float, default=1.00, help="Max allowed deviation from scenario-median end y")
    parser.add_argument("--displacement-threshold-m", type=float, default=1.00, help="Max allowed deviation from scenario-median displacement")
    parser.add_argument("--h-acc-threshold-m", type=float, default=0.05, help="Max acceptable reported horizontal accuracy")
    parser.add_argument("--pdop-threshold", type=float, default=3.0, help="Max acceptable PDOP")
    parser.add_argument("--min-fix-type", type=int, default=3, help="Minimum acceptable fix_type")
    parser.add_argument("--min-satellites", type=int, default=6, help="Minimum acceptable satellite count")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    log_dir = Path(str(args.log_dir))
    if not log_dir.exists():
        raise FileNotFoundError(f"--log-dir not found: {log_dir}")

    gps_files = sorted(log_dir.glob(GPS_GLOB))
    if not gps_files:
        raise FileNotFoundError(f"No files matching {GPS_GLOB!r} under {log_dir}")

    if args.axis_gps_file is not None:
        axis_gps_path = Path(str(args.axis_gps_file))
    else:
        candidate = log_dir / DEFAULT_AXIS_FILE
        axis_gps_path = candidate if candidate.exists() else gps_files[0]

    axis_gps_df = ard.load_gps_truth(str(axis_gps_path))
    axis_frame = _build_axis_frame(axis_gps_df)

    rows = [
        _build_row(
            p,
            axis_frame=axis_frame,
            gps_smooth_window_s=float(args.gps_smooth_window_s),
        )
        for p in gps_files
    ]
    run_df = pd.DataFrame(rows).sort_values(["scenario", "run_id"]).reset_index(drop=True)
    run_df, scenario_summary_df = _add_scenario_consistency_columns(
        run_df,
        start_x_thresh_m=float(args.start_x_threshold_m),
        start_y_thresh_m=float(args.start_y_threshold_m),
        end_x_thresh_m=float(args.end_x_threshold_m),
        end_y_thresh_m=float(args.end_y_threshold_m),
        displacement_thresh_m=float(args.displacement_threshold_m),
        h_acc_thresh_m=float(args.h_acc_threshold_m),
        pdop_thresh=float(args.pdop_threshold),
        min_fix_type=int(args.min_fix_type),
        min_satellites=int(args.min_satellites),
    )

    out_prefix = Path(str(args.out_prefix))
    out_prefix.parent.mkdir(parents=True, exist_ok=True)
    run_csv = Path(str(out_prefix) + "_run_level.csv")
    scenario_csv = Path(str(out_prefix) + "_scenario_summary.csv")
    summary_txt = Path(str(out_prefix) + "_summary.txt")

    run_df.to_csv(run_csv, index=False)
    scenario_summary_df.to_csv(scenario_csv, index=False)

    summary = _build_summary_text(
        run_df,
        axis_gps_path=axis_gps_path,
        out_prefix=out_prefix,
        start_x_thresh_m=float(args.start_x_threshold_m),
        start_y_thresh_m=float(args.start_y_threshold_m),
        end_x_thresh_m=float(args.end_x_threshold_m),
        end_y_thresh_m=float(args.end_y_threshold_m),
        displacement_thresh_m=float(args.displacement_threshold_m),
        h_acc_thresh_m=float(args.h_acc_threshold_m),
        pdop_thresh=float(args.pdop_threshold),
        min_fix_type=int(args.min_fix_type),
        min_satellites=int(args.min_satellites),
    )
    summary_txt.write_text(summary + "\n", encoding="utf-8")

    print(summary)
    print(f"Wrote: {run_csv}")
    print(f"Wrote: {scenario_csv}")
    print(f"Wrote: {summary_txt}")


if __name__ == "__main__":
    main()