#!/usr/bin/env python3
"""visualize_real_world_detections.py

Visualize radar, camera and fused detections from real-world CSV logs.

Inputs (real_world_logger outputs; default under testing_tools/real_world_logs):
  - raw_detections_<test_name>.csv
  - fused_detections_<test_name>.csv
  - (optional) tracks_<test_name>.csv

This script focuses on visualization (no metrics):
  - "static" mode: scatter all detections
  - "playback" mode: interactive time slider + windowed scatter

Coordinate conventions:
  - Positions are assumed to be in tractor body frame after applying the same
    transforms as analysis_tools/analyze_scenario.py.
  - By default, plots match analyze_scenario convention: x-axis is y [m],
    y-axis is x [m] (so "forward" is up).

Examples:
  - ./visualize_real_world_detections.py --test-name t1 --mode playback
  - ./visualize_real_world_detections.py --test-name t1 --mode static --only-fused
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import numpy as np
except ModuleNotFoundError as exc:  # pragma: no cover
    raise SystemExit(
        "Missing dependency: numpy. Install analysis deps: "
        "`pip install -r testing_tools/analysis_tools/requirements.txt`"
    ) from exc

try:
    import pandas as pd
except ModuleNotFoundError as exc:  # pragma: no cover
    raise SystemExit(
        "Missing dependency: pandas. Install analysis deps: "
        "`pip install -r testing_tools/analysis_tools/requirements.txt`"
    ) from exc

try:
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider
except ModuleNotFoundError as exc:  # pragma: no cover
    raise SystemExit(
        "Missing dependency: matplotlib. Install analysis deps: "
        "`pip install -r testing_tools/analysis_tools/requirements.txt`"
    ) from exc

from analyze_scenario import load_fused, load_raw_detections, load_tracks  # type: ignore


def _build_paths(log_dir: Path, test_name: str) -> Dict[str, Path]:
    return {
        "raw": log_dir / f"raw_detections_{test_name}.csv",
        "fused": log_dir / f"fused_detections_{test_name}.csv",
        "tracks": log_dir / f"tracks_{test_name}.csv",
    }


def _discover_test_names(log_dir: Path) -> List[str]:
    """Return sorted test names that have at least raw+fused logs."""
    raw = {p.name.removeprefix("raw_detections_").removesuffix(".csv") for p in log_dir.glob("raw_detections_*.csv")}
    fused = {p.name.removeprefix("fused_detections_").removesuffix(".csv") for p in log_dir.glob("fused_detections_*.csv")}
    names = sorted(raw & fused)
    return names


def _prompt(text: str, default: Optional[str] = None) -> str:
    if default is None:
        suffix = ": "
    else:
        suffix = f" [{default}]: "
    value = input(text + suffix).strip()
    return value if value else (default or "")


def _prompt_choice(text: str, choices: Sequence[str], default_index: int = 0) -> str:
    if not choices:
        raise ValueError("choices must be non-empty")
    default_index = max(0, min(default_index, len(choices) - 1))
    while True:
        print(text)
        for i, c in enumerate(choices, start=1):
            d = " (default)" if (i - 1) == default_index else ""
            print(f"  {i}. {c}{d}")
        raw = input("Select option number: ").strip()
        if not raw:
            return choices[default_index]
        if raw.isdigit():
            idx = int(raw) - 1
            if 0 <= idx < len(choices):
                return choices[idx]
        print("Invalid selection. Try again.\n")


def _resolve_view_option(option: str) -> Tuple[bool, bool, bool, bool]:
    """Return (show_raw, show_fused, show_tracks, only_real_fused)."""
    mapping = {
        "1. plot only raw detections": (True, False, False, False),
        "2. plot fused and raw detections": (True, True, False, False),
        "3. plot tracks and fused detections": (False, True, True, False),
        "4. plot real fused detections (no single sensor fused)": (False, True, False, True),
        "5. plot real fused and raw detections": (True, True, False, True),
        "6. plot all": (True, True, True, False),
    }
    if option not in mapping:
        raise ValueError(f"Unknown option: {option}")
    return mapping[option]


def _get_time_col(df: pd.DataFrame, time_source: str) -> str:
    if time_source not in ("header", "logger"):
        raise ValueError("time_source must be 'header' or 'logger'")

    col = "header_t" if time_source == "header" else "logger_t"
    if col not in df.columns:
        raise ValueError(
            f"Expected time column '{col}' in dataframe. "
            "(Did you pass the right log file?)"
        )
    return col


def _ensure_sorted(df: pd.DataFrame, time_col: str) -> pd.DataFrame:
    if df.empty:
        return df
    return df.sort_values(time_col)


def _mask_window(times: np.ndarray, t_center: float, half_window: float) -> np.ndarray:
    if times.size == 0:
        return np.zeros((0,), dtype=bool)
    t0 = t_center - half_window
    t1 = t_center + half_window
    return (times >= t0) & (times <= t1)


def _xy_for_plot(df: pd.DataFrame, swap_axes: bool) -> np.ndarray:
    if df.empty:
        return np.zeros((0, 2), dtype=float)
    x = df["x"].to_numpy(dtype=float)
    y = df["y"].to_numpy(dtype=float)

    # Matplotlib scatter expects (x_plot, y_plot)
    if swap_axes:
        return np.column_stack([y, x])
    return np.column_stack([x, y])


def _limits_for_scenario(scenario: Optional[str], swap_axes: bool) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Return ((xlim0,xlim1),(ylim0,ylim1)) in plot coordinates."""
    if not scenario:
        return None

    # Keep this intentionally small/safe: only mirror the existing S1 choice.
    if scenario == "S1":
        # Keep a more usable default than analyze_scenario's very narrow lateral window.
        # When swap_axes=True: plot x-axis is y (lateral), y-axis is x (forward).
        if swap_axes:
            return ((-10.0, 10.0), (-5.0, 35.0))
        # If not swapping, treat x forward on x-axis with a wider view
        return ((-5.0, 35.0), (-10.0, 10.0))

    return None


def _limits_from_data(
    dfs: Iterable[pd.DataFrame],
    *,
    swap_axes: bool,
    min_span_lateral: float = 20.0,
    min_span_forward: float = 30.0,
) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Auto-scale limits from plotted points with minimum spans.

    When swap_axes=True, lateral axis is x-axis (y [m]) and forward is y-axis (x [m]).
    """

    points_all: list[np.ndarray] = []
    for df in dfs:
        if df is None or df.empty:
            continue
        pts = _xy_for_plot(df, swap_axes)
        if pts.size:
            points_all.append(pts)

    if not points_all:
        return None

    pts = np.vstack(points_all)
    x0, y0 = float(np.nanmin(pts[:, 0])), float(np.nanmin(pts[:, 1]))
    x1, y1 = float(np.nanmax(pts[:, 0])), float(np.nanmax(pts[:, 1]))
    if not (np.isfinite(x0) and np.isfinite(x1) and np.isfinite(y0) and np.isfinite(y1)):
        return None

    # Expand slightly for readability
    pad_x = max(0.5, 0.10 * (x1 - x0))
    pad_y = max(0.5, 0.10 * (y1 - y0))
    x0 -= pad_x
    x1 += pad_x
    y0 -= pad_y
    y1 += pad_y

    # Enforce minimum spans
    span_x = x1 - x0
    span_y = y1 - y0
    if swap_axes:
        min_span_x = float(min_span_lateral)
        min_span_y = float(min_span_forward)
    else:
        min_span_x = float(min_span_forward)
        min_span_y = float(min_span_lateral)

    if span_x < min_span_x:
        mid = 0.5 * (x0 + x1)
        x0, x1 = mid - 0.5 * min_span_x, mid + 0.5 * min_span_x
    if span_y < min_span_y:
        mid = 0.5 * (y0 + y1)
        y0, y1 = mid - 0.5 * min_span_y, mid + 0.5 * min_span_y

    return ((x0, x1), (y0, y1))


def _plot_static(
    radar_df: pd.DataFrame,
    camera_df: pd.DataFrame,
    fused_df: pd.DataFrame,
    tracks_df: Optional[pd.DataFrame],
    *,
    swap_axes: bool,
    only_fused: bool,
    show_tracks: bool,
    scenario: Optional[str],
    title: str,
) -> None:
    fig, ax = plt.subplots(figsize=(9, 8))

    if not radar_df.empty:
        pts = _xy_for_plot(radar_df, swap_axes)
        ax.scatter(pts[:, 0], pts[:, 1], s=6, alpha=0.35, c="#1f77b4", label="Raw radar")

    if not camera_df.empty:
        pts = _xy_for_plot(camera_df, swap_axes)
        ax.scatter(pts[:, 0], pts[:, 1], s=10, alpha=0.35, c="#2ca02c", label="Raw camera")

    if not fused_df.empty:
        if only_fused and "detection_type" in fused_df.columns:
            fused_df = fused_df[fused_df["detection_type"] == "fused"].copy()

        if "detection_type" in fused_df.columns:
            for det_type, color, marker, label in (
                ("fused", "#d62728", "o", "Fused"),
                ("radar", "#17becf", "x", "Fusion input (radar)"),
                ("camera", "#bcbd22", "x", "Fusion input (camera)"),
            ):
                sub = fused_df[fused_df["detection_type"] == det_type]
                if sub.empty:
                    continue
                pts = _xy_for_plot(sub, swap_axes)
                ax.scatter(pts[:, 0], pts[:, 1], s=18, alpha=0.55, c=color, marker=marker, label=label)
        else:
            pts = _xy_for_plot(fused_df, swap_axes)
            ax.scatter(pts[:, 0], pts[:, 1], s=18, alpha=0.55, c="#d62728", label="Fused")

    if show_tracks and tracks_df is not None and not tracks_df.empty:
        pts = _xy_for_plot(tracks_df, swap_axes)
        ax.scatter(pts[:, 0], pts[:, 1], s=22, alpha=0.8, c="#9467bd", marker=".", label="Tracks")

    if swap_axes:
        ax.set_xlabel("y [m]")
        ax.set_ylabel("x [m]")
    else:
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")

    ax.set_title(title)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)

    lims = _limits_for_scenario(scenario, swap_axes)
    if not lims:
        lims = _limits_from_data(
            [radar_df, camera_df, fused_df] + ([tracks_df] if tracks_df is not None else []),
            swap_axes=swap_axes,
        )
    if lims:
        ax.set_xlim(*lims[0])
        ax.set_ylim(*lims[1])

    ax.legend(loc="best")
    plt.tight_layout()

    out_path = Path("plots") / "real_world" / (title.replace(" ", "_") + "_static.png")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=250)
    print(f"Saved figure: {out_path}")
    plt.show()


def _plot_playback(
    radar_df: pd.DataFrame,
    camera_df: pd.DataFrame,
    fused_df: pd.DataFrame,
    tracks_df: Optional[pd.DataFrame],
    *,
    swap_axes: bool,
    only_fused: bool,
    show_tracks: bool,
    scenario: Optional[str],
    time_source: str,
    window_s: float,
    title: str,
) -> None:
    # Pick a common time range across datasets that are present
    all_times: list[np.ndarray] = []
    for df in (radar_df, camera_df, fused_df):
        if df is None or df.empty:
            continue
        tc = _get_time_col(df, time_source)
        all_times.append(df[tc].to_numpy(dtype=float))
    if show_tracks and tracks_df is not None and not tracks_df.empty:
        tc = _get_time_col(tracks_df, time_source)
        all_times.append(tracks_df[tc].to_numpy(dtype=float))

    if not all_times:
        raise SystemExit("No detections to plot (all inputs empty).")

    t_min = float(min(t.min() for t in all_times if t.size > 0))
    t_max = float(max(t.max() for t in all_times if t.size > 0))
    if not np.isfinite(t_min) or not np.isfinite(t_max) or t_max <= t_min:
        raise SystemExit("Invalid time range in logs.")

    half_window = max(0.0, window_s * 0.5)

    # Pre-extract time arrays for speed
    radar_tcol = _get_time_col(radar_df, time_source) if not radar_df.empty else None
    camera_tcol = _get_time_col(camera_df, time_source) if not camera_df.empty else None
    fused_tcol = _get_time_col(fused_df, time_source) if not fused_df.empty else None
    tracks_tcol = _get_time_col(tracks_df, time_source) if (show_tracks and tracks_df is not None and not tracks_df.empty) else None

    radar_times = radar_df[radar_tcol].to_numpy(dtype=float) if radar_tcol else np.array([])
    camera_times = camera_df[camera_tcol].to_numpy(dtype=float) if camera_tcol else np.array([])
    fused_times = fused_df[fused_tcol].to_numpy(dtype=float) if fused_tcol else np.array([])
    tracks_times = tracks_df[tracks_tcol].to_numpy(dtype=float) if tracks_tcol else np.array([])

    fig, ax = plt.subplots(figsize=(9, 8))
    plt.subplots_adjust(bottom=0.18)

    # Create initial scatters with empty data, then update via slider.
    s_radar = ax.scatter([], [], s=18, alpha=0.55, c="#1f77b4", label="Raw radar")
    s_camera = ax.scatter([], [], s=22, alpha=0.55, c="#2ca02c", label="Raw camera")

    # Fused: allow grouping by detection_type, but keep it simple for playback.
    fused_df_plot = fused_df
    if only_fused and "detection_type" in fused_df_plot.columns:
        fused_df_plot = fused_df_plot[fused_df_plot["detection_type"] == "fused"].copy()
        fused_times_plot = fused_df_plot[fused_tcol].to_numpy(dtype=float) if fused_tcol else np.array([])
    else:
        fused_times_plot = fused_times

    s_fused = ax.scatter([], [], s=28, alpha=0.75, c="#d62728", label="Fused")

    s_tracks = None
    if show_tracks and tracks_df is not None:
        s_tracks = ax.scatter([], [], s=28, alpha=0.85, c="#9467bd", marker=".", label="Tracks")

    # Labels/limits
    if swap_axes:
        ax.set_xlabel("y [m]")
        ax.set_ylabel("x [m]")
    else:
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")

    ax.set_title(title)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)

    lims = _limits_for_scenario(scenario, swap_axes)
    if not lims:
        lims = _limits_from_data(
            [radar_df, camera_df, fused_df] + ([tracks_df] if tracks_df is not None else []),
            swap_axes=swap_axes,
        )
    if lims:
        ax.set_xlim(*lims[0])
        ax.set_ylim(*lims[1])

    ax.legend(loc="best")

    # Slider
    ax_slider = plt.axes([0.12, 0.06, 0.78, 0.04])
    slider = Slider(
        ax=ax_slider,
        label=f"t ({time_source})",
        valmin=t_min,
        valmax=t_max,
        valinit=t_min,
        valstep=max(0.01, (t_max - t_min) / 500.0),
    )

    info_text = ax.text(
        0.01,
        0.99,
        "",
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=9,
        bbox=dict(facecolor="white", alpha=0.7, edgecolor="none"),
    )

    def update(t_center: float) -> None:
        t_center_f = float(t_center)

        if radar_times.size:
            m = _mask_window(radar_times, t_center_f, half_window)
            pts = _xy_for_plot(radar_df.loc[m], swap_axes)
            s_radar.set_offsets(pts)
        else:
            s_radar.set_offsets(np.zeros((0, 2)))

        if camera_times.size:
            m = _mask_window(camera_times, t_center_f, half_window)
            pts = _xy_for_plot(camera_df.loc[m], swap_axes)
            s_camera.set_offsets(pts)
        else:
            s_camera.set_offsets(np.zeros((0, 2)))

        if fused_times_plot.size:
            m = _mask_window(fused_times_plot, t_center_f, half_window)
            pts = _xy_for_plot(fused_df_plot.loc[m], swap_axes)
            s_fused.set_offsets(pts)
        else:
            s_fused.set_offsets(np.zeros((0, 2)))

        if s_tracks is not None and tracks_times.size and tracks_df is not None:
            m = _mask_window(tracks_times, t_center_f, half_window)
            pts = _xy_for_plot(tracks_df.loc[m], swap_axes)
            s_tracks.set_offsets(pts)

        n_r = int(np.count_nonzero(_mask_window(radar_times, t_center_f, half_window))) if radar_times.size else 0
        n_c = int(np.count_nonzero(_mask_window(camera_times, t_center_f, half_window))) if camera_times.size else 0
        n_f = int(np.count_nonzero(_mask_window(fused_times_plot, t_center_f, half_window))) if fused_times_plot.size else 0
        n_t = int(np.count_nonzero(_mask_window(tracks_times, t_center_f, half_window))) if tracks_times.size else 0

        info_text.set_text(
            f"t={t_center_f:.3f}s  window={window_s:.2f}s\n"
            f"raw radar={n_r}  raw camera={n_c}  fused={n_f}" + (f"  tracks={n_t}" if s_tracks is not None else "")
        )

        fig.canvas.draw_idle()

    slider.on_changed(update)
    update(t_min)

    out_path = Path("plots") / "real_world" / (title.replace(" ", "_") + "_playback.png")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=250)
    print(f"Saved figure: {out_path}")

    plt.show()

    # Save again after the interactive window closes (captures last slider position)
    fig.savefig(out_path, dpi=250)
    print(f"Saved figure: {out_path}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--test-name",
        default=None,
        help="Test name, e.g. t1 (loads raw_detections_t1.csv, fused_detections_t1.csv). If omitted, an interactive prompt is shown.",
    )
    parser.add_argument("--log-dir", default="real_world_logs", help="Directory containing the CSV logs")

    parser.add_argument("--mode", choices=["static", "playback"], default="playback")
    parser.add_argument("--time-source", choices=["header", "logger"], default="header")
    parser.add_argument("--window-s", type=float, default=0.25, help="Playback window size in seconds")

    parser.add_argument("--scenario", default="", help="Optional scenario name (used for axis presets, e.g. S1)")
    parser.add_argument("--swap-axes", action="store_true", default=True, help="Plot y on x-axis and x on y-axis (matches analyze_scenario plots)")
    parser.add_argument("--no-swap-axes", dest="swap_axes", action="store_false", help="Plot x on x-axis and y on y-axis")

    parser.add_argument("--only-fused", action="store_true", help="Only show detection_type=='fused' from fused_detections CSV")
    parser.add_argument("--show-tracks", action="store_true", help="Also load/plot tracks_<test_name>.csv if present")
    parser.add_argument(
        "--view",
        default=None,
        choices=[
            "raw",
            "raw+fused",
            "raw+real_fused",
            "tracks+fused",
            "real_fused",
            "all",
        ],
        help="Quick view preset. If omitted and --test-name is omitted, you will be prompted.",
    )

    args = parser.parse_args()

    log_dir = Path(args.log_dir)

    # Interactive wizard if test_name not provided.
    if args.test_name is None:
        print("Real-world detection visualizer")
        log_dir_in = _prompt("Log directory", str(log_dir))
        log_dir = Path(log_dir_in)
        if not log_dir.exists():
            raise FileNotFoundError(f"Log directory does not exist: {log_dir}")

        names = _discover_test_names(log_dir)
        if not names:
            raise FileNotFoundError(
                f"No tests found in {log_dir}. Expected files like raw_detections_<test>.csv and fused_detections_<test>.csv"
            )
        args.test_name = _prompt_choice("Select test", names, default_index=max(0, len(names) - 1))

        scenario_in = _prompt("Scenario (optional, affects axis presets)", "S1")
        args.scenario = scenario_in

        view_labels = [
            "1. plot only raw detections",
            "2. plot fused and raw detections",
            "3. plot tracks and fused detections",
            "4. plot real fused detections (no single sensor fused)",
            "5. plot real fused and raw detections",
            "6. plot all",
        ]
        view_selected = _prompt_choice("Choose what to plot", view_labels, default_index=1)
        show_raw, show_fused, show_tracks, only_real_fused = _resolve_view_option(view_selected)

        mode_in = _prompt_choice("Plot mode", ["playback", "static"], default_index=0)
        args.mode = mode_in

        if args.mode == "playback":
            args.time_source = _prompt_choice("Time source", ["header", "logger"], default_index=0)
            window_in = _prompt("Window size seconds", str(args.window_s))
            try:
                args.window_s = float(window_in)
            except ValueError:
                print("Invalid window, using default.")

        # Apply view to flags.
        args.show_tracks = bool(show_tracks)
        args.only_fused = bool(only_real_fused)

        # If user chose not to show fused at all, we will later pass an empty df.
        # If user chose not to show raw at all, we will later pass empty dfs.
        args._show_raw = bool(show_raw)  # type: ignore[attr-defined]
        args._show_fused = bool(show_fused)  # type: ignore[attr-defined]
    else:
        # Non-interactive: translate --view presets (optional)
        show_raw = True
        show_fused = True
        show_tracks = bool(args.show_tracks)
        only_real_fused = bool(args.only_fused)

        if args.view == "raw":
            show_raw, show_fused, show_tracks, only_real_fused = True, False, False, False
        elif args.view == "raw+fused":
            show_raw, show_fused, show_tracks, only_real_fused = True, True, False, False
        elif args.view == "raw+real_fused":
            show_raw, show_fused, show_tracks, only_real_fused = True, True, False, True
        elif args.view == "tracks+fused":
            show_raw, show_fused, show_tracks, only_real_fused = False, True, True, False
        elif args.view == "real_fused":
            show_raw, show_fused, show_tracks, only_real_fused = False, True, False, True
        elif args.view == "all":
            show_raw, show_fused, show_tracks, only_real_fused = True, True, True, False

        args.show_tracks = bool(show_tracks)
        args.only_fused = bool(only_real_fused)
        args._show_raw = bool(show_raw)  # type: ignore[attr-defined]
        args._show_fused = bool(show_fused)  # type: ignore[attr-defined]

    assert args.test_name is not None
    paths = _build_paths(log_dir, args.test_name)

    if not paths["raw"].exists():
        raise FileNotFoundError(f"Missing file: {paths['raw']}")
    if not paths["fused"].exists():
        raise FileNotFoundError(f"Missing file: {paths['fused']}")

    if getattr(args, "_show_raw", True):
        radar_df, camera_df = load_raw_detections(str(paths["raw"]))
    else:
        radar_df, camera_df = pd.DataFrame(), pd.DataFrame()

    if getattr(args, "_show_fused", True):
        fused_df = load_fused(str(paths["fused"]))
    else:
        fused_df = pd.DataFrame()

    tracks_df: Optional[pd.DataFrame] = None
    if args.show_tracks and paths["tracks"].exists():
        tracks_df = load_tracks(str(paths["tracks"]))

    # Standardize time sorting for slider performance
    for df_name, df in (
        ("radar", radar_df),
        ("camera", camera_df),
        ("fused", fused_df),
    ):
        if df.empty:
            continue
        tcol = _get_time_col(df, args.time_source)
        if tcol not in df.columns:
            raise ValueError(f"{df_name} df missing time column {tcol}")

    radar_df = _ensure_sorted(radar_df, _get_time_col(radar_df, args.time_source)) if not radar_df.empty else radar_df
    camera_df = _ensure_sorted(camera_df, _get_time_col(camera_df, args.time_source)) if not camera_df.empty else camera_df
    fused_df = _ensure_sorted(fused_df, _get_time_col(fused_df, args.time_source)) if not fused_df.empty else fused_df
    if tracks_df is not None and not tracks_df.empty:
        tracks_df = _ensure_sorted(tracks_df, _get_time_col(tracks_df, args.time_source))

    if not fused_df.empty and "detection_type" in fused_df.columns:
        counts = fused_df["detection_type"].astype(str).str.strip().str.casefold().value_counts()
        fused_n = int(counts.get("fused", 0))
        total_n = int(len(fused_df))
        if args.only_fused:
            print(
                f"Fused log contains {total_n} rows: "
                + ", ".join([f"{k}={int(v)}" for k, v in counts.items()])
            )
            print(
                f"NOTE: --only-fused/--view real_fused will plot only detection_type=='fused' "
                f"({fused_n}/{total_n})."
            )

    scenario = args.scenario.strip() or None
    title = f"Real-world detections: {args.test_name}" + (f" ({scenario})" if scenario else "")

    if args.mode == "static":
        _plot_static(
            radar_df,
            camera_df,
            fused_df,
            tracks_df,
            swap_axes=args.swap_axes,
            only_fused=args.only_fused,
            show_tracks=args.show_tracks,
            scenario=scenario,
            title=title,
        )
    else:
        _plot_playback(
            radar_df,
            camera_df,
            fused_df,
            tracks_df,
            swap_axes=args.swap_axes,
            only_fused=args.only_fused,
            show_tracks=args.show_tracks,
            scenario=scenario,
            time_source=args.time_source,
            window_s=float(args.window_s),
            title=title,
        )


if __name__ == "__main__":
    main()
