from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys
import time
from typing import Any

import numpy as np

if __package__ in {None, ""}:
    package_root = Path(__file__).resolve().parents[1]
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))

    from uwb_processing.animate_aoa import save_aoa_animation
    from uwb_processing.annotations import load_annotation_file
    from uwb_processing.aoa import annotate_detections_aoa, build_range_angle_map, estimate_track_aoa_session, pair_rx_channels
    from uwb_processing.breathing import run_breathing_extraction
    from uwb_processing.cfar import annotate_detections_with_doppler, combine_paths_max, merge_dual_channel_peaks, run_cfar_rd, run_cfar_session
    from uwb_processing.detection import detect_window
    from uwb_processing.loaders import load_session
    from uwb_processing.plotting import (
        save_doppler_animation,
        save_fov_animation,
        save_fov_polar_plot,
        save_multi_peak_tracking_plot,
        save_range_doppler_heatmap,
        save_range_time_plot,
        save_spectrogram_plot,
    )
    from uwb_processing.preprocessing import preprocess_session
    from uwb_processing.tracker import MultiTargetTracker
    from uwb_processing.types import CfarDetectionConfig, DetectionConfig, Track, TrackingConfig
else:
    from .animate_aoa import save_aoa_animation
    from .annotations import load_annotation_file
    from .aoa import annotate_detections_aoa, build_range_angle_map, estimate_track_aoa_session, pair_rx_channels
    from .breathing import run_breathing_extraction
    from .cfar import annotate_detections_with_doppler, combine_paths_max, merge_dual_channel_peaks, run_cfar_rd, run_cfar_session
    from .detection import detect_window
    from .loaders import load_session
    from .plotting import (
        save_doppler_animation,
        save_fov_animation,
        save_fov_polar_plot,
        save_multi_peak_tracking_plot,
        save_range_doppler_heatmap,
        save_range_time_plot,
        save_spectrogram_plot,
    )
    from .preprocessing import preprocess_session
    from .tracker import MultiTargetTracker
    from .types import CfarDetectionConfig, DetectionConfig, Track, TrackingConfig


_REPO_ROOT = Path(__file__).resolve().parents[2]
# DEFAULT_SESSION_NAME = "shubo_edison_two_static_walking_medium_2ms"
DEFAULT_SESSION_NAME = "single_person_walking_preset_2ms"


# ---------------------------------------------------------------------------
# CLI helpers
# ---------------------------------------------------------------------------

def _default_input_path(all_rosbags: bool = False) -> Path:
    if all_rosbags:
        return _REPO_ROOT / "uwb_rosbags"
    return _REPO_ROOT / "uwb_rosbags" / DEFAULT_SESSION_NAME

def _discover_session_inputs(path: Path, all_rosbags: bool = False) -> list[Path]:
    """Return [path] unless all_rosbags=True, in which case scan for rosbag/npz dirs."""
    if not all_rosbags:
        return [path]
    results: list[Path] = []
    for child in sorted(path.iterdir()):
        if not child.is_dir():
            continue
        if (child / "metadata.yaml").exists():
            results.append(child)
        elif any(child.glob("*.npz")):
            results.append(child)
    return results


def _resolve_annotation_path(
    session_dir: Path,
    annotation_path: Path | None,
    annotations_root: Path | None,
) -> Path | None:
    if annotation_path is not None:
        return annotation_path
    if annotations_root is not None:
        candidate = annotations_root / f"{session_dir.name}.yaml"
        if candidate.exists():
            return candidate
    return None


def _resolve_output_dir(
    session_dir: Path,
    output_root: Path | None,
    batch_mode: bool = False,
) -> Path:
    if batch_mode and output_root is not None:
        return output_root / session_dir.name
    if output_root is not None:
        return output_root
    return session_dir / "analysis"


def _format_stage_timings(timings_s: dict[str, float]) -> str:
    return ", ".join(f"{name}={duration:.2f}s" for name, duration in timings_s.items())


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run offline UWB session processing.")
    parser.add_argument(
        "--input",
        help=f"Path to rosbag or npz directory. Default: uwb_rosbags/{DEFAULT_SESSION_NAME}",
    )
    parser.add_argument("--annotation", help="Path to annotation YAML.")
    parser.add_argument("--annotations-root", help="Directory to search for per-session annotations.")
    parser.add_argument("--output-dir", help="Directory for output artifacts.")
    parser.add_argument("--all-rosbags", action="store_true",
                        help="Process all sessions found under --input.")
    parser.add_argument("--topic", default="/uwb/frame_raw")
    parser.add_argument("--range-resolution-m", type=float, default=0.15)
    parser.add_argument("--default-range-min-m", type=float, default=0.3)
    parser.add_argument("--default-range-max-m", type=float, default=6.0)
    parser.add_argument("--wall-clip-m", type=float, default=0.3)
    parser.add_argument("--motion-threshold", type=float, default=3.0)
    parser.add_argument("--background-window-s", type=float, default=1.5)
    parser.add_argument("--doppler-window-s", type=float, default=4.0)
    parser.add_argument(
        "--max-rd-fft-frames",
        type=int,
        default=256,
        help="Cap the range_doppler slow-time FFT length, especially for high-FPS bags.",
    )
    parser.add_argument("--microdoppler-window-s", type=float, default=1.5)
    parser.add_argument("--stft-overlap", type=float, default=0.75)
    parser.add_argument("--top-k-taps", type=int, default=6)
    parser.add_argument("--animate", action="store_true", default=False)
    parser.add_argument("--animate-3d", action="store_true", default=False)
    parser.add_argument("--animate-aoa", action="store_true", default=False)
    parser.add_argument("--aoa-rx-spacing-m", type=float, default=_FOV_RX_SPACING_M)
    parser.add_argument("--aoa-fov-deg", type=float, default=_FOV_LIMIT_DEG)
    parser.add_argument("--aoa-angle-bins", type=int, default=121)
    parser.add_argument(
        "--cfar-mode", default="range_only", choices=["range_only", "range_doppler"],
        help="range_only: 1-D OS/CA-CFAR per frame (default). range_doppler: slow-time FFT + per-Doppler-bin CFAR.",
    )
    parser.add_argument(
        "--use-2d-ekf", action="store_true", default=False,
        help="Cartesian EKF state [x,y,vx,vy] with nonlinear polar measurement model. "
             "Requires dual-RX for AoA. Combine with --cfar-mode range_doppler for full paper pipeline.",
    )
    parser.add_argument(
        "--use-group-association", action="store_true", default=False,
        help="Each track absorbs all gated unassigned detections and updates on their averaged measurement (paper eq. 15).",
    )
    parser.add_argument(
        "--use-mvdr", action="store_true", default=False,
        help="Use MVDR beamforming for pre-tracking AoA instead of instantaneous PDoA. "
             "Builds a sample covariance over a sliding frame window (--use-2d-ekf must also be set).",
    )
    parser.add_argument(
        "--use-dbscan-init", action="store_true", default=False,
        help="Cluster unassigned CFAR detections with DBSCAN before track initiation. "
             "Suppresses single-point noise tracks; improves person-count accuracy.",
    )
    parser.add_argument(
        "--occ-min-duration", type=float, default=3.0,
        help="Minimum track duration (s) for a track to count as a person in occupancy. "
             "Increase to suppress short spurious tracks (default 3.0 s).",
    )
    return parser


def config_from_args(args: argparse.Namespace) -> DetectionConfig:
    return DetectionConfig(
        tap_spacing_m=float(getattr(args, "range_resolution_m", 0.15)),
        default_range_gate_m=(
            float(getattr(args, "default_range_min_m", 0.3)),
            float(getattr(args, "default_range_max_m", 6.0)),
        ),
        wall_clip_m=float(getattr(args, "wall_clip_m", 0.3)),
        motion_threshold=float(getattr(args, "motion_threshold", 3.0)),
        doppler_window_s=float(getattr(args, "doppler_window_s", 4.0)),
        max_rd_fft_frames=int(getattr(args, "max_rd_fft_frames", 256)),
        microdoppler_window_s=float(getattr(args, "microdoppler_window_s", 1.5)),
        stft_overlap=float(getattr(args, "stft_overlap", 0.75)),
        occ_min_duration_s=float(getattr(args, "occ_min_duration", 3.0)),
    )


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _per_path_clutter_mag(preprocessed, path_idx: int) -> np.ndarray:
    """Clutter-removed magnitude for a single RX path (batch mean subtraction)."""
    frames = preprocessed.session.frames          # (F, P, T) complex
    global_mean = frames.mean(axis=0, keepdims=True)
    clutter = np.abs(frames - global_mean).astype(np.float32)
    wall_mask = preprocessed.range_axis_m >= preprocessed.config.wall_clip_m
    path_mag = clutter[:, path_idx, :].copy()
    path_mag[:, ~wall_mask] = 0.0
    return path_mag


def _save_detections_csv(
    detections_per_frame: list[list],
    timestamps_s: np.ndarray,
    output_path: Path,
) -> None:
    fields = ["frame_idx", "timestamp_s", "tap_idx", "range_m", "magnitude", "threshold", "range_rate_m_s"]
    with output_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for fi, dets in enumerate(detections_per_frame):
            t_s = float(timestamps_s[fi]) if fi < len(timestamps_s) else float(fi) * 0.01
            for d in dets:
                writer.writerow({
                    "frame_idx": d.frame_idx,
                    "timestamp_s": round(t_s, 6),
                    "tap_idx": d.tap_idx,
                    "range_m": round(d.range_m, 4),
                    "magnitude": round(d.magnitude, 6),
                    "threshold": round(d.threshold, 6),
                    "range_rate_m_s": round(d.range_rate_m_s, 4) if d.range_rate_m_s is not None else "",
                })


def _append_unique_samples(dst_t: list[float], dst_m: list[float], src_t: list[float], src_m: list[float]) -> None:
    for t_s, r_m in zip(src_t, src_m):
        if dst_t and float(t_s) <= float(dst_t[-1]):
            continue
        dst_t.append(float(t_s))
        dst_m.append(float(r_m))


def _append_unique_trajectory(
    dst_t: list[float],
    dst_m: list[float],
    dst_obs: list[bool],
    src_t: list[float],
    src_m: list[float],
    src_obs: list[bool],
) -> None:
    for t_s, r_m, observed in zip(src_t, src_m, src_obs):
        if dst_t and float(t_s) <= float(dst_t[-1]):
            continue
        dst_t.append(float(t_s))
        dst_m.append(float(r_m))
        dst_obs.append(bool(observed))


def _consolidate_tracks(
    tracks: list[Track],
    gate_m: float = 0.60,
    merge_gap_s: float = 1.0,
) -> tuple[list[Track], dict[int, int]]:
    """Merge short adjacent track fragments that follow the same physical target.

    Returns (merged_tracks, id_remap) where id_remap maps every raw track_id to
    the track_id of the consolidated survivor.  Use the remap when computing
    per-frame occupancy so that fragments of the same person deduplicate.
    """
    if len(tracks) <= 1:
        return list(tracks), {t.track_id: t.track_id for t in tracks}

    sorted_tracks = sorted(tracks, key=lambda t: t.history_t[0] if t.history_t else np.inf)
    merged: list[Track] = []
    id_remap: dict[int, int] = {}

    for track in sorted_tracks:
        if not track.history_t:
            continue

        t_start = float(track.history_t[0])
        r_start = float(track.history_m[0])
        best: Track | None = None
        best_err = np.inf

        for candidate in merged:
            if not candidate.history_t:
                continue
            t_end = float(candidate.history_t[-1])
            time_gap = t_start - t_end
            if not (0.0 <= time_gap <= merge_gap_s):
                continue

            if len(candidate.history_m) >= 3:
                dt_hist = float(candidate.history_t[-1] - candidate.history_t[-3])
                dr_hist = float(candidate.history_m[-1] - candidate.history_m[-3])
                velocity = (dr_hist / dt_hist) if dt_hist > 1e-6 else float(candidate.state[1])
            else:
                velocity = float(candidate.state[1])

            pred_r = float(candidate.history_m[-1]) + velocity * time_gap
            err = abs(pred_r - r_start)
            if err <= gate_m and err < best_err:
                best = candidate
                best_err = err

        if best is None:
            merged.append(track)
            id_remap[track.track_id] = track.track_id
            continue

        # track is absorbed into best — record the remap before mutating best
        id_remap[track.track_id] = best.track_id
        _append_unique_samples(best.history_t, best.history_m, track.history_t, track.history_m)
        _append_unique_trajectory(
            best.trajectory_t,
            best.trajectory_m,
            best.trajectory_observed,
            track.trajectory_t,
            track.trajectory_m,
            track.trajectory_observed,
        )
        best.state = np.array(track.state, copy=True)
        best.covariance = np.array(track.covariance, copy=True)
        best.hit_count += track.hit_count
        best.miss_count = min(best.miss_count, track.miss_count)

    return merged, id_remap


def _save_occupancy_plot(
    occupancy_per_frame: list[int],
    timestamps_s: np.ndarray,
    person_count_estimate: int,
    output_path: Path,
) -> None:
    """Save a time-series plot of confirmed-track count (= person occupancy)."""
    import matplotlib.pyplot as plt

    ts = np.asarray(timestamps_s, dtype=np.float64)
    if ts.size != len(occupancy_per_frame):
        ts = np.arange(len(occupancy_per_frame), dtype=np.float64)

    fig, ax = plt.subplots(figsize=(10, 3))
    ax.step(ts, occupancy_per_frame, where="post", color="steelblue", linewidth=1.2, label="confirmed tracks")
    ax.axhline(person_count_estimate, color="tomato", linewidth=1.5, linestyle="--",
               label=f"estimate: {person_count_estimate} person(s)")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("person count")
    ax.set_title("Person Occupancy")
    ax.set_ylim(bottom=0, top=max(max(occupancy_per_frame, default=0) + 1, person_count_estimate + 1))
    ax.yaxis.set_major_locator(plt.MaxNLocator(integer=True))
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(output_path, dpi=120)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main pipeline
# ---------------------------------------------------------------------------

_FOV_LIMIT_DEG = 60.0
_FOV_RX_SPACING_M = 0.108
_FOV_RANGE_MAX_M = 7.5


def process_session(
    input_path: Path,
    annotation_path: Path | None,
    output_dir: Path,
    config: DetectionConfig,
    topic: str = "/uwb/frame_raw",
    animate_doppler: bool = False,
    animate_aoa: bool = True,
    aoa_rx_spacing_m: float = 0.108,
    aoa_fov_deg: float = 60.0,
    aoa_angle_bins: int = 121,
    cfar_mode: str = "range_only",
    use_2d_ekf: bool = False,
    use_group_association: bool = False,
    use_mvdr: bool = False,
    use_dbscan_init: bool = False,
) -> tuple[dict[str, Any], Path]:
    """Run the full offline pipeline and save all required artifacts.

    Follows the step-by-step procedure in ground_truth/uwb_radar_processing_guide.tex.

    Returns
    -------
    summary : dict  — JSON-serialisable session summary (used by batch_eval aggregation)
    output_dir : Path — directory where all artifacts were written
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    timings_s: dict[str, float] = {}
    total_start = time.perf_counter()

    # 1. Load and preprocess
    stage_start = time.perf_counter()
    session = load_session(input_path, topic=topic)
    preprocessed = preprocess_session(session, config)
    timings_s["load_preprocess"] = time.perf_counter() - stage_start

    # Per-frame SNR from ROI/background power ratio (used in video HUDs).
    snr_db_per_frame = 10.0 * np.log10(
        np.asarray(preprocessed.roi_to_background_power_ratio, dtype=np.float64) + 1e-6
    )

    # 2. Session-level plots (guide §2.3, §6)
    stage_start = time.perf_counter()
    save_range_time_plot(preprocessed, output_dir / "range_time.png")
    save_range_doppler_heatmap(preprocessed, output_dir / "range_doppler.png")
    timings_s["session_plots"] = time.perf_counter() - stage_start

    # 3. CFAR detection (guide §3.1, §5.2)
    n_rx = session.frames.shape[1]
    cfar_variant = "OS" if n_rx >= 2 else "CA"
    cfar_cfg = CfarDetectionConfig(
        num_ref_cells=8, num_guard_cells=2, pfa=1e-2,
        variant=cfar_variant, os_rank=12, max_peaks_per_frame=4,
    )
    n_paths = session.frames.shape[1]
    stage_start = time.perf_counter()
    if cfar_mode == "range_doppler":
        # Slow-time FFT → per-Doppler-bin OS-CFAR; range_rate_m_s populated from Doppler axis
        detections_per_frame = run_cfar_rd(
            preprocessed, cfar_cfg,
            doppler_window_s=config.doppler_window_s,
            max_window_frames=config.max_rd_fft_frames,
            hop_frames=config.rd_hop_frames if config.rd_hop_frames > 0 else None,
        )
    else:
        # 1-D range-only CFAR with dual-channel merge, then phase-Doppler annotation
        if n_paths >= 2:
            dets_c = run_cfar_session(preprocessed, cfar_cfg,
                                      clutter_mag=_per_path_clutter_mag(preprocessed, 0))
            dets_b = run_cfar_session(preprocessed, cfar_cfg,
                                      clutter_mag=_per_path_clutter_mag(preprocessed, 1))
            detections_per_frame = [
                merge_dual_channel_peaks(c, b, fi, preprocessed.range_axis_m)
                for fi, (c, b) in enumerate(zip(dets_c, dets_b))
            ]
        else:
            combined_mag = combine_paths_max(preprocessed)
            detections_per_frame = run_cfar_session(preprocessed, cfar_cfg, clutter_mag=combined_mag)
        annotate_detections_with_doppler(
            detections_per_frame,
            preprocessed.highpass_complex,
            preprocessed.frame_rate_hz,
            preprocessed.config.carrier_frequency_hz,
        )
    timings_s["cfar"] = time.perf_counter() - stage_start

    # 3.5  Pre-tracking AoA annotation — sets CfarDetection.azimuth_deg on every
    #      detection so the 2-D Cartesian EKF can initialise tracks with a proper
    #      (range, azimuth) measurement.  Requires dual-RX; skipped silently if
    #      only one RX path is available.
    stage_start = time.perf_counter()
    if use_2d_ekf and session.frames.shape[1] >= 2:
        try:
            paired_pre = pair_rx_channels(session)
            annotate_detections_aoa(
                paired_pre,
                detections_per_frame,
                carrier_frequency_hz=preprocessed.config.carrier_frequency_hz,
                rx_spacing_m=aoa_rx_spacing_m,
                fov_limit_deg=aoa_fov_deg,
                use_mvdr=use_mvdr,
            )
            method = "MVDR" if use_mvdr else "PDoA"
            n_annotated = sum(1 for f in detections_per_frame for d in f if d.azimuth_deg is not None)
            print(f"[pre-aoa] annotated {n_annotated} detections with {method} azimuth")
        except Exception as exc:
            print(f"[pre-aoa] skipped ({exc}); falling back to 1-D EKF")
            use_2d_ekf = False
    timings_s["pre_aoa"] = time.perf_counter() - stage_start

    # 4. EKF tracking (guide §3.4)
    stage_start = time.perf_counter()
    dt_s = 1.0 / max(preprocessed.frame_rate_hz, 1.0)
    # All timing parameters scale with actual frame rate for 20 Hz or 500 Hz bags.
    fps = preprocessed.frame_rate_hz
    max_misses = max(15, int(0.40 * fps))        # survive short CFAR dropouts (~0.4 s)
    init_window = max(6, int(0.08 * fps))        # allow brief startup gaps before confirming
    # Exponential velocity decay with a gentler ~400 ms time constant.
    # This keeps tracks connected through short fades without letting them drift forever.
    velocity_decay = float(np.exp(-2.5 * dt_s))
    # With DBSCAN init the cluster itself already provides multi-detection evidence,
    # so we can afford a higher confirm_hits threshold to suppress short noise tracks.
    confirm_hits = max(4, int(0.20 * fps)) if use_dbscan_init else 2
    tracker_cfg = TrackingConfig(
        dt_s=dt_s,
        sigma_meas_m=0.15,
        sigma_meas_v=0.25,
        sigma_process_v=0.45,
        filter_name="ekf",
        gate_distance_m=0.85,
        gate_chi2=12.0,
        confirm_hits=confirm_hits,
        max_misses=max_misses,
        init_hits=2,
        init_window_frames=init_window,
        use_hungarian=True,
        vel_weight=0.0,
        min_track_duration_s=1.0,
        velocity_decay=velocity_decay,
        use_2d_ekf=use_2d_ekf,
        use_group_association=use_group_association,
        use_dbscan_init=use_dbscan_init,
    )
    tracker = MultiTargetTracker(tracker_cfg)
    tracks_per_frame: list[list] = []
    for fi, dets in enumerate(detections_per_frame):
        t_s = float(session.timestamps_s[fi]) if fi < len(session.timestamps_s) else float(fi) * dt_s
        tracks_per_frame.append(tracker.update(fi, t_s, dets))

    raw_tracks = tracker.tracks_snapshot()
    all_tracks, _id_remap = _consolidate_tracks(raw_tracks, gate_m=0.70, merge_gap_s=2.5)
    save_multi_peak_tracking_plot(
        preprocessed,
        all_tracks,
        output_dir / "peak_tracking.png",
        cfar_variant=cfar_variant,
        tracker_label="EKF",
    )

    # Person occupancy: count only tracks that were active long enough to be a real person.
    # Short-lived tracks (noise, clutter bursts) are excluded by the duration threshold.
    occ_min_duration_s = max(tracker_cfg.min_track_duration_s,
                             float(getattr(config, "occ_min_duration_s", 3.0)))
    _occ_range_max_m = config.default_range_gate_m[1]
    persistent_ids: set[int] = {
        t.track_id for t in all_tracks
        if len(t.history_t) >= 2
        and (max(t.history_t) - min(t.history_t)) >= occ_min_duration_s
        and float(np.mean(t.history_m)) <= _occ_range_max_m
    }
    persistent_list = [t for t in all_tracks if t.track_id in persistent_ids]

    # Deduplicate overlapping persistent tracks: if a shorter track overlaps a
    # longer one for >= 2 s AND their median ranges during the overlap differ by
    # < 0.5 m, it is an EKF duplicate of the same person.  Remove the shorter
    # track so it does not inflate the simultaneous count.
    _sorted_p = sorted(
        persistent_list,
        key=lambda t: (max(t.history_t) - min(t.history_t)) if len(t.history_t) >= 2 else 0.0,
        reverse=True,
    )
    _dup_ids: set[int] = set()
    for _i, _ta in enumerate(_sorted_p):
        if _ta.track_id in _dup_ids:
            continue
        _ta_lo, _ta_hi = float(min(_ta.history_t)), float(max(_ta.history_t))
        for _tb in _sorted_p[_i + 1:]:
            if _tb.track_id in _dup_ids:
                continue
            _tb_lo, _tb_hi = float(min(_tb.history_t)), float(max(_tb.history_t))
            _overlap = max(0.0, min(_ta_hi, _tb_hi) - max(_ta_lo, _tb_lo))
            if _overlap < 2.0:
                continue
            # Compare median range during the shared overlap window
            def _median_range_in(trk: Track, lo: float, hi: float) -> float:
                pts = [r for t, r in zip(trk.history_t, trk.history_m) if lo <= t <= hi]
                return float(np.median(pts)) if pts else float(np.mean(trk.history_m))
            _r_a = _median_range_in(_ta, max(_ta_lo, _tb_lo), min(_ta_hi, _tb_hi))
            _r_b = _median_range_in(_tb, max(_ta_lo, _tb_lo), min(_ta_hi, _tb_hi))
            if abs(_r_a - _r_b) < 0.5:
                _dup_ids.add(_tb.track_id)

    if _dup_ids:
        persistent_ids -= _dup_ids
        persistent_list = [t for t in persistent_list if t.track_id not in _dup_ids]

    n_persistent = len(persistent_ids)

    # Build per-frame occupancy from tracks_per_frame (same source as peak_tracking.png)
    # with consolidation remap + dedup so each physical person counts as one.
    occupancy_per_frame: list[int] = [
        len({
            _id_remap.get(t.track_id, t.track_id)
            for t in frame_tracks
            if _id_remap.get(t.track_id, t.track_id) in persistent_ids
        })
        for frame_tracks in tracks_per_frame
    ]
    nonzero_occ = [c for c in occupancy_per_frame if c > 0]
    person_count_peak = max(nonzero_occ) if nonzero_occ else 0

    # Mode = most frequent simultaneous count.  Immune to brief spikes; a duplicate
    # must be active for the majority of the session to shift it.  This is the sole
    # estimate — DBSCAN on mean range is unreliable for walking targets whose mean
    # range shifts as they move and would fragment one person into several clusters.
    from collections import Counter as _Counter
    _occ_hist = _Counter(nonzero_occ)
    person_count_mode = max(_occ_hist, key=_occ_hist.get) if _occ_hist else 0
    person_count_estimate = person_count_mode

    for _t in persistent_list:
        _dur = float(max(_t.history_t) - min(_t.history_t)) if len(_t.history_t) >= 2 else 0.0
        print(f"  [occ] track {_t.track_id:3d}  dur={_dur:.1f}s  "
              f"range {float(min(_t.history_m)):.2f}–{float(max(_t.history_m)):.2f} m  "
              f"mean={float(np.mean(_t.history_m)):.2f} m")
    _dup_note = f", {len(_dup_ids)} dup(s) removed" if _dup_ids else ""
    print(f"[occupancy] estimated {person_count_estimate} person(s)  "
          f"(peak {person_count_peak}, mode={person_count_mode}, "
          f"{n_persistent} persistent / {len(all_tracks)} total tracks{_dup_note}, "
          f"occ_min_dur={occ_min_duration_s:.1f}s)")
    _save_occupancy_plot(
        occupancy_per_frame,
        session.timestamps_s,
        person_count_estimate,
        output_dir / "occupancy.png",
    )

    timings_s["tracking"] = time.perf_counter() - stage_start
    print(f"[tracking] {len(raw_tracks)} raw track(s) -> {len(all_tracks)} consolidated track(s)")

    # Sliding-window range-Doppler animation (guide §3.2 — slow-time FFT)
    stage_start = time.perf_counter()
    if animate_doppler:
        try:
            save_doppler_animation(
                preprocessed,
                output_dir / "doppler.mp4",
                tracks=all_tracks,
                occupancy_per_frame=occupancy_per_frame,
            )
        except Exception as exc:
            import traceback
            print(f"[doppler anim] failed: {exc}")
            traceback.print_exc()
    timings_s["doppler_animation"] = time.perf_counter() - stage_start

    # FOV polar plot / animation (requires dual-RX for AoA).
    # Triggers with --animate or --animate-aoa when two RX paths are present.
    # Full range-angle background grid (expensive per-frame FFT) only built with --animate-aoa.
    stage_start = time.perf_counter()
    if (animate_aoa or animate_doppler) and session.frames.shape[1] >= 2:
        try:
            paired = pair_rx_channels(session)
            track_aoas_per_frame = estimate_track_aoa_session(
                paired,
                detections_per_frame,
                tracks_per_frame,
                carrier_frequency_hz=preprocessed.config.carrier_frequency_hz,
                rx_spacing_m=aoa_rx_spacing_m,
                fov_limit_deg=aoa_fov_deg,
            )
            if len(all_tracks) > 0:
                save_fov_polar_plot(
                    track_aoas_per_frame,
                    fov_limit_deg=aoa_fov_deg,
                    output_path=output_dir / "fov.png",
                    range_max_m=_FOV_RANGE_MAX_M,
                )
                save_fov_animation(
                    track_aoas_per_frame,
                    session.timestamps_s,
                    fov_limit_deg=aoa_fov_deg,
                    output_path=output_dir / "fov.mp4",
                    range_max_m=_FOV_RANGE_MAX_M,
                    snr_db_per_frame=snr_db_per_frame,
                    occupancy_per_frame=occupancy_per_frame,
                    presence_per_frame=preprocessed.presence_mask,
                )
            if animate_aoa:
                range_angle_grids = np.zeros(
                    (len(session.timestamps_s), preprocessed.range_axis_m.size, aoa_angle_bins),
                    dtype=np.float32,
                )
                _angle_axis_deg: np.ndarray | None = None
                _range_axis_m: np.ndarray | None = None
                for frame_idx in range(len(session.timestamps_s)):
                    grid, _angle_axis_deg, _range_axis_m = build_range_angle_map(
                        paired,
                        range_axis_m=preprocessed.range_axis_m,
                        frame_idx=frame_idx,
                        carrier_frequency_hz=preprocessed.config.carrier_frequency_hz,
                        rx_spacing_m=aoa_rx_spacing_m,
                        fov_limit_deg=aoa_fov_deg,
                        angle_bins=aoa_angle_bins,
                    )
                    range_angle_grids[frame_idx] = grid
                if _angle_axis_deg is not None and _range_axis_m is not None:
                    save_aoa_animation(
                        timestamps_s=session.timestamps_s,
                        track_aoas_per_frame=track_aoas_per_frame,
                        range_angle_grids=range_angle_grids,
                        angle_axis_deg=_angle_axis_deg,
                        range_axis_m=_range_axis_m,
                        output_path=output_dir / "range_azimuth.mp4",
                        fov_limit_deg=aoa_fov_deg,
                        fps=10,
                    )
        except Exception as exc:
            import traceback
            print(f"[aoa/fov] skipped: {exc}")
            traceback.print_exc()
    timings_s["aoa"] = time.perf_counter() - stage_start

    # 5. Breathing extraction from each confirmed track (guide §4)
    stage_start = time.perf_counter()
    breathing_results: list[dict[str, Any]] = []
    for track in all_tracks:
        track_duration_s = (
            max(track.history_t) - min(track.history_t) if len(track.history_t) >= 2 else 0.0
        )
        if track_duration_s < tracker_cfg.min_track_duration_s:
            continue
        ts = session.timestamps_s
        t_start, t_end = float(min(track.history_t)), float(max(track.history_t))
        frame_mask = (ts >= t_start) & (ts <= t_end)
        mean_range_m = float(np.mean(track.history_m))
        track_tap = int(np.clip(
            round(mean_range_m / preprocessed.config.tap_spacing_m),
            0, preprocessed.highpass_complex.shape[1] - 1,
        ))
        br = run_breathing_extraction(
            preprocessed.highpass_complex,
            preprocessed.peak_tap_per_frame,
            frame_mask,
            track_tap,
            preprocessed.frame_rate_hz,
        )
        breathing_results.append({
            "track_id": track.track_id,
            "tap_idx": br.tap_idx,
            "is_static": br.is_static,
            "freq_hz": br.freq_hz,
            "freq_bpm": br.freq_bpm,
            "snr_db": br.snr_db,
            "phase_arc_deg": br.phase_arc_deg,
        })
    timings_s["breathing"] = time.perf_counter() - stage_start

    # 6. Per-window annotation analysis (guide §4, §7)
    stage_start = time.perf_counter()
    window_results: list[dict[str, Any]] = []
    if annotation_path is not None:
        annotation = load_annotation_file(annotation_path)
        for window in annotation.windows:
            window_dir = output_dir / window.label
            window_dir.mkdir(parents=True, exist_ok=True)
            try:
                det_result, extras = detect_window(preprocessed, window, artifact_dir=window_dir)
                save_spectrogram_plot(
                    extras["doppler"],
                    window_dir / "doppler.png",
                    frequency_limit_hz=config.doppler_limit_hz,
                )
                save_spectrogram_plot(
                    extras["microdoppler"],
                    window_dir / "microdoppler.png",
                    frequency_limit_hz=config.microdoppler_limit_hz,
                )
                window_results.append(det_result.to_dict())
            except (ValueError, RuntimeError):
                pass
    timings_s["annotations"] = time.perf_counter() - stage_start

    # 7. Artifacts: detections.csv and summary.json
    stage_start = time.perf_counter()
    _save_detections_csv(detections_per_frame, session.timestamps_s, output_dir / "detections.csv")

    summary: dict[str, Any] = {
        "session_id": session.session_id,
        "frame_count": int(session.frames.shape[0]),
        "frame_rate_hz": float(preprocessed.frame_rate_hz),
        "duration_s": float(session.timestamps_s[-1]) if session.timestamps_s.size > 0 else 0.0,
        "selected_path": int(preprocessed.selected_path),
        "dominant_tap": int(preprocessed.dominant_tap),
        "dominant_range_m": float(preprocessed.range_axis_m[preprocessed.dominant_tap]),
        "cfar_variant": cfar_variant,
        "cfar_mode": cfar_mode,
        "tracker_filter": tracker_cfg.filter_name,
        "detections": window_results,
        "confirmed_tracks": [
            {
                "track_id": t.track_id,
                "history_length": len(t.history_m),
                "mean_range_m": float(np.mean(t.history_m)) if t.history_m else 0.0,
            }
            for t in all_tracks
        ],
        "breathing": breathing_results,
        "occupancy": {
            "person_count_estimate": person_count_estimate,
            "person_count_peak": person_count_peak,
            "confirmed_track_count": len(all_tracks),
            "persistent_track_count": n_persistent,
            "occ_min_duration_s": occ_min_duration_s,
        },
    }
    timings_s["artifacts"] = time.perf_counter() - stage_start
    timings_s["total"] = time.perf_counter() - total_start
    summary["timings_s"] = {name: round(duration, 3) for name, duration in timings_s.items()}
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2))
    print(f"[timing] {_format_stage_timings(summary['timings_s'])}")

    return summary, output_dir


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    input_path = Path(args.input) if args.input else _default_input_path(all_rosbags=args.all_rosbags)
    output_root = Path(args.output_dir) if args.output_dir else None
    annotation_path = Path(args.annotation) if args.annotation else None
    annotations_root = Path(args.annotations_root) if getattr(args, "annotations_root", None) else None
    config = config_from_args(args)

    session_inputs = _discover_session_inputs(input_path, all_rosbags=args.all_rosbags)

    for session_path in session_inputs:
        output_dir = _resolve_output_dir(session_path, output_root, batch_mode=args.all_rosbags)
        resolved_ann = _resolve_annotation_path(session_path, annotation_path, annotations_root)
        summary, out_dir = process_session(
            input_path=session_path,
            annotation_path=resolved_ann,
            output_dir=output_dir,
            config=config,
            topic=args.topic,
            animate_doppler=args.animate,
            animate_aoa=args.animate_aoa,
            aoa_rx_spacing_m=args.aoa_rx_spacing_m,
            aoa_fov_deg=args.aoa_fov_deg,
            aoa_angle_bins=args.aoa_angle_bins,
            cfar_mode=args.cfar_mode,
            use_2d_ekf=args.use_2d_ekf,
            use_group_association=args.use_group_association,
            use_mvdr=args.use_mvdr,
            use_dbscan_init=args.use_dbscan_init,
        )
        print(json.dumps({
            "session": summary["session_id"],
            "output_dir": str(out_dir),
            "timings_s": summary.get("timings_s", {}),
        }, indent=2))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
