"""run.py — Hardcoded single-session runner.

Edit the CONFIG block below, then:
    cd bosch_UWB
    python uwb_processing/run.py
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

# ── PATH SETUP ───────────────────────────────────────────────────────────────
_HERE = Path(__file__).resolve().parent          # bosch_UWB/uwb_processing/
_ROOT = _HERE.parent                             # bosch_UWB/
sys.path.insert(0, str(_HERE))                   # makes "uwb_processing" importable

# ═════════════════════════════════════════════════════════════════════════════
# CONFIG — edit everything here
# ═════════════════════════════════════════════════════════════════════════════

# Which rosbag to process (folder name inside uwb_rosbags/)
BAG = "shubo_edison_two_static_walking_medium_2ms"

# Annotation file (None = skip per-window doppler/microdoppler plots)
# Set to a string like "breathing_kaytlin_edison.yaml" to load it
ANNOTATION = None

# Where to write outputs (None = uwb_rosbags/<BAG>/analysis/)
OUTPUT_DIR = None

# ── Range gating ─────────────────────────────────────────────────────────────
RANGE_MIN_M = 0.30    # hard wall clip (taps below this are masked)
RANGE_MAX_M = 6.00    # far gate (ignored beyond this)

# ── CFAR ─────────────────────────────────────────────────────────────────────
CFAR_NUM_REF      = 6     # reference cells per side
CFAR_NUM_GUARD    = 2     # guard cells per side
CFAR_PFA          = 0.05 # probability of false alarm (lower = fewer detections)
CFAR_MAX_PEAKS    = 6     # max detections per frame
CFAR_CLUSTER_GAP  = 0     # taps: merge adjacent detections within this gap+1
                           # 0 = only fuse literally-touching taps (best for multi-person)
                           # 2 = fuse within 3 taps (default, fine for single person)

# ── Kalman tracker ────────────────────────────────────────────────────────────
TRACKER_GATE_M      = 0.45   # association gate radius (m)
TRACKER_CONFIRM_HITS = 2     # frames needed to confirm a new track
TRACKER_MAX_MISSES   = 15    # consecutive misses before dropping a track

# ── Breathing ────────────────────────────────────────────────────────────────
ENABLE_BREATHING    = True
BREATHING_NFFT      = 1024   # must be >= 1024 for 5.9 BPM resolution at 100 Hz
BREATHING_HP_HZ     = 0.05   # high-pass cut-off for phase drift removal
BREATHING_BAND_HZ   = (0.1, 0.8)   # search band (Hz) = 6–48 BPM

# ── Dual-channel CFAR merge ───────────────────────────────────────────────────
ENABLE_DUAL_MERGE    = True   # merge RxC and RxB peaks (guide §5.2)
MERGE_TOLERANCE_TAPS = 0     # max tap offset for a match
NMS_MIN_SEP_TAPS     = 0     # NMS: 0=off, 1=same-tap only, 2=suppress within 2 taps

# ── Spectrograms ─────────────────────────────────────────────────────────────
DOPPLER_WINDOW_S       = 4.0
MICRODOPPLER_WINDOW_S  = 1.5
STFT_OVERLAP           = 0.75

# ── ROS topic ────────────────────────────────────────────────────────────────
TOPIC = "/uwb/frame_raw"
ENABLE_FOV_VIDEO       = False   # export AoA/FOV mp4 using current CFAR + tracking
FOV_VIDEO_FPS          = 10      # playback fps for the output video
FOV_VIDEO_FRAME_STRIDE = 4       # render every Nth frame; 1 = full-rate export
FOV_VIDEO_INCLUDE_FALLBACK_DETECTIONS = False  # if False, only show confirmed tracks
FOV_VIDEO_MIN_TRACK_HISTORY_FRAMES = 4         # hide short-lived tracks to reduce flicker
FOV_VIDEO_RX_SPACING_M = 0.009   # RxC-RxB baseline on this board
FOV_VIDEO_FOV_DEG      = 90.0    # render +/- this field of view
FOV_VIDEO_ANGLE_BINS   = 181     # range-angle heatmap resolution

# ═════════════════════════════════════════════════════════════════════════════
# END OF CONFIG — nothing below normally needs editing
# ═════════════════════════════════════════════════════════════════════════════

class _Tee:
    """Write to multiple streams simultaneously (stdout + log file)."""
    def __init__(self, *streams): self._streams = streams
    def write(self, s):
        for st in self._streams: st.write(s)
    def flush(self):
        for st in self._streams: st.flush()

from uwb_processing.annotations import load_annotation_file
from uwb_processing.animate_aoa import save_aoa_animation
from uwb_processing.aoa import build_range_angle_map, estimate_track_aoa_session, pair_rx_channels
from uwb_processing.breathing import run_breathing_extraction
from uwb_processing.cfar import (
    CfarDetectionConfig,
    combine_paths_max,
    merge_dual_channel_peaks,
    run_cfar_session,
)
from uwb_processing.detection import detect_window
from uwb_processing.loaders import load_session
from uwb_processing.plotting import (
    save_multi_peak_tracking_plot,
    save_range_doppler_heatmap,
    save_range_time_plot,
    save_spectrogram_plot,
)
from uwb_processing.preprocessing import preprocess_session
from uwb_processing.tracker import MultiTargetTracker
from uwb_processing.types import (
    AnnotationWindow,
    CfarDetectionConfig,
    DetectionConfig,
    Track,
    TrackingConfig,
)


def _save_aoa_plot(preprocessed, session, detections, tracks, out_dir: Path) -> None:
    """AoA summary: angle-over-time per track + time-averaged range-angle heatmap."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from uwb_processing.aoa import (
        build_range_angle_map, estimate_angle_deg_from_pair, pair_rx_channels,
    )
    from uwb_processing.visualization_utils import db_limits, jet_color, magnitude_to_db

    if session.frames.shape[1] < 2:
        print("  [aoa] skipped: only 1 RX path")
        return
    try:
        paired = pair_rx_channels(session)
    except ValueError as exc:
        print(f"  [aoa] skipped: {exc}")
        return

    carrier_hz   = 7987.2e6   # Ch9
    rx_spacing_m = 0.009      # L = 9 mm baseline RxC–RxB
    fov_deg      = 90.0
    angle_bins   = 181
    ts           = session.timestamps_s
    n_tracks     = max(len(tracks), 1)

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # ── Left: angle-over-time per confirmed track ────────────────────────────
    ax_t = axes[0]
    for ti, track in enumerate(tracks):
        angles, t_vals, prev = [], [], None
        for t_hist, r_hist in zip(track.history_t, track.history_m):
            fi  = int(np.clip(np.argmin(np.abs(ts - t_hist)), 0, paired.rx1.shape[0] - 1))
            tap = int(np.clip(round(r_hist / preprocessed.config.tap_spacing_m),
                              0, paired.rx1.shape[1] - 1))
            angle = estimate_angle_deg_from_pair(
                complex(paired.rx1[fi, tap]), complex(paired.rx2[fi, tap]),
                carrier_hz, rx_spacing_m, fov_deg, prev,
            )
            prev = angle
            angles.append(angle)
            t_vals.append(t_hist)
        color = jet_color(ti, n_tracks)
        mean_a = float(np.mean(angles)) if angles else 0.0
        side   = "LEFT" if mean_a < 0 else "RIGHT"
        ax_t.plot(t_vals, angles, linewidth=1.0, color=color, alpha=0.85,
                  label=f"track {track.track_id}  {mean_a:+.1f} deg  ({side})")
    ax_t.axhline(0, color="gray", linewidth=0.8, linestyle="--", alpha=0.6)
    ax_t.set_ylim(-95, 95)
    ax_t.set_xlabel("Time (s)")
    ax_t.set_ylabel("AoA angle (deg)")
    ax_t.set_title("Angle of Arrival over time\n"
                   "negative = left of boresight | positive = right")
    ax_t.legend(loc="upper right", fontsize=8)
    ax_t.grid(True, alpha=0.3)

    # ── Right: time-averaged range-angle heatmap (sampled every ~50 frames) ──
    ax_h = axes[1]
    n_frames_total = paired.rx1.shape[0]
    step = max(1, n_frames_total // 60)
    sample_frames = list(range(0, n_frames_total, step))
    n_taps = preprocessed.range_axis_m.size
    grid_sum = np.zeros((n_taps, angle_bins), dtype=np.float32)
    angle_axis = None
    for fi in sample_frames:
        g, angle_axis, _ = build_range_angle_map(
            paired, preprocessed.range_axis_m, fi,
            carrier_hz, rx_spacing_m, fov_deg, angle_bins,
        )
        grid_sum += g
    grid_avg = grid_sum / max(len(sample_frames), 1)

    valid = preprocessed.range_axis_m >= preprocessed.config.wall_clip_m
    r_valid = preprocessed.range_axis_m[valid]
    db_grid = magnitude_to_db(grid_avg[valid])
    vmin, vmax = db_limits(db_grid)

    if angle_axis is not None:
        im = ax_h.imshow(db_grid, aspect="auto", origin="lower",
                         extent=[float(angle_axis[0]), float(angle_axis[-1]),
                                 float(r_valid[0]), float(r_valid[-1])],
                         cmap="jet", vmin=vmin, vmax=vmax)
        fig.colorbar(im, ax=ax_h, label="Magnitude (dB)", fraction=0.046, pad=0.04)
        ax_h.axvline(0, color="white", linewidth=0.8, linestyle="--", alpha=0.5)
        for ti, track in enumerate(tracks):
            mean_r   = float(np.mean(track.history_m))
            mid_t    = float(np.median(track.history_t))
            mid_fi   = int(np.clip(np.argmin(np.abs(ts - mid_t)), 0, paired.rx1.shape[0] - 1))
            tap      = int(np.clip(round(mean_r / preprocessed.config.tap_spacing_m),
                                   0, paired.rx1.shape[1] - 1))
            mean_angle = estimate_angle_deg_from_pair(
                complex(paired.rx1[mid_fi, tap]), complex(paired.rx2[mid_fi, tap]),
                carrier_hz, rx_spacing_m, fov_deg, None,
            )
            ax_h.scatter([mean_angle], [mean_r], c=[jet_color(ti, n_tracks)],
                         s=100, zorder=5, edgecolors="white", linewidths=0.8,
                         label=f"track {track.track_id}")
    ax_h.set_xlabel("Angle (deg)")
    ax_h.set_ylabel("Range (m)")
    ax_h.set_title(f"Range-angle heatmap (averaged over {len(sample_frames)} frames)\n"
                   "WARNING: L=9mm → resolution ~119 deg; left/right only reliable")
    if tracks:
        ax_h.legend(fontsize=8)

    plt.tight_layout()
    out_path = out_dir / "aoa_tracks.png"
    fig.savefig(out_path, dpi=160)
    plt.close(fig)
    print(f"  saved aoa_tracks.png")


def _save_breathing_plot(br, track_id: int, frr: float, out_dir: Path) -> None:
    """2-panel breathing plot: HP phase over time + FFT spectrum with peak marked."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(14, 4))

    # ── Left: HP-filtered phase over time ────────────────────────────────────
    t_axis = np.arange(len(br.phase_hp)) / frr
    axes[0].plot(t_axis, br.phase_hp, linewidth=0.9, color="steelblue")
    axes[0].set_xlabel("Time (s)")
    axes[0].set_ylabel("Phase (rad)")
    static_str = "static" if br.is_static else "MOVING — results unreliable"
    arc_str = f"  arc = {br.phase_arc_deg:.1f} deg" if br.phase_arc_deg is not None else ""
    axes[0].set_title(f"HP-filtered unwrapped phase — track {track_id}\n"
                      f"{static_str}{arc_str}")
    axes[0].grid(True, alpha=0.3)

    # ── Right: FFT spectrum ───────────────────────────────────────────────────
    hz_mask = (br.fft_freqs_hz >= 0.05) & (br.fft_freqs_hz <= 1.5)
    freqs   = br.fft_freqs_hz[hz_mask]
    mag     = br.fft_magnitude[hz_mask]
    axes[1].plot(freqs, mag, linewidth=1.2, color="steelblue")
    axes[1].axvspan(0.1, 0.8, alpha=0.10, color="green", label="search band (6–48 BPM)")
    if br.freq_hz is not None:
        axes[1].axvline(br.freq_hz, color="red", linewidth=2.0,
                        label=f"peak: {br.freq_bpm:.1f} BPM  |  SNR = {br.snr_db:.1f} dB")
    axes[1].set_xlabel("Frequency (Hz)")
    axes[1].set_ylabel("FFT magnitude")
    axes[1].set_title(f"Breathing spectrum — track {track_id}\n"
                      f"(N_FFT={len(br.fft_freqs_hz)*2-2}  |  FRR={frr:.0f} Hz)")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)
    # Top axis in BPM
    ax2 = axes[1].twiny()
    xl  = axes[1].get_xlim()
    ax2.set_xlim(xl[0] * 60, xl[1] * 60)
    ax2.set_xlabel("BPM")

    plt.tight_layout()
    out_path = out_dir / f"breathing_track{track_id}.png"
    fig.savefig(out_path, dpi=160)
    plt.close(fig)
    print(f"  saved breathing_track{track_id}.png")


def _snapshot_active_tracks(active_tracks: list[Track]) -> list[Track]:
    """Freeze per-frame tracker state so later updates do not mutate video input."""
    snapshots: list[Track] = []
    for track in active_tracks:
        snapshots.append(
            Track(
                track_id=track.track_id,
                state=np.array(track.state, copy=True),
                covariance=np.array(track.covariance, copy=True),
                hit_count=track.hit_count,
                miss_count=track.miss_count,
                confirmed=track.confirmed,
                history_m=list(track.history_m),
                history_t=list(track.history_t),
            )
        )
    return snapshots


def _save_fov_video(
    preprocessed,
    session,
    detections_per_frame,
    tracks_per_frame: list[list[Track]],
    out_dir: Path,
) -> Path | None:
    """Render a range-angle MP4 using the current CFAR detections and tracker states."""
    if session.frames.shape[1] < 2:
        print("  [fov] skipped: only 1 RX path")
        return None

    try:
        paired = pair_rx_channels(session)
    except ValueError as exc:
        print(f"  [fov] skipped: {exc}")
        return None

    frame_count = session.frames.shape[0]
    frame_stride = max(1, int(FOV_VIDEO_FRAME_STRIDE))
    frame_indices = np.arange(0, frame_count, frame_stride, dtype=np.int32)
    if frame_indices.size == 0 or int(frame_indices[-1]) != frame_count - 1:
        frame_indices = np.append(frame_indices, frame_count - 1)

    sampled_timestamps_s = session.timestamps_s[frame_indices]
    sampled_detections = [detections_per_frame[int(frame_idx)] for frame_idx in frame_indices]
    sampled_tracks = [tracks_per_frame[int(frame_idx)] for frame_idx in frame_indices]

    track_aoas_per_frame = estimate_track_aoa_session(
        paired,
        sampled_detections,
        sampled_tracks,
        carrier_frequency_hz=7987.2e6,
        rx_spacing_m=FOV_VIDEO_RX_SPACING_M,
        fov_limit_deg=FOV_VIDEO_FOV_DEG,
        include_detection_fallback=FOV_VIDEO_INCLUDE_FALLBACK_DETECTIONS,
        min_track_history_points=FOV_VIDEO_MIN_TRACK_HISTORY_FRAMES,
    )

    valid_mask = preprocessed.range_axis_m >= preprocessed.config.wall_clip_m
    range_axis_m = preprocessed.range_axis_m[valid_mask]
    angle_axis_deg = None
    range_angle_grids = np.zeros(
        (frame_indices.size, int(np.count_nonzero(valid_mask)), FOV_VIDEO_ANGLE_BINS),
        dtype=np.float32,
    )

    for output_frame_idx, source_frame_idx in enumerate(frame_indices):
        grid, angle_axis_deg, _ = build_range_angle_map(
            paired,
            preprocessed.range_axis_m,
            int(source_frame_idx),
            carrier_frequency_hz=7987.2e6,
            rx_spacing_m=FOV_VIDEO_RX_SPACING_M,
            fov_limit_deg=FOV_VIDEO_FOV_DEG,
            angle_bins=FOV_VIDEO_ANGLE_BINS,
        )
        range_angle_grids[output_frame_idx] = grid[valid_mask]

    output_path = out_dir / "aoa_fov.mp4"
    print(
        f"  [fov] exporting {frame_indices.size}/{frame_count} frames "
        f"(stride={frame_stride}, bins={FOV_VIDEO_ANGLE_BINS}, "
        f"fallback={'on' if FOV_VIDEO_INCLUDE_FALLBACK_DETECTIONS else 'off'}, "
        f"min_hist={FOV_VIDEO_MIN_TRACK_HISTORY_FRAMES})"
    )
    save_aoa_animation(
        timestamps_s=sampled_timestamps_s,
        track_aoas_per_frame=track_aoas_per_frame,
        range_angle_grids=range_angle_grids,
        angle_axis_deg=angle_axis_deg,
        range_axis_m=range_axis_m,
        output_path=output_path,
        fov_limit_deg=FOV_VIDEO_FOV_DEG,
        fps=FOV_VIDEO_FPS,
    )
    return output_path


def _consolidate_tracks(tracks: list, gate_m: float = 0.45, merge_gap_s: float = 0.5) -> list:
    """Merge temporally adjacent fragments from the same physical target.

    Two tracks are merged when the gap between the end of one and the start of
    the next is <= merge_gap_s, and the range extrapolated forward (using the
    last-few-points velocity) lands within gate_m of the next track's start.
    """
    if len(tracks) <= 1:
        return list(tracks)

    sorted_tracks = sorted(tracks, key=lambda t: t.history_t[0] if t.history_t else 0.0)
    merged: list = []

    for track in sorted_tracks:
        if not track.history_t:
            continue
        t_start = track.history_t[0]
        r_start = track.history_m[0]

        best = None
        best_err = np.inf
        for sup in merged:
            t_end = sup.history_t[-1]
            time_gap = t_start - t_end
            if not (0.0 <= time_gap <= merge_gap_s):
                continue
            # Estimate velocity from last 3+ points of super-track
            if len(sup.history_m) >= 3:
                dt = sup.history_t[-1] - sup.history_t[-3]
                dr = sup.history_m[-1] - sup.history_m[-3]
                vel = (dr / dt) if dt > 1e-6 else 0.0
            else:
                vel = 0.0
            pred_r = sup.history_m[-1] + vel * time_gap
            err = abs(pred_r - r_start)
            if err <= gate_m and err < best_err:
                best_err = err
                best = sup

        if best is not None:
            best.history_t.extend(track.history_t)
            best.history_m.extend(track.history_m)
        else:
            merged.append(track)

    return merged


def _per_path_clutter_mag(preprocessed, path_idx: int) -> np.ndarray:
    frames = preprocessed.session.frames
    mean = frames.mean(axis=0, keepdims=True)
    clutter = np.abs(frames - mean).astype(np.float32)
    wall = preprocessed.range_axis_m >= preprocessed.config.wall_clip_m
    out = clutter[:, path_idx, :].copy()
    out[:, ~wall] = 0.0
    return out


def main() -> None:
    bag_path = _ROOT / "uwb_rosbags" / BAG
    if not bag_path.exists():
        sys.exit(f"Bag not found: {bag_path}")

    out_dir = Path(OUTPUT_DIR) if OUTPUT_DIR else bag_path / "analysis"
    out_dir.mkdir(parents=True, exist_ok=True)

    _log_file = (out_dir / "run.log").open("w")
    sys.stdout = _Tee(sys.__stdout__, _log_file)

    ann_path = None
    if ANNOTATION:
        ann_path = _ROOT / "ground_truth" / "annotations" / ANNOTATION
        if not ann_path.exists():
            ann_path = _ROOT / "ground_truth" / "annotations" / f"{ANNOTATION}.yaml"
        if not ann_path.exists():
            print(f"[warn] annotation not found: {ANNOTATION}, skipping windows")
            ann_path = None

    print(f"\n{'='*60}")
    print(f"  bag        : {BAG}")
    print(f"  output     : {out_dir}")
    print(f"  annotation : {ann_path.name if ann_path else 'none'}")
    print(f"  breathing  : {'on' if ENABLE_BREATHING else 'off'}")
    print(f"  dual merge : {'on' if ENABLE_DUAL_MERGE else 'off'}")
    if ENABLE_FOV_VIDEO:
        print(
            "  fov video  : on  "
            f"(stride={FOV_VIDEO_FRAME_STRIDE}, bins={FOV_VIDEO_ANGLE_BINS}, "
            f"fallback={'on' if FOV_VIDEO_INCLUDE_FALLBACK_DETECTIONS else 'off'}, "
            f"min_hist={FOV_VIDEO_MIN_TRACK_HISTORY_FRAMES})"
        )
    else:
        print(f"  fov video  : off")
    print(f"{'='*60}\n")

    t0 = time.time()

    # ── 1. Load + preprocess ─────────────────────────────────────────────────
    print("Loading session...")
    session = load_session(bag_path, topic=TOPIC)
    config = DetectionConfig(
        tap_spacing_m=0.15,
        default_range_gate_m=(RANGE_MIN_M, RANGE_MAX_M),
        wall_clip_m=RANGE_MIN_M,
        doppler_window_s=DOPPLER_WINDOW_S,
        microdoppler_window_s=MICRODOPPLER_WINDOW_S,
        stft_overlap=STFT_OVERLAP,
    )
    preprocessed = preprocess_session(session, config)
    frr = preprocessed.frame_rate_hz
    print(f"  {session.frames.shape[0]} frames  |  {session.frames.shape[1]} paths  "
          f"|  {session.frames.shape[2]} taps  |  {frr:.1f} Hz")

    # ── 2. Session-level plots ────────────────────────────────────────────────
    print("Saving range-time and range-Doppler plots...")
    save_range_time_plot(preprocessed, out_dir / "range_time.png")
    save_range_doppler_heatmap(preprocessed, out_dir / "range_doppler.png")

    # ── 3. CFAR detection ─────────────────────────────────────────────────────
    print("Running CFAR...")
    cfar_cfg = CfarDetectionConfig(
        num_ref_cells=CFAR_NUM_REF,
        num_guard_cells=CFAR_NUM_GUARD,
        pfa=CFAR_PFA,
        max_peaks_per_frame=CFAR_MAX_PEAKS,
        cluster_min_gap_taps=CFAR_CLUSTER_GAP,
    )
    n_paths = session.frames.shape[1]
    if ENABLE_DUAL_MERGE and n_paths >= 2:
        dets_c = run_cfar_session(preprocessed, cfar_cfg,
                                  clutter_mag=_per_path_clutter_mag(preprocessed, 0))
        dets_b = run_cfar_session(preprocessed, cfar_cfg,
                                  clutter_mag=_per_path_clutter_mag(preprocessed, 1))
        detections = [
            merge_dual_channel_peaks(c, b, fi, preprocessed.range_axis_m, MERGE_TOLERANCE_TAPS, NMS_MIN_SEP_TAPS)
            for fi, (c, b) in enumerate(zip(dets_c, dets_b))
        ]
        print(f"  dual-channel merge: {sum(len(d) for d in detections)} total peaks")
    else:
        detections = run_cfar_session(preprocessed, cfar_cfg,
                                      clutter_mag=combine_paths_max(preprocessed))
        print(f"  single-path CFAR: {sum(len(d) for d in detections)} total peaks")

    # ── 4. Kalman tracking ────────────────────────────────────────────────────
    print("Tracking...")
    dt_s = 1.0 / max(frr, 1.0)
    tracker = MultiTargetTracker(TrackingConfig(
        dt_s=dt_s,
        sigma_meas_m=0.15,
        gate_distance_m=TRACKER_GATE_M,
        confirm_hits=TRACKER_CONFIRM_HITS,
        max_misses=TRACKER_MAX_MISSES,
        init_hits=TRACKER_CONFIRM_HITS,
        init_window_frames=TRACKER_CONFIRM_HITS + 2,
    ))
    tracks_per_frame: list[list[Track]] = []
    for fi, dets in enumerate(detections):
        t_s = float(session.timestamps_s[fi]) if fi < len(session.timestamps_s) else fi * dt_s
        active_tracks = tracker.update(fi, t_s, dets)
        tracks_per_frame.append(_snapshot_active_tracks(active_tracks))

    raw_tracks = tracker.tracks_snapshot()
    tracks = _consolidate_tracks(raw_tracks, gate_m=TRACKER_GATE_M, merge_gap_s=1.5)
    n_short = sum(1 for t in tracks if len(t.history_m) < 50)
    long_tracks = [t for t in tracks if len(t.history_m) >= 50]
    print(f"  {len(raw_tracks)} raw → {len(tracks)} consolidated track(s)  "
          f"({n_short} with <50 frames filtered from display)")
    for t in long_tracks:
        print(f"    track {t.track_id}: mean range {np.mean(t.history_m):.2f} m  "
              f"({len(t.history_m)} frames)")

    save_multi_peak_tracking_plot(preprocessed, tracks, out_dir / "peak_tracking.png")

    # ── AoA / fisheye plot ────────────────────────────────────────────────────
    print("Computing AoA...")
    _save_aoa_plot(preprocessed, session, detections, tracks, out_dir)

    fov_video_path = None
    if ENABLE_FOV_VIDEO:
        print("Rendering FOV video...")
        try:
            fov_video_path = _save_fov_video(
                preprocessed,
                session,
                detections,
                tracks_per_frame,
                out_dir,
            )
            if fov_video_path is not None:
                print(f"  saved {fov_video_path.name}")
        except Exception as exc:
            print(f"  [fov] failed: {exc}")

    # ── Multi-person validation (guide §5, §8) ────────────────────────────────
    multi_frames   = [d for d in detections if len(d) >= 2]
    pct_multi      = 100.0 * len(multi_frames) / max(len(detections), 1)
    tap_seps       = []
    for frame_dets in multi_frames:
        taps = sorted(d.tap_idx for d in frame_dets)
        for a, b in zip(taps, taps[1:]):
            tap_seps.append(b - a)

    print(f"\n  -- multi-person validation --")
    print(f"  frames with >=2 peaks : {len(multi_frames)} / {len(detections)}  ({pct_multi:.1f}%)")
    if tap_seps:
        print(f"  tap separation        : min={min(tap_seps)}  median={int(np.median(tap_seps))}  max={max(tap_seps)}")
        ok_sep = sum(1 for s in tap_seps if s >= 2)
        print(f"  separation >= 2 taps  : {ok_sep}/{len(tap_seps)}  "
              f"({'PASS' if ok_sep/len(tap_seps) > 0.9 else 'FAIL -- peaks too close'})")
    if len(long_tracks) >= 2:
        ranges = sorted(float(np.mean(t.history_m)) for t in long_tracks)
        delta_r = preprocessed.config.tap_spacing_m * 2
        for i in range(len(ranges) - 1):
            sep_m = ranges[i+1] - ranges[i]
            status = "PASS" if sep_m >= delta_r else "FAIL -- within one range bin"
            print(f"  track range sep       : {sep_m:.2f} m  ({status}, need >= {delta_r:.2f} m)")
    elif len(long_tracks) == 1:
        print(f"  [warn] only 1 substantial track -- try lowering CFAR_PFA or TRACKER_CONFIRM_HITS")

    # ── 5. Breathing detection ────────────────────────────────────────────────
    breathing_results = []
    if ENABLE_BREATHING and tracks:
        print("Extracting breathing rates...")
        ts = session.timestamps_s
        for track in tracks:
            if len(track.history_t) < 100:
                continue
            # Gate 1: range span — person moved more than 2 taps
            range_span_m = float(np.max(track.history_m)) - float(np.min(track.history_m))
            if range_span_m > 0.30:
                print(f"  track {track.track_id}: MOVING (range span {range_span_m:.2f} m) "
                      f"— skipped")
                breathing_results.append({
                    "track_id": track.track_id, "is_static": False,
                    "freq_hz": None, "freq_bpm": None, "snr_db": None, "phase_arc_deg": None,
                })
                continue
            t_start, t_end = float(min(track.history_t)), float(max(track.history_t))
            frame_mask = (ts >= t_start) & (ts <= t_end)
            tap = int(np.clip(
                round(float(np.mean(track.history_m)) / 0.15),
                0, preprocessed.highpass_complex.shape[1] - 1,
            ))
            br = run_breathing_extraction(
                preprocessed.highpass_complex,
                preprocessed.peak_tap_per_frame,
                frame_mask,
                tap,
                frr,
                nfft=BREATHING_NFFT,
                hp_cutoff_hz=BREATHING_HP_HZ,
                search_band_hz=BREATHING_BAND_HZ,
            )
            # Gate 2: arc > 360° means accumulated phase > 1 full cycle — person is walking
            if br.phase_arc_deg is not None and br.phase_arc_deg > 360.0:
                print(f"  track {track.track_id}: MOVING (arc={br.phase_arc_deg:.0f} deg > 360) "
                      f"— skipped")
                breathing_results.append({
                    "track_id": track.track_id, "is_static": False,
                    "freq_hz": None, "freq_bpm": None, "snr_db": None,
                    "phase_arc_deg": br.phase_arc_deg,
                })
                continue
            if br.freq_bpm is not None:
                print(f"  track {track.track_id} (static): "
                      f"{br.freq_bpm:.1f} BPM  SNR={br.snr_db:.1f} dB  "
                      f"arc={br.phase_arc_deg:.1f} deg")
            else:
                print(f"  track {track.track_id} (static): no breathing peak found")
            _save_breathing_plot(br, track.track_id, frr, out_dir)
            breathing_results.append({
                "track_id": track.track_id,
                "is_static": True,
                "freq_hz": br.freq_hz,
                "freq_bpm": br.freq_bpm,
                "snr_db": br.snr_db,
                "phase_arc_deg": br.phase_arc_deg,
            })
    elif ENABLE_BREATHING:
        print("Breathing: no confirmed tracks to extract from.")

    # ── 6. Per-window annotation analysis ────────────────────────────────────
    window_results = []
    if ann_path is not None:
        annotation = load_annotation_file(ann_path)
        print(f"Processing {len(annotation.windows)} annotation window(s)...")
        for window in annotation.windows:
            window_dir = out_dir / window.label
            window_dir.mkdir(parents=True, exist_ok=True)
            try:
                det_result, extras = detect_window(preprocessed, window, artifact_dir=window_dir)
                save_spectrogram_plot(extras["doppler"],
                                      window_dir / "doppler.png",
                                      frequency_limit_hz=config.doppler_limit_hz)
                save_spectrogram_plot(extras["microdoppler"],
                                      window_dir / "microdoppler.png",
                                      frequency_limit_hz=config.microdoppler_limit_hz)
                status = "PRESENT" if det_result.predicted_present else "absent"
                print(f"  [{window.label}] {status}  "
                      f"range={det_result.dominant_range_m:.2f}m  "
                      f"score={det_result.presence_score:.1f}")
                window_results.append(det_result.to_dict())
            except (ValueError, RuntimeError) as exc:
                print(f"  [{window.label}] ERROR: {exc}")

    # ── 7. Save detections.csv and summary.json ───────────────────────────────
    import csv
    with (out_dir / "detections.csv").open("w", newline="") as f:
        writer = csv.DictWriter(
            f, fieldnames=["frame_idx", "timestamp_s", "tap_idx", "range_m", "magnitude", "threshold"]
        )
        writer.writeheader()
        for fi, dets in enumerate(detections):
            t_s = float(session.timestamps_s[fi]) if fi < len(session.timestamps_s) else fi * dt_s
            for d in dets:
                writer.writerow(dict(
                    frame_idx=d.frame_idx, timestamp_s=round(t_s, 6),
                    tap_idx=d.tap_idx, range_m=round(d.range_m, 4),
                    magnitude=round(d.magnitude, 6), threshold=round(d.threshold, 6),
                ))

    summary = {
        "bag": BAG,
        "frame_count": int(session.frames.shape[0]),
        "frame_rate_hz": float(frr),
        "duration_s": float(session.timestamps_s[-1]) if session.timestamps_s.size > 0 else 0.0,
        "selected_path": int(preprocessed.selected_path),
        "dominant_tap": int(preprocessed.dominant_tap),
        "dominant_range_m": float(preprocessed.range_axis_m[preprocessed.dominant_tap]),
        "confirmed_tracks": [
            {"track_id": t.track_id,
             "mean_range_m": round(float(np.mean(t.history_m)), 3),
             "frames": len(t.history_m)}
            for t in tracks
        ],
        "fov_video": fov_video_path.name if fov_video_path is not None else None,
        "fov_video_frame_stride": int(FOV_VIDEO_FRAME_STRIDE) if fov_video_path is not None else None,
        "fov_video_include_fallback_detections": (
            bool(FOV_VIDEO_INCLUDE_FALLBACK_DETECTIONS) if fov_video_path is not None else None
        ),
        "fov_video_min_track_history_frames": (
            int(FOV_VIDEO_MIN_TRACK_HISTORY_FRAMES) if fov_video_path is not None else None
        ),
        "breathing": breathing_results,
        "detections": window_results,
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2))

    elapsed = time.time() - t0
    print(f"\nDone in {elapsed:.1f}s — outputs in {out_dir}\n")

    sys.stdout = sys.__stdout__
    _log_file.close()


if __name__ == "__main__":
    main()
