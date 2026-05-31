from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
import numpy as np

from .aoa import TrackAoA
from .types import CfarDetection, PreprocessedSession, SpectrogramResult, Track
from .visualization_utils import (
    FOV_BACKGROUND,
    FOV_GRID,
    FOV_SECTOR_EDGE,
    FOV_SECTOR_FILL,
    FOV_TEXT,
    db_limits,
    jet_color,
    magnitude_to_db,
    power_ratio_to_db,
    track_color,
    with_alpha,
)


def save_range_time_plot(preprocessed: PreprocessedSession, output_path: Path) -> None:
    frame_axis = preprocessed.session.timestamps_s

    # Crop to bins at or beyond wall_clip_m.  Bins below this threshold were
    # zeroed in preprocessing; including them would show 20*log10(1e-6)≈-120 dB
    # as a solid dark band and collapse the colormap range for the real signal.
    full_range = preprocessed.range_axis_m
    valid = full_range >= preprocessed.config.wall_clip_m
    range_axis = full_range[valid]

    raw = magnitude_to_db(preprocessed.raw_magnitude[:, valid]).T
    clutter = magnitude_to_db(preprocessed.clutter_removed[:, valid]).T
    raw_vmin, raw_vmax = db_limits(raw)
    clutter_vmin, clutter_vmax = db_limits(clutter)

    fig, axes = plt.subplots(2, 1, figsize=(12, 9), sharex=True)

    raw_im = axes[0].imshow(
        raw,
        aspect="auto",
        origin="lower",
        extent=[frame_axis[0], frame_axis[-1], range_axis[0], range_axis[-1]],
        cmap="jet",
        vmin=raw_vmin,
        vmax=raw_vmax,
    )
    axes[0].set_title(f"Raw range-time magnitude (path {preprocessed.selected_path}, dB)")
    axes[0].set_ylabel("Range (m)")
    fig.colorbar(raw_im, ax=axes[0], label="Magnitude (dB)")

    im = axes[1].imshow(
        clutter,
        aspect="auto",
        origin="lower",
        extent=[frame_axis[0], frame_axis[-1], range_axis[0], range_axis[-1]],
        cmap="jet",
        vmin=clutter_vmin,
        vmax=clutter_vmax,
    )
    axes[1].set_title("Clutter-suppressed range-time magnitude (dB)")
    axes[1].set_ylabel("Range (m)")
    axes[1].set_xlabel("Time (s)")
    fig.colorbar(im, ax=axes[1], label="Magnitude (dB)")

    plt.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def save_peak_tracking_plot(preprocessed: PreprocessedSession, output_path: Path) -> None:
    peak_taps = np.argmax(preprocessed.clutter_removed, axis=1)
    peak_ranges = preprocessed.range_axis_m[peak_taps]

    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(preprocessed.session.timestamps_s, peak_ranges, linewidth=1.0, color=jet_color(0, 2))
    ax.set_title("Dominant moving reflector range over time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Range (m)")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def save_presence_score_plot(preprocessed: PreprocessedSession, output_path: Path) -> None:
    timestamps = preprocessed.session.timestamps_s
    scores_db = power_ratio_to_db(preprocessed.roi_to_background_power_ratio)
    threshold_db = float(power_ratio_to_db(np.asarray([preprocessed.presence_threshold], dtype=np.float32))[0])
    states = preprocessed.presence_mask.astype(np.float32)

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True, height_ratios=(3, 1))

    axes[0].plot(
        timestamps,
        scores_db,
        color=jet_color(1, 5),
        linewidth=1.4,
        label="ROI/background power ratio (dB)",
    )
    axes[0].axhline(
        threshold_db,
        color=jet_color(4, 5),
        linestyle="--",
        linewidth=1.2,
        label=f"Threshold = {threshold_db:.2f} dB",
    )
    axes[0].fill_between(
        timestamps,
        np.minimum(scores_db, threshold_db),
        scores_db,
        where=preprocessed.presence_mask,
        color=jet_color(2, 5),
        alpha=0.35,
        interpolate=True,
        label="Monitoring state",
    )
    axes[0].set_title("Presence checking to monitoring transition")
    axes[0].set_ylabel("ROI / background power (dB)")
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(loc="upper right")

    axes[1].step(timestamps, states, where="post", color=jet_color(3, 5), linewidth=1.6)
    axes[1].fill_between(timestamps, 0.0, states, step="post", color=jet_color(3, 5), alpha=0.25)
    axes[1].set_yticks([0.0, 1.0], labels=["checking", "monitoring"])
    axes[1].set_ylim(-0.1, 1.1)
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Mode")
    axes[1].grid(True, alpha=0.25)

    plt.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def save_presence_monitoring_plot(preprocessed: PreprocessedSession, output_path: Path) -> None:
    timestamps = preprocessed.session.timestamps_s
    peak_ranges = getattr(
        preprocessed,
        "peak_range_centroid_m_per_frame",
        preprocessed.peak_range_m_per_frame,
    )
    smoothed_ranges = preprocessed.smoothed_peak_range_m
    active = preprocessed.presence_mask

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(
        timestamps,
        peak_ranges,
        color=jet_color(1, 4),
        linewidth=0.9,
        alpha=0.85,
        label="Instantaneous dominant range",
    )
    ax.plot(
        timestamps[active],
        smoothed_ranges[active],
        color=jet_color(3, 4),
        linewidth=2.0,
        label="Monitoring trace (smoothed)",
    )
    ax.scatter(
        timestamps[active],
        peak_ranges[active],
        s=10,
        color=jet_color(2, 4),
        alpha=0.55,
        label="Detected presence",
    )
    ax.set_title("Presence monitoring trace")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Range (m)")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right")

    plt.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def _decorate_fov_axis(ax, fov_limit_deg: float, range_max_m: float, title: str) -> None:
    theta_deg = np.linspace(-fov_limit_deg, fov_limit_deg, 241, dtype=np.float32)
    theta_rad = np.deg2rad(theta_deg)
    theta_poly = np.concatenate(([theta_rad[0]], theta_rad, [theta_rad[-1]]))
    radius_poly = np.concatenate(([0.0], np.full(theta_rad.shape, range_max_m), [0.0]))

    ax.set_facecolor(FOV_BACKGROUND)
    ax.fill(theta_poly, radius_poly, color=FOV_SECTOR_FILL, alpha=0.95, zorder=0)
    ax.plot(theta_rad, np.full(theta_rad.shape, range_max_m), color=FOV_SECTOR_EDGE, linewidth=1.4, zorder=1)
    ax.plot([theta_rad[0], theta_rad[0]], [0.0, range_max_m], color=FOV_SECTOR_EDGE, linewidth=1.2, linestyle="--", zorder=1)
    ax.plot([theta_rad[-1], theta_rad[-1]], [0.0, range_max_m], color=FOV_SECTOR_EDGE, linewidth=1.2, linestyle="--", zorder=1)
    ax.plot([0.0, 0.0], [0.0, range_max_m], color=FOV_SECTOR_EDGE, linewidth=1.0, linestyle=":", alpha=0.85, zorder=1)

    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.set_thetamin(-fov_limit_deg)
    ax.set_thetamax(fov_limit_deg)
    ax.set_rlim(0.0, range_max_m)
    ax.set_title(title, pad=18, fontsize=12, color=FOV_TEXT)

    r_ticks = np.arange(1.0, range_max_m + 0.01, 1.0)
    ax.set_rticks(r_ticks.tolist())
    ax.set_rlabel_position(fov_limit_deg * 0.62)
    ax.yaxis.set_tick_params(labelsize=8, colors=FOV_TEXT)
    ax.set_thetagrids(np.arange(-fov_limit_deg, fov_limit_deg + 1, 15.0).tolist(), fontsize=8)
    for label in ax.get_xticklabels():
        label.set_color(FOV_TEXT)
    ax.grid(True, alpha=0.45, linewidth=0.7, color=FOV_GRID)
    ax.spines["polar"].set_color(FOV_SECTOR_EDGE)
    ax.spines["polar"].set_linewidth(1.0)


def save_all_cfar_peaks_tap_plot(
    preprocessed: PreprocessedSession,
    detections_per_frame: list[list[CfarDetection]],
    output_path: Path,
) -> None:
    timestamps = preprocessed.session.timestamps_s
    det_t: list[float] = []
    det_tap: list[int] = []
    det_mag_db: list[float] = []

    for frame_idx, detections in enumerate(detections_per_frame):
        if frame_idx >= len(timestamps):
            break
        for detection in detections:
            det_t.append(float(timestamps[frame_idx]))
            det_tap.append(int(detection.tap_idx))
            det_mag_db.append(float(magnitude_to_db(np.asarray([detection.magnitude], dtype=np.float32))[0]))

    fig, ax = plt.subplots(figsize=(12, 5))
    if det_t:
        scatter = ax.scatter(
            det_t,
            det_tap,
            c=det_mag_db,
            s=16,
            cmap="jet",
            alpha=0.8,
            linewidths=0,
        )
        fig.colorbar(scatter, ax=ax, label="CFAR magnitude (dB)")
    ax.set_title(f"All CFAR peaks over time by tap (path {preprocessed.selected_path})")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Tap index")
    ax.grid(True, alpha=0.25)
    plt.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def save_range_doppler_plot(preprocessed: PreprocessedSession, output_path: Path) -> None:
    signal = preprocessed.highpass_complex  # (N, K)
    N = signal.shape[0]

    window = np.hanning(N).reshape(-1, 1)
    rdm = np.fft.fftshift(np.fft.fft(signal * window, axis=0), axes=0)
    rdm_db = magnitude_to_db(np.abs(rdm))

    doppler_hz = np.fft.fftshift(np.fft.fftfreq(N, d=1.0 / preprocessed.frame_rate_hz))
    c = 3e8
    fc = preprocessed.config.carrier_frequency_hz
    velocity_ms = doppler_hz * c / (2.0 * fc)

    range_axis = preprocessed.range_axis_m
    valid = range_axis >= preprocessed.config.wall_clip_m
    vmin, vmax = db_limits(rdm_db[:, valid].T)

    fig, ax = plt.subplots(figsize=(11, 6))
    im = ax.imshow(
        rdm_db[:, valid].T,
        aspect="auto",
        origin="lower",
        extent=[velocity_ms[0], velocity_ms[-1], range_axis[valid][0], range_axis[valid][-1]],
        cmap="jet",
        vmin=vmin,
        vmax=vmax,
    )
    ax.axvline(0, color="white", linewidth=0.6, alpha=0.5, linestyle="--")
    ax.set_title(f"Range-Doppler map (path {preprocessed.selected_path}, fc={fc/1e9:.2f} GHz)")
    ax.set_xlabel("Radial velocity (m/s)  [negative = approaching, positive = receding]")
    ax.set_ylabel("Range (m)")
    fig.colorbar(im, ax=ax, label="Magnitude (dB)")
    plt.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def _estimate_nominal_dt(times: list[float]) -> float:
    if len(times) < 2:
        return 0.0
    diffs = [float(t1 - t0) for t0, t1 in zip(times[:-1], times[1:]) if float(t1 - t0) > 0.0]
    if not diffs:
        return 0.0
    return float(np.median(np.asarray(diffs, dtype=np.float64)))


def _iter_true_runs(
    mask: list[bool],
    times: list[float] | None = None,
    max_step_s: float | None = None,
) -> list[tuple[int, int]]:
    runs: list[tuple[int, int]] = []
    start: int | None = None
    for idx, value in enumerate(mask):
        if value and start is None:
            start = idx
            continue
        if (
            value
            and start is not None
            and times is not None
            and max_step_s is not None
            and idx > 0
            and float(times[idx] - times[idx - 1]) > float(max_step_s)
        ):
            runs.append((start, idx))
            start = idx
        elif not value and start is not None:
            runs.append((start, idx))
            start = None
    if start is not None:
        runs.append((start, len(mask)))
    return runs


def _iter_bridge_segments(
    times: list[float],
    observed_mask: list[bool],
    max_bridge_gap_s: float,
    max_step_s: float | None = None,
) -> list[tuple[int, int]]:
    runs = _iter_true_runs(observed_mask, times=times, max_step_s=max_step_s)
    bridges: list[tuple[int, int]] = []
    for (prev_start, prev_end), (next_start, next_end) in zip(runs, runs[1:]):
        del prev_start, next_end
        last_idx = prev_end - 1
        first_idx = next_start
        if last_idx < 0 or first_idx >= len(times):
            continue
        gap_s = float(times[first_idx] - times[last_idx])
        if 0.0 < gap_s <= float(max_bridge_gap_s):
            bridges.append((last_idx, first_idx))
    return bridges


def save_multi_peak_tracking_plot(
    preprocessed: PreprocessedSession,
    confirmed_tracks: list[Track],
    output_path: Path,
    cfar_variant: str = "CA",
    tracker_label: str = "Kalman",
) -> None:
    """Range trajectories of all confirmed CFAR tracks over time."""
    fig, ax = plt.subplots(figsize=(12, 5))
    track_count = max(len(confirmed_tracks), 1)

    for i, track in enumerate(confirmed_tracks):
        traj_t = track.trajectory_t if len(track.trajectory_t) >= 2 else track.history_t
        traj_m = track.trajectory_m if len(track.trajectory_m) >= 2 else track.history_m
        traj_observed = (
            track.trajectory_observed
            if len(track.trajectory_observed) == len(traj_t)
            else [True] * len(traj_t)
        )
        if len(traj_t) < 2:
            continue
        color = jet_color(i, track_count)
        label = f"track {track.track_id}"
        nominal_dt_s = _estimate_nominal_dt(traj_t)
        max_step_s = (1.75 * nominal_dt_s) if nominal_dt_s > 0.0 else None
        bridge_gap_s = max(0.75, 3.0 * nominal_dt_s) if nominal_dt_s > 0.0 else 0.75
        label_drawn = False
        for start, end in _iter_true_runs(traj_observed, times=traj_t, max_step_s=max_step_s):
            if end - start >= 2:
                ax.plot(
                    traj_t[start:end],
                    traj_m[start:end],
                    color=color,
                    linewidth=1.8,
                    # label=label if not label_drawn else None,
                )
                label_drawn = True
            elif start < len(traj_t):
                ax.scatter(
                    [traj_t[start]],
                    [traj_m[start]],
                    s=12,
                    color=[color],
                    alpha=0.85,
                    # label=label if not label_drawn else None,
                )
                label_drawn = True
        for start, end in _iter_bridge_segments(
            traj_t,
            traj_observed,
            max_bridge_gap_s=bridge_gap_s,
            max_step_s=max_step_s,
        ):
            ax.plot(
                traj_t[start : end + 1],
                traj_m[start : end + 1],
                color=color,
                linewidth=1.2,
                linestyle=":",
                alpha=0.85,
            )

    ax.set_title(f"Confirmed track trajectories ({cfar_variant}-CFAR + {tracker_label})")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Range (m)")
    ax.grid(True, alpha=0.3)
    if confirmed_tracks:
        ax.legend(loc="upper right", fontsize=8)
    plt.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def save_spectrogram_plot(
    spectrogram: SpectrogramResult,
    output_path: Path,
    frequency_limit_hz: float,
) -> None:
    fig, ax = plt.subplots(figsize=(11, 5))
    valid = np.abs(spectrogram.frequencies_hz) <= frequency_limit_hz
    freq = spectrogram.frequencies_hz[valid]
    mag = spectrogram.magnitude_db[valid]
    vmin, vmax = db_limits(mag)
    mesh = ax.pcolormesh(
        spectrogram.times_s,
        freq,
        mag,
        shading="auto",
        cmap="jet",
        vmin=vmin,
        vmax=vmax,
    )
    kind_label = "Phase micro-Doppler" if spectrogram.kind == "microdoppler" else spectrogram.kind.capitalize()
    ax.set_title(
        f"{kind_label} spectrogram "
        f"(path {spectrogram.selected_path}, tap {spectrogram.selected_tap}, "
        f"range {spectrogram.selected_range_m:.2f} m)"
    )
    ax.set_xlabel("Slow time (s)")
    ax.set_ylabel("Frequency (Hz)")
    fig.colorbar(mesh, ax=ax, label="Magnitude (dB)")
    plt.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Scientific Range-Doppler heatmap (jet, range on X, velocity on Y)
# ---------------------------------------------------------------------------

def _compute_rdm(
    signal: np.ndarray,
    frame_rate_hz: float,
    carrier_frequency_hz: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (rdm_db, range_valid_mask, velocity_ms) for a (N, K) complex signal."""
    N = signal.shape[0]
    win = np.hanning(N).reshape(-1, 1)
    rdm = np.fft.fftshift(np.fft.fft(signal * win, axis=0), axes=0)
    rdm_db = magnitude_to_db(np.abs(rdm))
    doppler_hz = np.fft.fftshift(np.fft.fftfreq(N, d=1.0 / frame_rate_hz))
    velocity_ms = doppler_hz * 3e8 / (2.0 * carrier_frequency_hz)
    return rdm_db, velocity_ms


def save_range_doppler_heatmap(
    preprocessed: PreprocessedSession,
    output_path: Path,
    vel_lim_ms: float = 3.0,
) -> None:
    """Scientific Range-Doppler heatmap: range on X, velocity on Y, jet colormap.

    Uses the full session FFT (all frames) to give a time-integrated view of
    target Doppler.  Saved as a publication-quality PNG.

    Parameters
    ----------
    vel_lim_ms:
        Half-range of the velocity axis in m/s (default 3.0 → shows −3 to +3).
        Clamped to the actual Nyquist velocity of the data if the data does not
        reach that speed.
    """
    import matplotlib.ticker as ticker

    signal = preprocessed.highpass_complex          # (N, K)
    range_axis = preprocessed.range_axis_m
    valid = range_axis >= preprocessed.config.wall_clip_m
    range_valid = range_axis[valid]

    rdm_db, velocity_ms = _compute_rdm(
        signal, preprocessed.frame_rate_hz, preprocessed.config.carrier_frequency_hz
    )
    # rdm_db shape: (N, K) — rows=Doppler bins, cols=range taps
    map_data = rdm_db[:, valid]                     # (N_doppler, N_range_valid)

    # clamp colour scale to [-20, peak] so faint targets stay visible
    vmin, vmax = db_limits(map_data)

    fig, ax = plt.subplots(figsize=(9, 5.5), facecolor="white")

    img = ax.pcolormesh(
        range_valid,
        velocity_ms,
        map_data,
        cmap="jet",
        vmin=vmin,
        vmax=vmax,
        shading="auto",
        rasterized=True,
    )

    cbar = fig.colorbar(img, ax=ax, pad=0.02)
    cbar.set_label("Power (dB)", fontsize=11)
    cbar.ax.tick_params(labelsize=9)

    v_lo = max(velocity_ms[0],  -vel_lim_ms)
    v_hi = min(velocity_ms[-1], +vel_lim_ms)

    ax.axhline(0, color="white", linewidth=0.7, linestyle="--", alpha=0.45)
    ax.set_xlabel("Range (m)", fontsize=12, color="black")
    ax.set_ylabel("Velocity (m/s)", fontsize=12, color="black")
    ax.set_xlim(range_valid[0], range_valid[-1])
    ax.set_ylim(v_lo, v_hi)
    ax.xaxis.set_major_locator(ticker.MultipleLocator(1.0))
    ax.xaxis.set_minor_locator(ticker.MultipleLocator(0.5))
    ax.yaxis.set_major_locator(ticker.MultipleLocator(0.5))
    ax.yaxis.set_minor_locator(ticker.MultipleLocator(0.25))
    ax.tick_params(axis="y", which="major", labelsize=9, color="black", labelcolor="black", length=5)
    ax.tick_params(axis="y", which="minor", labelsize=0,  color="black", length=3)
    ax.tick_params(axis="x", which="both",  labelsize=9, color="black", labelcolor="black")
    ax.yaxis.grid(True, which="major", color="white", linewidth=0.4, alpha=0.25, linestyle="-")
    ax.yaxis.grid(True, which="minor", color="white", linewidth=0.2, alpha=0.12, linestyle=":")
    ax.set_axisbelow(False)
    for spine in ax.spines.values():
        spine.set_edgecolor("black")

    fc_ghz = preprocessed.config.carrier_frequency_hz / 1e9
    ax.set_title(
        f"Range-Doppler Map  (path {preprocessed.selected_path}, "
        f"fc = {fc_ghz:.2f} GHz)",
        fontsize=12, color="black",
    )
    fig.text(
        0.5, 0.01,
        "Range-Doppler map showing two targets and Doppler ambiguity replicas.",
        ha="center", va="bottom", fontsize=9, color="#444444", style="italic",
    )

    fig.tight_layout(rect=[0, 0.04, 1, 1])
    fig.savefig(output_path, dpi=180, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def save_range_doppler_heatmap_animation(
    preprocessed: PreprocessedSession,
    output_path: Path,
    window_frames: int | None = None,
    step_frames: int | None = None,
    fps: int = 15,
    vel_lim_ms: float = 3.0,
) -> None:
    """Animated Range-Doppler map using a sliding window through the session.

    Encodes directly to MP4 via OpenCV VideoWriter (no ffmpeg binary required).

    Parameters
    ----------
    window_frames:
        FFT window length in frames.  Defaults to min(256, N//4), floor 32.
    step_frames:
        Frames to advance between animation frames.  Defaults to window//8.
    fps:
        Output video frame rate.
    vel_lim_ms:
        Half-range of the velocity axis in m/s (default 3.0 → shows −3 to +3).
    """
    import cv2
    import matplotlib.ticker as ticker

    signal = preprocessed.highpass_complex          # (N, K)
    N = signal.shape[0]
    range_axis = preprocessed.range_axis_m
    valid = range_axis >= preprocessed.config.wall_clip_m
    range_valid = range_axis[valid]
    timestamps = preprocessed.session.timestamps_s

    # window / step sizing
    win_len = window_frames or max(32, min(256, N // 4))
    win_len = min(win_len, N)
    step = step_frames or max(1, win_len // 8)

    # pre-compute colour limits from a representative middle window
    mid = max(0, (N - win_len) // 2)
    rdm_mid, velocity_ms = _compute_rdm(
        signal[mid: mid + win_len],
        preprocessed.frame_rate_hz,
        preprocessed.config.carrier_frequency_hz,
    )
    map_mid = rdm_mid[:, valid]
    vmin, vmax = db_limits(map_mid)

    # --- build figure once; reuse for each frame ---
    DPI = 120
    fig, ax = plt.subplots(figsize=(9, 5.5), facecolor="white")

    img = ax.pcolormesh(
        range_valid,
        velocity_ms,
        map_mid,
        cmap="jet",
        vmin=vmin,
        vmax=vmax,
        shading="auto",
        rasterized=True,
    )
    cbar = fig.colorbar(img, ax=ax, pad=0.02)
    cbar.set_label("Power (dB)", fontsize=11)
    cbar.ax.tick_params(labelsize=9)

    v_lo = max(velocity_ms[0],  -vel_lim_ms)
    v_hi = min(velocity_ms[-1], +vel_lim_ms)

    ax.axhline(0, color="white", linewidth=0.7, linestyle="--", alpha=0.45)
    ax.set_xlabel("Range (m)", fontsize=12, color="black")
    ax.set_ylabel("Velocity (m/s)", fontsize=12, color="black")
    ax.set_xlim(range_valid[0], range_valid[-1])
    ax.set_ylim(v_lo, v_hi)
    ax.xaxis.set_major_locator(ticker.MultipleLocator(1.0))
    ax.xaxis.set_minor_locator(ticker.MultipleLocator(0.5))
    ax.yaxis.set_major_locator(ticker.MultipleLocator(0.5))
    ax.yaxis.set_minor_locator(ticker.MultipleLocator(0.25))
    ax.tick_params(axis="y", which="major", labelsize=9, color="black", labelcolor="black", length=5)
    ax.tick_params(axis="y", which="minor", labelsize=0,  color="black", length=3)
    ax.tick_params(axis="x", which="both",  labelsize=9, color="black", labelcolor="black")
    ax.yaxis.grid(True, which="major", color="white", linewidth=0.4, alpha=0.25, linestyle="-")
    ax.yaxis.grid(True, which="minor", color="white", linewidth=0.2, alpha=0.12, linestyle=":")
    ax.set_axisbelow(False)
    for spine in ax.spines.values():
        spine.set_edgecolor("black")

    title = ax.set_title("Range-Doppler Map Over Time - Frame 001", fontsize=12, color="black")
    fig.text(
        0.5, 0.01,
        "Range-Doppler map showing two targets and Doppler ambiguity replicas.",
        ha="center", va="bottom", fontsize=9, color="#444444", style="italic",
    )
    fig.tight_layout(rect=[0, 0.04, 1, 1])

    # determine output pixel dimensions (must be even for mp4v codec)
    fig.canvas.draw()
    buf = fig.canvas.buffer_rgba()
    frame0_rgba = np.asarray(buf)
    h, w = frame0_rgba.shape[:2]
    h = h if h % 2 == 0 else h - 1
    w = w if w % 2 == 0 else w - 1

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(output_path), fourcc, fps, (w, h))

    starts = list(range(0, N - win_len + 1, step))
    n_anim = len(starts)

    for anim_idx, start in enumerate(starts):
        chunk = signal[start: start + win_len]
        rdm_db, _ = _compute_rdm(
            chunk,
            preprocessed.frame_rate_hz,
            preprocessed.config.carrier_frequency_hz,
        )
        img.set_array(rdm_db[:, valid].ravel())

        t_centre = float(timestamps[start + win_len // 2]) if (start + win_len // 2) < len(timestamps) else float(start + win_len // 2) / preprocessed.frame_rate_hz
        title.set_text(
            f"Range-Doppler Map Over Time - Frame {anim_idx + 1:03d}  "
            f"(t = {t_centre:.2f} s)"
        )
        fig.canvas.draw()

        buf = fig.canvas.buffer_rgba()
        rgba = np.asarray(buf)[:h, :w, :3]
        bgr = rgba[:, :, ::-1].copy()
        writer.write(bgr)

        if (anim_idx + 1) % 20 == 0 or anim_idx == n_anim - 1:
            print(f"  [rdm anim] frame {anim_idx + 1:03d}/{n_anim}")

    writer.release()
    plt.close(fig)


def save_fov_polar_plot(
    track_aoas_per_frame: list[list[TrackAoA]],
    fov_limit_deg: float,
    output_path: Path,
    range_max_m: float = 7.5,
) -> None:
    """Static polar-sector FOV plot aggregating all frame track AoA positions.

    Shows angle (horizontal axis) vs. range (radial) up to range_max_m for
    every confirmed track across the full session.
    """
    track_points: dict[int, tuple[list[float], list[float]]] = {}
    for frame_aoas in track_aoas_per_frame:
        for ta in frame_aoas:
            if ta.range_m > range_max_m:
                continue
            if ta.track_id not in track_points:
                track_points[ta.track_id] = ([], [])
            track_points[ta.track_id][0].append(float(np.deg2rad(ta.angle_deg)))
            track_points[ta.track_id][1].append(float(ta.range_m))

    fig = plt.figure(figsize=(8, 7), facecolor="white")
    ax = fig.add_subplot(111, projection="polar")
    _decorate_fov_axis(
        ax,
        fov_limit_deg=fov_limit_deg,
        range_max_m=range_max_m,
        title=f"Field of View  (+/-{fov_limit_deg:.0f} deg, {range_max_m:.1f} m range)",
    )

    if not track_points:
        ax.text(
            0.5, 0.5, "No tracks in FOV",
            transform=ax.transAxes, ha="center", va="center",
            fontsize=11, color=FOV_TEXT,
        )
    else:
        for tid, (thetas, ranges) in sorted(track_points.items()):
            color = track_color(tid)
            ax.plot(thetas, ranges, linewidth=1.4, color=color, alpha=0.55, zorder=3)
            ax.scatter(thetas, ranges, s=10, color=[with_alpha(color, 0.20)], linewidths=0, zorder=2)
            med_theta = float(np.median(thetas))
            med_range = float(np.median(ranges))
            ax.scatter(
                [med_theta],
                [med_range],
                s=90,
                color=[color],
                alpha=1.0,
                linewidths=1.0,
                edgecolors="black",
                label=f"track {tid}  {np.degrees(med_theta):.0f} deg  {med_range:.1f} m",
                zorder=5,
            )
            label = ax.text(
                med_theta,
                min(range_max_m, med_range + 0.22),
                f"T{tid}",
                color=FOV_TEXT,
                fontsize=8,
                ha="center",
                va="bottom",
                zorder=6,
            )
            label.set_path_effects([pe.withStroke(linewidth=2.5, foreground="white")])

    fig.tight_layout()
    fig.savefig(output_path, dpi=160, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def save_fov_animation(
    track_aoas_per_frame: list[list[TrackAoA]],
    timestamps_s: np.ndarray,
    fov_limit_deg: float,
    output_path: Path,
    range_max_m: float = 7.5,
    fps: int = 15,
    trail_s: float = 1.5,
) -> None:
    """Animate the polar FOV sector frame-by-frame with a fading trail, saved as MP4."""
    import imageio.v2 as imageio

    output_path = Path(output_path)
    partial = output_path.with_name(f"{output_path.stem}.partial{output_path.suffix}")
    if partial.exists():
        partial.unlink()

    n_frames = len(track_aoas_per_frame)
    all_ids = [ta.track_id for frame in track_aoas_per_frame for ta in frame]
    unique_ids = sorted(set(all_ids)) if all_ids else [0]

    dt = float(timestamps_s[1] - timestamps_s[0]) if len(timestamps_s) > 1 else 0.05
    # Stride: only render every Nth source frame to match the requested output fps.
    # A 500 Hz bag rendered at 15 fps output needs stride = 500/15 ≈ 33.
    stride = max(1, round(1.0 / (dt * fps)))
    trail_frames = max(1, int(trail_s / dt))    # in source-frame units

    # ------------------------------------------------------------------
    # Build figure ONCE — only scatter artists change per frame.
    # This avoids figure allocation / axes setup overhead on every frame,
    # giving ~10x speedup over creating a new figure per frame.
    # ------------------------------------------------------------------
    fig = plt.figure(figsize=(7, 7), facecolor="white")
    ax = fig.add_subplot(111, projection="polar")
    _decorate_fov_axis(
        ax,
        fov_limit_deg=fov_limit_deg,
        range_max_m=range_max_m,
        title="FOV  t = 0.00 s",
    )
    title = ax.title
    fig.tight_layout()
    fig.canvas.draw()

    # Pre-compute RGBA base colors per track id (keyed on actual observed IDs)
    track_rgba: dict[int, tuple[float, float, float, float]] = {
        tid: track_color(tid) for tid in unique_ids
    }

    dynamic: list = []   # scatter artists removed each frame

    render_indices = list(range(0, n_frames, stride))
    print(f"[fov anim] {n_frames} source frames → {len(render_indices)} rendered @ {fps} fps (stride {stride}) → {partial.name}")
    try:
        with imageio.get_writer(str(partial), fps=fps, codec="libx264") as writer:
            for render_i, fi in enumerate(render_indices):
                if render_i == 0 or render_i == len(render_indices) - 1 or render_i % 50 == 0:
                    print(f"[fov anim] frame {render_i + 1}/{len(render_indices)}")

                # Remove previous scatter batch
                for a in dynamic:
                    a.remove()
                dynamic.clear()

                t_now = float(timestamps_s[fi]) if fi < len(timestamps_s) else fi * dt
                title.set_text(f"FOV  t = {t_now:.2f} s")

                # Collect all trail points into a single scatter call (per-point RGBA)
                thetas: list[float] = []
                ranges: list[float] = []
                sizes: list[float] = []
                colors: list[tuple[float, float, float, float]] = []
                current_thetas: list[float] = []
                current_ranges: list[float] = []
                current_colors: list[tuple[float, float, float, float]] = []

                for trail_fi in range(max(0, fi - trail_frames), fi + 1):
                    age = fi - trail_fi
                    frac = 1.0 - age / trail_frames
                    alpha = max(0.05, frac ** 1.5)
                    size = 6.0 + 54.0 * frac
                    for ta in track_aoas_per_frame[trail_fi]:
                        if ta.range_m > range_max_m:
                            continue
                        r, g, b, _ = track_rgba.get(ta.track_id, (1.0, 1.0, 1.0, 1.0))
                        thetas.append(float(np.deg2rad(ta.angle_deg)))
                        ranges.append(ta.range_m)
                        sizes.append(size)
                        colors.append((r, g, b, alpha))
                        if trail_fi == fi:
                            current_thetas.append(float(np.deg2rad(ta.angle_deg)))
                            current_ranges.append(ta.range_m)
                            current_colors.append((r, g, b, 1.0))

                if thetas:
                    sc = ax.scatter(
                        np.asarray(thetas, dtype=np.float32),
                        np.asarray(ranges, dtype=np.float32),
                        s=np.asarray(sizes, dtype=np.float32),
                        c=np.asarray(colors, dtype=np.float32),
                        linewidths=0,
                    )
                    dynamic.append(sc)
                if current_thetas:
                    sc_now = ax.scatter(
                        np.asarray(current_thetas, dtype=np.float32),
                        np.asarray(current_ranges, dtype=np.float32),
                        s=80.0,
                        c=np.asarray(current_colors, dtype=np.float32),
                        linewidths=0.8,
                        edgecolors="black",
                        zorder=5,
                    )
                    dynamic.append(sc_now)

                fig.canvas.draw()
                rgb = np.asarray(fig.canvas.buffer_rgba(), dtype=np.uint8)[..., :3].copy()
                writer.append_data(rgb)

        plt.close(fig)
        partial.replace(output_path)
        print(f"[fov anim] wrote {output_path.name}")
    except Exception:
        plt.close(fig)
        if partial.exists():
            partial.unlink()
        raise


def save_doppler_animation(
    preprocessed: PreprocessedSession,
    output_path: Path,
    fps: int = 15,
    window_s: float = 3.0,
    vel_limit_ms: float = 6.0,
    range_max_m: float | None = None,
    tracks: list[Track] | None = None,
) -> None:
    """Range-Doppler heatmap animation with DBSCAN+Kalman tracking overlay.

    Left panel: sliding-window STFT heatmap with CFAR detection dots (white)
    and confirmed DBSCAN track centroids (colored circles) overlaid.
    Right panel: confirmed track range histories vs time.
    """
    import imageio.v2 as imageio
    from .range_doppler import compute_range_doppler_frames, detect_on_rd_frame
    from .doppler_tracker import DopplerTracker

    _PALETTE = ["#ff4444", "#44ff88", "#4488ff", "#ffcc00", "#ff44ff", "#00ffff"]

    def _tcolor(tid: int) -> str:
        return _PALETTE[tid % len(_PALETTE)]

    output_path = Path(output_path)
    partial = output_path.with_name(f"{output_path.stem}.partial{output_path.suffix}")
    if partial.exists():
        partial.unlink()

    # --- signal ---------------------------------------------------------------
    cir = preprocessed.highpass_complex.astype(np.complex64)  # (F, T)
    fps_src = float(preprocessed.frame_rate_hz)
    range_axis = preprocessed.range_axis_m
    timestamps = preprocessed.session.timestamps_s

    effective_range_max_m = (
        float(range_max_m) if range_max_m is not None else float(np.max(range_axis))
    )
    tap_mask = (range_axis >= preprocessed.config.wall_clip_m) & (range_axis <= effective_range_max_m)
    cir_roi = cir[:, tap_mask]
    range_roi = range_axis[tap_mask]
    n_source, n_taps = cir_roi.shape

    if n_taps == 0:
        print("[doppler anim] no taps in range window — skipping")
        return

    # --- STFT parameters for visual rendering ---------------------------------
    nperseg = min(max(16, int(round(fps_src * window_s))), n_source)
    hann = np.hanning(nperseg).astype(np.float32)

    lam_m = 3e8 / float(preprocessed.config.carrier_frequency_hz)
    vel_per_hz = lam_m / 2.0
    freqs = np.fft.fftshift(np.fft.fftfreq(nperseg, d=1.0 / fps_src)).astype(np.float32)
    velocity_ms = freqs * vel_per_hz
    freq_limit_hz = float(vel_limit_ms) / max(float(vel_per_hz), 1e-12)
    freq_mask = np.abs(freqs) <= freq_limit_hz
    velocity_disp = velocity_ms[freq_mask]

    step = max(1, round(fps_src / fps))
    render_centers = list(range(nperseg, n_source, step))
    n_renders = len(render_centers)

    if n_renders == 0:
        print("[doppler anim] session too short — skipping")
        return

    # --- Phase 1: detection + tracking (coarse RD frames) --------------------
    from .retr_detector import RETRDetector

    _RETR_WEIGHTS = (
        Path(__file__).resolve().parents[2]
        / "radar-detection-transformer"
        / "logs"
        / "pretrained_model"
        / "p2s1_retr_detseg.pth"
    )

    rd_frames = compute_range_doppler_frames(preprocessed, n_fft=32, hop=16)

    if _RETR_WEIGHTS.exists():
        print("[doppler anim] RETR weights found — using RETR detector")
        try:
            _detector = RETRDetector(weights_path=_RETR_WEIGHTS, device="cpu")
            rd_dets_list = _detector.detect(rd_frames)
        except Exception as _exc:
            print(f"[doppler anim] RETR failed: {_exc} — falling back to CFAR")
            rd_dets_list = [detect_on_rd_frame(rdf) for rdf in rd_frames]
    else:
        print("[doppler anim] RETR weights not found — using CFAR detector")
        rd_dets_list = [detect_on_rd_frame(rdf) for rdf in rd_frames]

    dbscan_tracker = DopplerTracker(confirm_hits=2, max_misses=8, min_samples=1)
    rd_ts_list: list[float] = []
    rd_snaps: list[list] = []
    for rdf, dets in zip(rd_frames, rd_dets_list):
        snap = dbscan_tracker.update(rdf.frame_idx, rdf.timestamp_s, dets)
        rd_ts_list.append(rdf.timestamp_s)
        rd_snaps.append(snap)

    rd_ts_arr = np.array(rd_ts_list) if rd_ts_list else np.array([0.0])
    n_rd = len(rd_snaps)
    print(f"[doppler anim] {len(rd_frames)} RD frames → {n_rd} tracking steps")

    # Full confirmed track histories for the history panel
    final_confirmed = dbscan_tracker.confirmed_tracks_snapshot()
    track_histories = {
        t.track_id: (t.history_t, t.history_range_m)
        for t in final_confirmed
    }

    # --- calibrate colour limits ----------------------------------------------
    sample_mags: list[np.ndarray] = []
    for fi in render_centers[::max(1, n_renders // 20)]:
        seg = cir_roi[fi - nperseg : fi, :] * hann[:, None]
        spec = np.abs(np.fft.fftshift(np.fft.fft(seg, axis=0), axes=0)[freq_mask, :])
        sample_mags.append(spec.ravel())
    all_mag = np.concatenate(sample_mags)
    vmin_db = float(20.0 * np.log10(float(np.percentile(all_mag, 2)) + 1e-9))
    vmax_db = float(20.0 * np.log10(float(np.percentile(all_mag, 99)) + 1e-9))

    # --- build figure ---------------------------------------------------------
    fig, (ax_rd, ax_hist) = plt.subplots(
        1, 2, figsize=(14, 5), facecolor="black",
        gridspec_kw={"width_ratios": [2, 1]},
    )
    for ax in (ax_rd, ax_hist):
        ax.set_facecolor("#080818")
        ax.tick_params(colors="white")
        for spine in ax.spines.values():
            spine.set_edgecolor("#444444")

    # Left: heatmap
    im = ax_rd.imshow(
        np.zeros((n_taps, len(velocity_disp)), dtype=np.float32),
        aspect="auto", origin="lower",
        extent=[float(velocity_disp[0]), float(velocity_disp[-1]),
                float(range_roi[0]), float(range_roi[-1])],
        cmap="jet", vmin=vmin_db, vmax=vmax_db,
    )
    cbar = fig.colorbar(im, ax=ax_rd, pad=0.02, fraction=0.04)
    cbar.set_label("dB", color="white", fontsize=8)
    cbar.ax.yaxis.set_tick_params(color="white", labelcolor="white")
    ax_rd.set_xlabel("Radial velocity (m/s)", color="white", fontsize=10)
    ax_rd.set_ylabel("Range (m)", color="white", fontsize=10)
    ax_rd.set_xlim(-float(vel_limit_ms), float(vel_limit_ms))

    # Dynamic overlays
    scat_dets = ax_rd.scatter([], [], s=10, c="white", alpha=0.6, zorder=3, linewidths=0)
    scat_tent = ax_rd.scatter([], [], s=30, c="grey", marker="x", alpha=0.7, zorder=4)
    conf_scatter: dict[int, object] = {}  # track_id → PathCollection
    conf_labels: dict[int, object] = {}   # track_id → Text

    title = ax_rd.set_title("Range-Doppler  t = 0.00 s", color="white", fontsize=11, pad=6)

    # Right: track history
    t_start = float(timestamps[0]) if len(timestamps) else 0.0
    t_end = float(timestamps[-1]) if len(timestamps) else 1.0
    ax_hist.set_xlabel("Time (s)", color="white", fontsize=10)
    ax_hist.set_ylabel("Range (m)", color="white", fontsize=10)
    ax_hist.set_xlim(t_start, t_end)
    ax_hist.set_ylim(float(range_roi[0]), float(range_roi[-1]))
    ax_hist.set_title("Confirmed tracks — range", color="white", fontsize=10, pad=4)
    for tid, (ht, hr) in track_histories.items():
        ax_hist.plot(ht, hr, color=_tcolor(tid), alpha=0.25, linewidth=0.8)
    vline = ax_hist.axvline(t_start, color="white", linewidth=0.8, linestyle="--", alpha=0.6)

    fig.tight_layout(pad=1.2)
    fig.canvas.draw()

    print(f"[doppler anim] {n_source} src frames → {n_renders} rendered "
          f"@ {fps} fps (step {step}, window {window_s:.1f} s) → {partial.name}")
    try:
        with imageio.get_writer(str(partial), fps=fps, codec="libx264") as writer:
            for render_i, fi in enumerate(render_centers):
                if render_i == 0 or render_i == n_renders - 1 or render_i % 50 == 0:
                    print(f"[doppler anim] frame {render_i + 1}/{n_renders}")

                # Heatmap update
                seg = cir_roi[fi - nperseg : fi, :] * hann[:, None]
                spec = np.fft.fftshift(np.fft.fft(seg, axis=0), axes=0)
                mag_db = (20.0 * np.log10(np.abs(spec[freq_mask, :]).T + 1e-9)).astype(np.float32)
                im.set_data(mag_db)

                t_now = float(timestamps[fi]) if fi < len(timestamps) else fi / fps_src
                title.set_text(f"Range-Doppler  t = {t_now:.2f} s")
                vline.set_xdata([t_now, t_now])

                # Find closest DBSCAN tracking snapshot
                if n_rd > 0:
                    rd_idx = int(np.argmin(np.abs(rd_ts_arr - t_now)))
                    snap = rd_snaps[rd_idx]
                    dets_now = rd_dets_list[rd_idx]

                    # CFAR dots
                    if dets_now:
                        scat_dets.set_offsets(np.array([[d[1], d[0]] for d in dets_now]))
                    else:
                        scat_dets.set_offsets(np.empty((0, 2)))

                    # Confirmed tracks
                    conf_now = {t.track_id: t for t in snap if t.confirmed}
                    # Remove stale handles
                    for tid in list(conf_scatter.keys()):
                        if tid not in conf_now:
                            conf_scatter[tid].remove()
                            conf_labels[tid].remove()
                            del conf_scatter[tid]
                            del conf_labels[tid]
                    # Add/update
                    for tid, trk in conf_now.items():
                        vp = float(trk.state[1])   # velocity on x
                        rp = float(trk.state[0])   # range on y
                        c = _tcolor(tid)
                        if tid in conf_scatter:
                            conf_scatter[tid].set_offsets([[vp, rp]])
                            conf_labels[tid].set_position((vp, rp + 0.1))
                        else:
                            conf_scatter[tid] = ax_rd.scatter(
                                [vp], [rp], s=120, c=[c],
                                edgecolors="white", linewidths=1.0, zorder=5,
                            )
                            conf_labels[tid] = ax_rd.text(
                                vp, rp + 0.1, f"#{tid}",
                                color=c, fontsize=8, ha="center", zorder=6,
                            )

                    # Tentative tracks
                    tent = [(t.state[1], t.state[0]) for t in snap if not t.confirmed]
                    scat_tent.set_offsets(np.array(tent) if tent else np.empty((0, 2)))
                else:
                    scat_dets.set_offsets(np.empty((0, 2)))
                    scat_tent.set_offsets(np.empty((0, 2)))

                fig.canvas.draw()
                rgb = np.asarray(fig.canvas.buffer_rgba(), dtype=np.uint8)[..., :3].copy()
                writer.append_data(rgb)

        plt.close(fig)
        partial.replace(output_path)
        print(f"[doppler anim] wrote {output_path.name}")
    except Exception:
        plt.close(fig)
        if partial.exists():
            partial.unlink()
        raise
