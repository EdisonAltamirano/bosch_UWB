from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from .types import BreathingResult, CfarDetection, PreprocessedSession, SpectrogramResult, Track
from .visualization_utils import db_limits, jet_color, magnitude_to_db, power_ratio_to_db


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
    peak_ranges = preprocessed.peak_range_m_per_frame
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


def save_multi_peak_tracking_plot(
    preprocessed: PreprocessedSession,
    confirmed_tracks: list[Track],
    output_path: Path,
) -> None:
    """Range trajectories of all confirmed CFAR tracks over time."""
    fig, ax = plt.subplots(figsize=(12, 5))
    track_count = max(len(confirmed_tracks), 1)

    for i, track in enumerate(confirmed_tracks):
        if len(track.history_t) < 2:
            continue
        ax.plot(track.history_t, track.history_m,
                color=jet_color(i, track_count),
                linewidth=1.5,
                label=f"track {track.track_id}")

    ax.set_title("Confirmed track trajectories (CFAR + Kalman)")
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


def save_breathing_map(result: BreathingResult, output_path: Path) -> None:
    """Range x breathing-rate heatmap with the extracted rate annotated.

    Range on X, breathing rate (breaths/min) on Y, jet colormap — the breathing
    analog of the range-Doppler map.  The physiological band is shaded and the
    detected (range, rate) peak is marked.
    """
    import matplotlib.ticker as ticker

    rate_bpm = result.rate_bpm
    range_axis = result.range_axis_m
    map_db = result.power_db                        # (F, R)

    if map_db.size == 0 or range_axis.size == 0 or rate_bpm.size == 0:
        fig, ax = plt.subplots(figsize=(9, 5.5))
        ax.text(0.5, 0.5, "Insufficient data for breathing map",
                ha="center", va="center")
        fig.savefig(output_path, dpi=160)
        plt.close(fig)
        return

    vmin, vmax = db_limits(map_db)

    fig, ax = plt.subplots(figsize=(9, 5.5), facecolor="white")
    img = ax.pcolormesh(
        range_axis,
        rate_bpm,
        map_db,
        cmap="jet",
        vmin=vmin,
        vmax=vmax,
        shading="auto",
        rasterized=True,
    )
    cbar = fig.colorbar(img, ax=ax, pad=0.02)
    cbar.set_label("Magnitude (dB)", fontsize=11)
    cbar.ax.tick_params(labelsize=9)

    band_lo_bpm = result.band_hz[0] * 60.0
    band_hi_bpm = result.band_hz[1] * 60.0
    ax.axhspan(band_lo_bpm, band_hi_bpm, color="white", alpha=0.08, zorder=1)
    ax.axhline(band_lo_bpm, color="white", linewidth=0.5, linestyle=":", alpha=0.5)
    ax.axhline(band_hi_bpm, color="white", linewidth=0.5, linestyle=":", alpha=0.5)

    # Mark the detected breathing peak.
    if result.best_rate_bpm > 0:
        ax.plot(
            result.best_range_m, result.best_rate_bpm,
            marker="o", markersize=9, markerfacecolor="none",
            markeredgecolor="white", markeredgewidth=1.6, zorder=5,
        )
        ax.annotate(
            f"{result.best_rate_bpm:.1f} brpm\n@ {result.best_range_m:.2f} m\nSNR {result.snr_db:.1f} dB",
            xy=(result.best_range_m, result.best_rate_bpm),
            xytext=(0.98, 0.96), textcoords="axes fraction",
            ha="right", va="top", color="white", fontsize=9, fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="black", alpha=0.45,
                      edgecolor="white", linewidth=0.5),
        )

    ax.set_xlabel("Range (m)", fontsize=12)
    ax.set_ylabel("Breathing rate (breaths/min)", fontsize=12)
    ax.set_ylim(0, rate_bpm[-1])
    ax.yaxis.set_major_locator(ticker.MultipleLocator(6.0))
    ax.tick_params(axis="both", labelsize=9)
    for spine in ax.spines.values():
        spine.set_edgecolor("black")

    ax.set_title(
        f"Breathing map  (path {result.selected_path}, "
        f"fs = {result.frame_rate_hz:.1f} Hz)",
        fontsize=12,
    )
    fig.text(
        0.5, 0.01,
        "Slow-time spectrum of the clutter-suppressed CIR per range bin; "
        "shaded band = physiological breathing range.",
        ha="center", va="bottom", fontsize=8.5, color="#444444", style="italic",
    )
    fig.tight_layout(rect=[0, 0.04, 1, 1])
    fig.savefig(output_path, dpi=180, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def save_phase_iq_plot(
    preprocessed: PreprocessedSession,
    output_path: Path,
    tap_index: int | None = None,
) -> None:
    """Phase-over-time and I/Q trajectory of the dominant-tap complex CIR.

    The complex signal is first bandpass-filtered to the physiological breathing
    band so that out-of-band noise does not corrupt the phase estimate.

    Layout (2 rows × 3 columns):
      (a) Left col   — I over time (top), Q over time (bottom).
      (b) Centre col — I/Q phasor trajectory (spans both rows).
      (c) Right col  — wrapped phase in [-π, π] (top), magnitude (bottom).
    """
    cfg = preprocessed.config
    tap = preprocessed.dominant_tap if tap_index is None else int(tap_index)
    tap = int(np.clip(tap, 0, preprocessed.highpass_complex.shape[1] - 1))

    signal = preprocessed.highpass_complex[:, tap].astype(np.complex128)
    t = np.asarray(preprocessed.session.timestamps_s, dtype=np.float64)
    if t.size != signal.size:
        t = np.arange(signal.size, dtype=np.float64) / max(preprocessed.frame_rate_hz, 1e-9)
    range_m = float(preprocessed.range_axis_m[tap])
    fs = float(preprocessed.frame_rate_hz)

    # Bandpass-filter I and Q separately in the breathing band before computing
    # the phase — prevents out-of-band noise from causing wrapping artefacts.
    from scipy.signal import butter, filtfilt
    low_hz, high_hz = cfg.breathing_band_hz
    nyq = fs / 2.0
    low_n = max(low_hz / nyq, 1e-4)
    high_n = min(high_hz / nyq, 0.9999)
    if low_n < high_n and signal.size >= 30:
        order = min(4, signal.size // 15)
        order = max(order, 1)
        b, a = butter(order, [low_n, high_n], btype="band")
        sig_filt = (
            filtfilt(b, a, np.real(signal)) + 1j * filtfilt(b, a, np.imag(signal))
        ).astype(np.complex128)
    else:
        sig_filt = signal

    i_comp     = np.real(sig_filt)
    q_comp     = np.imag(sig_filt)
    phase      = np.angle(sig_filt)          # wrapped phase in [-π, π]
    magnitude  = np.abs(sig_filt)

    # ---- figure: 2 rows × 3 columns ----------------------------------------
    fig = plt.figure(figsize=(13, 5), facecolor="white")
    gs = fig.add_gridspec(
        2, 3,
        width_ratios=[1.0, 1.0, 1.0],
        hspace=0.10,
        wspace=0.35,
    )
    ax_i    = fig.add_subplot(gs[0, 0])
    ax_q    = fig.add_subplot(gs[1, 0], sharex=ax_i)
    ax_traj = fig.add_subplot(gs[:, 1])
    ax_ph   = fig.add_subplot(gs[0, 2], sharex=ax_i)
    ax_mag  = fig.add_subplot(gs[1, 2], sharex=ax_i)

    clr = "#1f77b4"   # single colour matching the reference style

    # --- (a) I over time ---
    ax_i.plot(t, i_comp, color=clr, linewidth=0.9)
    ax_i.set_ylabel("I")
    ax_i.set_title("a) I/Q over time", loc="left", fontsize=9)
    ax_i.grid(True, alpha=0.3)
    ax_i.tick_params(labelbottom=False)

    # --- (a) Q over time ---
    ax_q.plot(t, q_comp, color=clr, linewidth=0.9)
    ax_q.set_ylabel("Q")
    ax_q.set_xlabel("Time [s]")
    ax_q.grid(True, alpha=0.3)

    # --- (b) I/Q trajectory ---
    ax_traj.scatter(i_comp, q_comp, color=clr, s=6, linewidths=0)
    ax_traj.axhline(0, color="0.7", linewidth=0.5)
    ax_traj.axvline(0, color="0.7", linewidth=0.5)
    ax_traj.set_aspect("equal", adjustable="datalim")
    ax_traj.set_xlabel("I")
    ax_traj.set_ylabel("Q")
    ax_traj.set_title("b) I/Q trajectory", loc="left", fontsize=9)
    ax_traj.grid(True, alpha=0.3)

    # --- (c) wrapped phase in [-π, π] ---
    ax_ph.plot(t, phase, color=clr, linewidth=0.9)
    ax_ph.set_ylabel("Phase [rad]")
    ax_ph.set_yticks([-np.pi, 0, np.pi])
    ax_ph.set_yticklabels([r"$-\pi$", "0", r"$\pi$"])
    ax_ph.set_ylim(-np.pi * 1.15, np.pi * 1.15)
    ax_ph.set_title("c) Phase/magnitude", loc="left", fontsize=9)
    ax_ph.grid(True, alpha=0.3)
    ax_ph.tick_params(labelbottom=False)

    # --- (c) magnitude ---
    ax_mag.plot(t, magnitude, color=clr, linewidth=0.9)
    ax_mag.set_ylabel("Magnitude")
    ax_mag.set_xlabel("Time [s]")
    ax_mag.set_ylim(bottom=0)
    ax_mag.grid(True, alpha=0.3)

    fig.suptitle(
        f"path {preprocessed.selected_path}, tap {tap}, {range_m:.2f} m  "
        f"[bp {low_hz:.2f}–{high_hz:.2f} Hz]",
        fontsize=9,
        y=1.01,
    )

    fig.savefig(output_path, dpi=160, bbox_inches="tight", facecolor="white")
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
