from __future__ import annotations

from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg

from .types import PreprocessedSession, SpectrogramResult

_RDM_FIGSIZE = (11.0, 6.0)
_RDM_DPI = 120
_SPEED_OF_LIGHT_M_S = 3e8


def save_range_time_plot(preprocessed: PreprocessedSession, output_path: Path) -> None:
    frame_axis = preprocessed.session.timestamps_s

    # Crop to bins at or beyond wall_clip_m.  Bins below this threshold were
    # zeroed in preprocessing; including them would show 20*log10(1e-6)≈-120 dB
    # as a solid dark band and collapse the colormap range for the real signal.
    full_range = preprocessed.range_axis_m
    valid = full_range >= preprocessed.config.wall_clip_m
    range_axis = full_range[valid]

    raw = preprocessed.raw_magnitude[:, valid].T
    clutter = (20.0 * np.log10(preprocessed.clutter_removed[:, valid] + 1e-6)).T

    fig, axes = plt.subplots(2, 1, figsize=(12, 9), sharex=True)

    axes[0].imshow(
        raw,
        aspect="auto",
        origin="lower",
        extent=[frame_axis[0], frame_axis[-1], range_axis[0], range_axis[-1]],
        cmap="viridis",
    )
    axes[0].set_title(f"Raw range-time magnitude (path {preprocessed.selected_path})")
    axes[0].set_ylabel("Range (m)")

    im = axes[1].imshow(
        clutter,
        aspect="auto",
        origin="lower",
        extent=[frame_axis[0], frame_axis[-1], range_axis[0], range_axis[-1]],
        cmap="plasma",
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
    ax.plot(preprocessed.session.timestamps_s, peak_ranges, linewidth=1.0)
    ax.set_title("Dominant moving reflector range over time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Range (m)")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def compute_range_doppler_map(
    signal: np.ndarray,
    frame_rate_hz: float,
    carrier_frequency_hz: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Range-Doppler map from complex slow-time signal (N, K)."""
    n_frames = signal.shape[0]
    taper = np.hanning(n_frames).reshape(-1, 1)
    rdm = np.fft.fftshift(np.fft.fft(signal * taper, axis=0), axes=0)
    rdm_db = (20.0 * np.log10(np.abs(rdm) + 1e-6)).astype(np.float32)
    doppler_hz = np.fft.fftshift(np.fft.fftfreq(n_frames, d=1.0 / frame_rate_hz))
    velocity_ms = (doppler_hz * _SPEED_OF_LIGHT_M_S / (2.0 * carrier_frequency_hz)).astype(np.float32)
    return rdm_db, velocity_ms


def _doppler_window_frames(preprocessed: PreprocessedSession) -> int:
    win = int(round(preprocessed.config.doppler_window_s * preprocessed.frame_rate_hz))
    win = max(8, win)
    return win if win % 2 == 1 else win + 1


def _sliding_window_at_center(
    signal: np.ndarray,
    center_index: int,
    win_frames: int,
) -> np.ndarray:
    half = win_frames // 2
    start = center_index - half
    windowed = np.zeros((win_frames, signal.shape[1]), dtype=signal.dtype)
    for dst in range(win_frames):
        src = start + dst
        if 0 <= src < signal.shape[0]:
            windowed[dst] = signal[src]
    return windowed


def _range_doppler_color_limits(
    preprocessed: PreprocessedSession,
    range_valid: np.ndarray,
) -> tuple[float, float]:
    rdm_db, _ = compute_range_doppler_map(
        preprocessed.highpass_complex,
        preprocessed.frame_rate_hz,
        preprocessed.config.carrier_frequency_hz,
    )
    values = rdm_db[:, range_valid]
    return float(np.percentile(values, 2)), float(np.percentile(values, 98))


def _render_range_doppler_frame(
    rdm_db: np.ndarray,
    velocity_ms: np.ndarray,
    range_axis: np.ndarray,
    range_valid: np.ndarray,
    vmin: float,
    vmax: float,
    title: str,
) -> np.ndarray:
    fig, ax = plt.subplots(figsize=_RDM_FIGSIZE)
    im = ax.imshow(
        rdm_db[:, range_valid].T,
        aspect="auto",
        origin="lower",
        extent=[
            float(velocity_ms[0]),
            float(velocity_ms[-1]),
            float(range_axis[range_valid][0]),
            float(range_axis[range_valid][-1]),
        ],
        cmap="plasma",
        vmin=vmin,
        vmax=vmax,
    )
    ax.axvline(0, color="white", linewidth=0.6, alpha=0.5, linestyle="--")
    ax.set_title(title)
    ax.set_xlabel("Radial velocity (m/s)  [negative = approaching, positive = receding]")
    ax.set_ylabel("Range (m)")
    fig.colorbar(im, ax=ax, label="Magnitude (dB)")
    fig.tight_layout()

    canvas = FigureCanvasAgg(fig)
    canvas.draw()
    rgba = np.asarray(canvas.buffer_rgba(), dtype=np.uint8)
    plt.close(fig)
    return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)


def save_range_doppler_plot(preprocessed: PreprocessedSession, output_path: Path) -> None:
    """Session-wide range-Doppler still image (full capture FFT)."""
    signal = preprocessed.highpass_complex
    rdm_db, velocity_ms = compute_range_doppler_map(
        signal,
        preprocessed.frame_rate_hz,
        preprocessed.config.carrier_frequency_hz,
    )
    range_axis = preprocessed.range_axis_m
    range_valid = range_axis >= preprocessed.config.wall_clip_m
    vmin, vmax = _range_doppler_color_limits(preprocessed, range_valid)
    fc = preprocessed.config.carrier_frequency_hz
    title = f"Range-Doppler map (path {preprocessed.selected_path}, fc={fc/1e9:.2f} GHz)"
    frame_bgr = _render_range_doppler_frame(
        rdm_db, velocity_ms, range_axis, range_valid, vmin, vmax, title
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), frame_bgr)


def save_range_doppler_video(
    preprocessed: PreprocessedSession,
    output_path: Path,
    fps: float | None = None,
) -> None:
    """Build range-Doppler.mp4: one RDM per acquisition frame (sliding Doppler window)."""
    signal = preprocessed.highpass_complex
    n_frames = signal.shape[0]
    if n_frames == 0:
        raise ValueError("Cannot build range-Doppler video: session has no frames.")

    win_frames = min(_doppler_window_frames(preprocessed), n_frames)
    if win_frames % 2 == 0:
        win_frames -= 1
    win_frames = max(win_frames, 3)

    range_axis = preprocessed.range_axis_m
    range_valid = range_axis >= preprocessed.config.wall_clip_m
    vmin, vmax = _range_doppler_color_limits(preprocessed, range_valid)
    fc = preprocessed.config.carrier_frequency_hz
    video_fps = fps if fps is not None else preprocessed.frame_rate_hz

    output_path = Path(output_path)
    if output_path.suffix.lower() != ".mp4":
        output_path = output_path.with_suffix(".mp4")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    writer: cv2.VideoWriter | None = None
    timestamps_s = preprocessed.session.timestamps_s

    try:
        for frame_index in range(n_frames):
            windowed = _sliding_window_at_center(signal, frame_index, win_frames)
            rdm_db, velocity_ms = compute_range_doppler_map(
                windowed,
                preprocessed.frame_rate_hz,
                fc,
            )
            time_s = float(timestamps_s[frame_index]) if timestamps_s.size > frame_index else frame_index / video_fps
            title = (
                f"Range-Doppler @ t={time_s:.2f}s "
                f"(path {preprocessed.selected_path}, fc={fc/1e9:.2f} GHz, "
                f"window={win_frames} frames)"
            )
            frame_bgr = _render_range_doppler_frame(
                rdm_db, velocity_ms, range_axis, range_valid, vmin, vmax, title
            )
            if writer is None:
                height, width = frame_bgr.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                writer = cv2.VideoWriter(str(output_path), fourcc, float(video_fps), (width, height))
                if not writer.isOpened():
                    raise RuntimeError(f"Failed to open video writer for {output_path}")
            writer.write(frame_bgr)
    finally:
        if writer is not None:
            writer.release()


def save_spectrogram_plot(
    spectrogram: SpectrogramResult,
    output_path: Path,
    frequency_limit_hz: float,
) -> None:
    fig, ax = plt.subplots(figsize=(11, 5))
    valid = np.abs(spectrogram.frequencies_hz) <= frequency_limit_hz
    freq = spectrogram.frequencies_hz[valid]
    mag = spectrogram.magnitude_db[valid]
    mesh = ax.pcolormesh(spectrogram.times_s, freq, mag, shading="auto", cmap="magma")
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
