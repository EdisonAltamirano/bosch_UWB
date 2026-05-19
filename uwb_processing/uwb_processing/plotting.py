from __future__ import annotations

import os
import warnings
from pathlib import Path

import cv2
import matplotlib

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
os.makedirs(os.environ["MPLCONFIGDIR"], exist_ok=True)

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg
from scipy.signal import find_peaks

from .preprocessing import linear_to_db
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


def _peaks_per_frame(
    preprocessed: PreprocessedSession,
) -> tuple[list[np.ndarray], list[float]]:
    """Per-frame local-max tap indices and ranges above peak_magnitude_threshold_db."""
    threshold_db = preprocessed.config.peak_magnitude_threshold_db
    min_sep = preprocessed.config.peak_min_separation_taps
    mag_db = linear_to_db(preprocessed.clutter_removed)
    ranges = preprocessed.range_axis_m
    valid = ranges >= preprocessed.config.wall_clip_m

    peak_taps_per_frame: list[np.ndarray] = []
    strongest_range_per_frame: list[float] = []

    for frame_db in mag_db:
        masked_db = frame_db.copy()
        masked_db[~valid] = -np.inf
        peaks, properties = find_peaks(
            masked_db,
            height=float(threshold_db),
            distance=max(1, int(min_sep)),
        )
        peak_taps_per_frame.append(peaks.astype(np.int32))
        if peaks.size:
            best = peaks[int(np.argmax(properties["peak_heights"]))]
            strongest_range_per_frame.append(float(ranges[best]))
        else:
            strongest_range_per_frame.append(float("nan"))

    return peak_taps_per_frame, strongest_range_per_frame


def save_peak_tracking_plot(preprocessed: PreprocessedSession, output_path: Path) -> None:
    threshold_db = preprocessed.config.peak_magnitude_threshold_db
    times = preprocessed.session.timestamps_s
    ranges = preprocessed.range_axis_m
    peak_taps_per_frame, strongest_ranges = _peaks_per_frame(preprocessed)

    fig, ax = plt.subplots(figsize=(12, 5))

    for index, range_m in enumerate(preprocessed.significant_ranges_m):
        ax.axhline(
            float(range_m),
            color="gray",
            linestyle=":",
            alpha=0.45,
            linewidth=0.8,
            label="Session-mean peaks" if index == 0 else None,
        )

    for time_s, peaks in zip(times, peak_taps_per_frame):
        if peaks.size:
            ax.scatter(
                np.full(peaks.size, time_s, dtype=np.float64),
                ranges[peaks],
                s=14,
                c="C0",
                alpha=0.45,
                linewidths=0,
            )

    ax.plot(
        times,
        strongest_ranges,
        color="C3",
        linewidth=1.2,
        label="Strongest peak per frame",
    )
    ax.scatter([], [], s=14, c="C0", alpha=0.6, label=f"All peaks ≥ {threshold_db:.0f} dB")
    ax.set_title(
        "Reflector ranges over time "
        f"(path {preprocessed.selected_path}, threshold {threshold_db:.0f} dB, "
        f"{preprocessed.significant_taps.size} session peaks)"
    )
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Range (m)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)
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


def _video_encode_fps(
    frame_rate_hz: float,
    n_frames: int,
    timestamps_s: np.ndarray,
    max_fps: float = 30.0,
) -> float:
    """Pick an MPEG-4-safe playback FPS (OpenCV mp4v fails when FPS is very high)."""
    if timestamps_s.size >= 2:
        duration_s = float(timestamps_s[-1] - timestamps_s[0])
        if duration_s > 0:
            effective_fps = n_frames / duration_s
        else:
            effective_fps = frame_rate_hz
    else:
        effective_fps = frame_rate_hz

    if not np.isfinite(effective_fps) or effective_fps <= 0:
        effective_fps = 10.0

    encode_fps = min(float(effective_fps), max_fps)
    encode_fps = max(encode_fps, 1.0)
    encode_fps = float(int(round(encode_fps)))

    if encode_fps < effective_fps - 1.0:
        warnings.warn(
            f"Video encode FPS capped to {encode_fps:.0f} Hz "
            f"(session effective rate ≈ {effective_fps:.1f} Hz) for MPEG-4 compatibility.",
            RuntimeWarning,
            stacklevel=2,
        )
    return encode_fps


def _open_video_writer(
    output_path: Path,
    fps: float,
    frame_size: tuple[int, int],
) -> tuple[cv2.VideoWriter, Path]:
    """Try a few codecs; return (writer, path actually opened)."""
    width, height = frame_size
    candidates = [
        (output_path, "mp4v"),
        (output_path.with_suffix(".avi"), "MJPG"),
        (output_path, "avc1"),
    ]
    errors: list[str] = []
    for path, fourcc_tag in candidates:
        writer = cv2.VideoWriter(
            str(path),
            cv2.VideoWriter_fourcc(*fourcc_tag),
            fps,
            (width, height),
        )
        if writer.isOpened():
            return writer, path
        writer.release()
        errors.append(f"{fourcc_tag}@{path.name}")
    raise RuntimeError(
        f"Failed to open video writer for {output_path} at {fps:.0f} FPS "
        f"({width}x{height}). Tried: {', '.join(errors)}"
    )


def save_range_doppler_video(
    preprocessed: PreprocessedSession,
    output_path: Path,
    fps: float | None = None,
    show_progress: bool = True,
) -> Path:
    """Build range-Doppler video: one RDM per acquisition frame (sliding Doppler window).

    Slow step: each frame renders a matplotlib figure (~0.1–0.2 s/frame typical).
    A 20 s capture at ~100 Hz (~2000 frames) can take on the order of 5–15 minutes.
    """
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
    timestamps_s = preprocessed.session.timestamps_s

    output_path = Path(output_path)
    if output_path.suffix.lower() not in {".mp4", ".avi"}:
        output_path = output_path.with_suffix(".mp4")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    if fps is not None:
        video_fps = max(1.0, float(int(round(fps))))
    else:
        video_fps = _video_encode_fps(preprocessed.frame_rate_hz, n_frames, timestamps_s)

    writer: cv2.VideoWriter | None = None
    opened_path = output_path

    frame_indices = range(n_frames)
    if show_progress:
        try:
            from tqdm import tqdm

            frame_indices = tqdm(
                frame_indices,
                desc="Range-Doppler video",
                unit="frame",
                total=n_frames,
            )
        except ImportError:
            pass

    try:
        for frame_index in frame_indices:
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
                writer, opened_path = _open_video_writer(
                    output_path,
                    video_fps,
                    (width, height),
                )
            writer.write(frame_bgr)
    finally:
        if writer is not None:
            writer.release()
    return opened_path


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
