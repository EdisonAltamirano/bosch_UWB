from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from .types import PreprocessedSession, SpectrogramResult


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


def save_range_doppler_plot(preprocessed: PreprocessedSession, output_path: Path) -> None:
    signal = preprocessed.highpass_complex  # (N, K)
    N = signal.shape[0]

    window = np.hanning(N).reshape(-1, 1)
    rdm = np.fft.fftshift(np.fft.fft(signal * window, axis=0), axes=0)
    rdm_db = (20.0 * np.log10(np.abs(rdm) + 1e-6)).astype(np.float32)

    doppler_hz = np.fft.fftshift(np.fft.fftfreq(N, d=1.0 / preprocessed.frame_rate_hz))
    c = 3e8
    fc = preprocessed.config.carrier_frequency_hz
    velocity_ms = doppler_hz * c / (2.0 * fc)

    range_axis = preprocessed.range_axis_m
    valid = range_axis >= preprocessed.config.wall_clip_m

    fig, ax = plt.subplots(figsize=(11, 6))
    im = ax.imshow(
        rdm_db[:, valid].T,
        aspect="auto",
        origin="lower",
        extent=[velocity_ms[0], velocity_ms[-1], range_axis[valid][0], range_axis[valid][-1]],
        cmap="plasma",
    )
    ax.axvline(0, color="white", linewidth=0.6, alpha=0.5, linestyle="--")
    ax.set_title(f"Range-Doppler map (path {preprocessed.selected_path}, fc={fc/1e9:.2f} GHz)")
    ax.set_xlabel("Radial velocity (m/s)  [negative = approaching, positive = receding]")
    ax.set_ylabel("Range (m)")
    fig.colorbar(im, ax=ax, label="Magnitude (dB)")
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
